"""Proxy for a XacroObject that reads a xacro file and outputs a xacro file
including it.

Can read:
- xacro file containing only macro definitions
- xacro file generating a full-feature URDF
- URDF file

Generates a xacro file that includes the input xacro, add a transform from
"world" to the root link of the robot (if any), and use the macro. Whether a
robot, a workcell, a static object is generated from the xacro depends on the
input xacro and the chosen macro from this xacro file.

The object type is called XacroObject to avoid confusion with the ROS package
xacro (and Python module xacro).

"""

from __future__ import annotations

from copy import copy
from typing import Any, ForwardRef, List, Optional
from xml.dom.minidom import Document
from xml.dom.minidom import Element
from xml.dom.minidom import parseString
from xml.parsers.expat import ExpatError
import xml.etree.ElementTree as et

import FreeCAD as fc

from xacro import Macro

from .freecad_utils import ProxyBase
from .freecad_utils import add_property
from .freecad_utils import is_group
from .freecad_utils import warn
from .freecad_utils import get_valid_property_name
from .ros.utils import abs_path_from_ros_path
from .ros.utils import ros_path_from_abs_path
from .wb_utils import ICON_PATH
from .wb_utils import is_robot
from .wb_utils import is_workcell
from .wb_utils import ros_name
try:
    from .robot_from_urdf import robot_from_urdf
    from .urdf_loader import UrdfLoader
    from .xacro_loader import XacroLoader
    from urdf_parser_py.urdf import Robot as UrdfRobot
except ImportError as e:
    # TODO: Warn the user more nicely.
    # TODO: provide alternates on import error
    warn(str(e))
    UrdfRobot = Any

# Stubs and typing hints.
from .link import Link as CrossLink  # A Cross::Link, i.e. a DocumentObject with Proxy "Link". # noqa: E501
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
from .xacro_object import XacroObject as CrossXacroObject  # A Cross::XacroObject, i.e. a DocumentObject with Proxy "XacroObject". # noqa: E501
DO = fc.DocumentObject
DOList = List[DO]
VPDO = ForwardRef('FreeCADGui.ViewProviderDocumentObject')  # Don't want to import FreeCADGui here. # noqa: E501


def _clear_robot(obj: CrossRobot) -> None:
    """Delete all joints and links from a robot."""
    doc = obj.Document
    # Let robot clean its generated objects.
    if obj.ViewObject:
        obj.ViewObject.ShowReal = False
        obj.ViewObject.ShowVisual = False
        obj.ViewObject.ShowCollision = False
    # Joints first.
    for joint in obj.Proxy.get_joints():
        joint.Label = 'to_be_removed'
        doc.removeObject(joint.Name)
    for link in obj.Proxy.get_links():
        # If without GUI. Group should be empty with GUI.
        for fc_link in link.Group:
            fc_link.Label = 'to_be_removed'
            doc.removeObject(fc_link.Name)
        link.Label = 'to_be_removed'
        doc.removeObject(link.Name)
    # Created objects.
    obj.Proxy.delete_created_objects()


class XacroObjectProxy(ProxyBase):
    """The XacroObject proxy."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Cross::XacroObject'

    def __init__(self, obj: CrossXacroObject):
        super().__init__('xacro_object', [
            'Group',
            'InputFile',
            'MainMacro',
            'Placement',
            '_Type',
            ])
        obj.Proxy = self
        self.xacro_object = obj

        # List of macro parameters, dict[name: prop_name], where `name` is the
        # parameter name as defined in the xacro file and `prop_name` is the
        # name of the FreeCAD property.
        self.param_properties: dict[str, str] = {}

        # Keep track of the old generated xacro, to avoid rebuilding the robot.
        if not hasattr(self, '_old_xacro_file_content'):
            self._old_xacro_file_content = ''
            self._urdf_robot: Optional[UrdfRobot] = None

        # The root link of the generated URDF.
        if not hasattr(self, '_root_link'):
            self._root_link = ''

        self.init_extensions(obj)
        self.init_properties(obj)

    @property
    def root_link(self) -> str:
        return str(self._root_link)

    def init_extensions(self, obj: CrossXacroObject) -> None:
        # Need a group to put the generated robot in.
        # obj.addExtension('App::GroupExtensionPython')
        obj.addExtension('App::GeoFeatureGroupExtensionPython')

        # Managed in self.reset_group().
        obj.setPropertyStatus('Group', ['ReadOnly', 'Hidden'])

    def init_properties(self, obj: CrossXacroObject):
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        add_property(obj, 'App::PropertyFile', 'InputFile', 'Input',
                     'The source xacro or URDF file, in'
                     ' `package://pkg/rel_path` format')
        add_property(obj, 'App::PropertyEnumeration', 'MainMacro', 'Input',
                     'The macro to use')

        # The computed placement (or the placement defined by the user if no
        # attachment mode is defined). This is only
        # when using `Part::AttachExtensionPython`.
        add_property(obj, 'App::PropertyPlacement', 'Placement',
                     'Base', 'Placement')

        self._toggle_editor_mode()

    def has_link(self, link_name: str) -> bool:
        """Return True if the link belongs to the xacro object.

        The link_name is the full name after macro expansion.

        """
        if not self._urdf_robot:
            return False
        return link_name in self._urdf_robot.link_map

    def get_link(self, link_name: str) -> Optional[CrossLink]:
        """Return True if the link belongs to the xacro object.

        The link_name is the full name after macro expansion.

        """
        for link in self.get_links():
            if ros_name(link) == link_name:
                return link
        return None

    def get_links(self) -> list[CrossLink]:
        if not self.is_execute_ready():
            return []
        robot = self.get_robot()
        if not robot:
            return []
        if not hasattr(robot, 'Proxy'):
            return []
        return robot.Proxy.get_links()

    def get_link_placement(self, link_name: str) -> Optional[fc.Placement]:
        """Return the link placement relative to the URDF root."""
        if not self.has_link(link_name):
            return None
        for link in self.get_links():
            if ros_name(link) == link_name:
                return link.Placement
        return None

    def execute(self, obj: CrossXacroObject) -> None:
        """Update the embedded robot.

        Called on recompute(), this method is mandatory for scripted objects.

        """
        if not self.is_execute_ready():
            return
        if obj is not self.xacro_object:
            raise ValueError('Proxied object was changed')
            return
        xo: CrossXacroObject = self.xacro_object
        if not xo.InputFile:
            self._root_link = ''
            return
        input_path = abs_path_from_ros_path(xo.InputFile)
        self.xacro = XacroLoader.load_from_file(input_path)
        macro_names = self.xacro.get_macro_names()
        if xo.getEnumerationsOfProperty('MainMacro') != macro_names:
            xo.MainMacro = macro_names
        self._toggle_editor_mode()
        self._add_param_properties()
        self.reset_group()

    def onChanged(self, obj: CrossXacroObject, prop: str) -> None:
        if prop == 'InputFile':
            if obj.InputFile.startswith('package://'):
                # Already in the correct form.
                return
            ros_path = ros_path_from_abs_path(obj.InputFile)
            if ros_path and (ros_path != obj.InputFile):
                obj.InputFile = ros_path
            elif (ros_path is None) and (obj.InputFile != ''):
                obj.InputFile = ''
        if prop == 'Placement':
            robot = self.get_robot()
            if robot and (robot.Placement != obj.Placement):
                # Avoid recursive recompute.
                robot.Placement = obj.Placement

    def onDocumentRestored(self, obj: CrossXacroObject):
        """Restore attributes because __init__ is not called on restore.

        This method is called on document restore, i.e. when the document is
        opened.

        """
        self.__init__(obj)
        self._fix_lost_fc_links()
        self.execute(obj)

    def __getstate__(self):
        return self.Type,

    def __setstate__(self, state):
        if state:
            self.Type, = state

    def reset_group(self):
        """Rebuild the CrossRobot object."""
        if not self.is_execute_ready():
            return
        xo: CrossXacroObject = self.xacro_object
        new_robot = self._generate_robot()
        if new_robot and xo.Group:
            old_robot = self.get_robot()
            if old_robot:
                # Delete the old robot.
                _clear_robot(old_robot)
                # Necessary to free the label for the new object object because
                # the object is removed later.
                old_robot.Label = 'to_be_removed'
                xo.Document.removeObject(old_robot.Name)
        if new_robot:
            new_robot.Label = f'Robot_of_{xo.Label}'
            # Add the new robot.
            # We add the robot by setting `Group` because of the bug described
            # below. When solved, use next line instead.
            # xo.addObject(new_robot)
            # Clean-up other objects.
            # The xacro objects contains other objects for some reason, remove
            # them from the group.
            # TODO: not nice, try to solve out why this is necessary.
            xo.Group = [new_robot]

    def export_urdf(self) -> Optional[et.Element]:
        """Export the xacro object as URDF."""
        if not self.is_execute_ready():
            return None
        xo: CrossXacroObject = self.xacro_object

        # Generate the URDF to obtain the root link.
        robot_name = ros_name(xo)
        params = self._get_xacro_params()

        # Generate the xacro xml document.
        xacro_xml = self.xacro.to_xml(robot_name, xo.MainMacro, params)

        return et.fromstring(xacro_xml.toxml())

    def get_link_names(self) -> list[str]:
        if not self._urdf_robot:
            return []
        return [link.name for link in self._urdf_robot.links]

    def get_robot(self) -> Optional[CrossRobot]:
        if ((not hasattr(self, 'xacro_object'))
                or (not hasattr(self.xacro_object, 'Group'))):
            return None
        for obj in self.xacro_object.Group:
            if is_robot(obj):
                return obj
        return None

    def _toggle_editor_mode(self) -> None:
        """Show/hide properties."""
        # Hide until an input file is given and macros are defined.
        xo: CrossXacroObject = self.xacro_object
        if (hasattr(xo, 'InputFile')
                and xo.InputFile
                and hasattr(self, 'xacro')
                and self.xacro
                and self.xacro.get_macro_names()):
            # Show property `MainMacro`.
            xo.setEditorMode('MainMacro', [])
        else:
            xo.setPropertyStatus('MainMacro', ['Hidden'])

    def _add_param_properties(self) -> None:
        if not self.is_execute_ready():
            return
        # Get old parameters to clear old parameters later on.
        old_param_properties: dict[str, str] = copy(self.param_properties)

        self.param_properties.clear()
        xo: CrossXacroObject = self.xacro_object
        if xo.MainMacro is None:
            # With pure URDF files.
            return
        macro: Macro = self.xacro.macros[xo.MainMacro]
        for name in macro.params:
            prop_name = get_valid_property_name(name)
            self.param_properties[name] = prop_name
            if name.startswith('*'):
                help = f'Macro parameter "{name}", must be a valid xml tag'
            else:
                help = f'Macro parameter "{name}"'
            if name in macro.defaultmap:
                # Default given.
                add_property(xo, 'App::PropertyString', prop_name, 'Input',
                             help,
                             macro.defaultmap[name][1])
            else:
                # No default.
                add_property(xo, 'App::PropertyString', prop_name, 'Input',
                             help)

        # Clear old parameters.
        for prop_name in old_param_properties.values():
            if prop_name not in self.param_properties.values():
                xo.removeProperty(prop_name)

    def _generate_robot(self) -> Optional[CrossRobot]:
        if not self.is_execute_ready():
            return None
        params = self._get_xacro_params()
        xo: CrossXacroObject = self.xacro_object
        xacro_txt = self.xacro.to_string(ros_name(xo), xo.MainMacro, params)
        if xacro_txt == self._old_xacro_file_content:
            return None
        self._old_xacro_file_content = xacro_txt
        self._urdf_robot = self._generate_urdf(f'{ros_name(xo)}_robot', xo.MainMacro, params)
        self._root_link = self._urdf_robot.get_root()
        robot = robot_from_urdf(xo.Document, self._urdf_robot)
        robot.Placement = self.xacro_object.Placement
        return robot

    def _generate_urdf(self,
                       robot_name: str,
                       macro: Optional[str] = '',
                       params: Optional[dict[str, str]] = None,
                       ) -> UrdfRobot:
        params = {} if params is None else params
        urdf_txt = self.xacro.to_urdf_string(robot_name, macro, params)
        urdf_robot = UrdfLoader.load_from_string(urdf_txt)
        return urdf_robot

    def _get_xacro_params(self) -> dict[str, [Any | Element]]:
        """Return the dictionnary of macro parameters for URDF generation."""
        if not hasattr(self, 'xacro_object'):
            return {}
        xo: CrossXacroObject = self.xacro_object

        params: dict[str, [Any | Element]] = {}
        for xacro_param_name, prop_name in self.param_properties.items():
            str_value = xo.getPropertyByName(prop_name)
            if xacro_param_name.startswith('*'):
                try:
                    value = parseString(str_value).documentElement
                except ExpatError as e:
                    value = Document().createComment('PARSING_ERROR'
                                                     f' in xacro parameter"{xacro_param_name}"'
                                                     f' of macro "{xo.MainMacro}":'
                                                     f' "{e}"')
            else:
                value = str_value
            params[xacro_param_name] = value
        return params

    def _fix_lost_fc_links(self) -> None:
        """Fix children lost on restore."""
        if not self.is_execute_ready():
            return
        xo = self.xacro_object
        # Keep track of objects that are in a group because we need to put them
        # back into this group. This is the case for parts in the group
        # `URDF Parts` created by this workbench and that FreeCAD decides to
        # put into `xo.Group` for some reason.
        objects_in_group: DOList = []
        for obj in xo.Document.Objects:
            if not hasattr(obj, 'InList'):
                continue
            if any([is_group(obj) for obj in obj.InList]):
                objects_in_group.append(obj)

        for obj in xo.Document.Objects:
            if (not hasattr(obj, 'InList')) or (len(obj.InList) != 1):
                continue
            potential_self = obj.InList[0]
            if ((obj is xo)
                    or (potential_self is not xo)
                    or (obj in xo.Group)
                    or (not is_robot(obj))):
                continue
            xo.addObject(obj)

        # Put back objects that were in a group by removing them from
        # `xo.Group`.
        for o in objects_in_group:
            xo.removeObject(o)


class _ViewProviderXacroObject(ProxyBase):
    """A view provider for the CROSS XacroObject object."""

    def __init__(self, vobj: VPDO):
        super().__init__('view_object', [
            'Visibility',
            ])
        vobj.Proxy = self

    def getIcon(self):
        # Implementation note: "return 'xacro.svg'" works only after
        # workbench activation in GUI.
        return str(ICON_PATH / 'xacro.svg')

    def attach(self, vobj: VPDO):
        self.ViewObject = vobj
        # vobj.addExtension('Gui::ViewProviderGroupExtensionPython')
        vobj.addExtension('Gui::ViewProviderGeoFeatureGroupExtensionPython')

    def updateData(self, obj: CrossXacroObject, prop: str):
        return

    def onChanged(self, vobj: VPDO, prop: str):
        pass

    def doubleClicked(self, vobj: VPDO):
        return True

    def setEdit(self, vobj: VPDO, mode):
        return False

    def unsetEdit(self, vobj: VPDO, mode):
        return

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


def make_xacro_object(name, doc: Optional[fc.Document] = None) -> CrossXacroObject:
    """Add a Cross::XacroObject to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    # xacro_obj = doc.addObject('App::FeaturePython', name)
    xacro_obj: CrossXacroObject = doc.addObject('Part::FeaturePython', name)
    XacroObjectProxy(xacro_obj)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        import FreeCADGui as fcgui

        _ViewProviderXacroObject(xacro_obj.ViewObject)

        # Make `xacro_obj` part of the selected `Cross::Workcell`.
        sel = fcgui.Selection.getSelection()
        if sel:
            candidate = sel[0]
            if (is_robot(candidate)
                    or is_workcell(candidate)):
                xacro_obj.adjustRelativeLinks(candidate)
                candidate.addObject(xacro_obj)

    doc.recompute()
    return xacro_obj
