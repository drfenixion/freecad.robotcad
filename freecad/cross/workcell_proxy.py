"""Proxy for Cross::Workcell FreeCAD objects

A workcell is a combination of CrossXacroObject and CrossJoint

A Cross::Workcell allows to combine existing URDF and xacro file to generate a
single robot description (or more generally a workcell description).
Joints must be defined between the included CrossXacroObject.

"""

from __future__ import annotations

from typing import ForwardRef, Optional
import xml.etree.ElementTree as et

import FreeCAD as fc

from .freecad_utils import ProxyBase
from .freecad_utils import add_property
from .freecad_utils import is_same_placement
from .freecad_utils import warn
from .gui_utils import tr
from .ros.utils import split_package_path
from .ui.file_overwrite_confirmation_dialog import FileOverwriteConfirmationDialog
from .urdf_utils import urdf_origin_from_placement
from .urdf_utils import xml_comment_element
from .utils import get_valid_filename
from .utils import save_xml
from .wb_utils import ICON_PATH
from .wb_utils import export_templates
from .wb_utils import get_joints
from .wb_utils import get_rel_and_abs_path
from .wb_utils import get_valid_urdf_name
from .wb_utils import get_xacro_chains
from .wb_utils import get_xacro_object_attachments
from .wb_utils import get_xacro_objects
from .wb_utils import remove_ros_workspace
from .wb_utils import ros_name

# Stubs and typing hints.
from .joint import Joint as CrossJoint  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint". # noqa: E501
from .xacro_object import XacroObject as CrossXacroObject  # A Cross::XacroObject, i.e. a DocumentObject with Proxy "XacroObject". # noqa: E501
from .workcell import Workcell as CrossWorkcell  # A Cross::Workcell, i.e. a DocumentObject with Proxy "Workcell". # noqa: E501
VPDO = ForwardRef('FreeCADGui.ViewProviderDocumentObject')  # Don't want to import FreeCADGui here. # noqa: E501


class WorkcellProxy(ProxyBase):
    """Proxy for CROSS::Workcell objects."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Cross::Workcell'

    def __init__(self, obj: CrossWorkcell):
        super().__init__('workcell', [
            '_Type',
            'OutputPath',
            'RootLink',
            ])
        obj.Proxy = self
        self.workcell = obj
        self.init_properties(obj)

    def init_properties(self, obj: CrossWorkcell) -> None:
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        add_property(obj, 'App::PropertyPath', 'OutputPath', 'Export',
                     'The path to the ROS package to export files to')

        add_property(obj, 'App::PropertyString', 'RootLink', 'Elements',
                     'The root link of the workcell, leave empty for'
                     ' a single mobile robot', 'world')

    def execute(self, obj: CrossWorkcell) -> None:
        self.set_joint_enum()
        self.place_xacro_objects()

    def onBeforeChange(self, obj: CrossWorkcell, prop: str) -> None:
        # TODO: save the old ros_name and update all joints that used it.
        pass

    def onChanged(self, obj: CrossWorkcell, prop: str) -> None:
        if prop == 'OutputPath':
            rel_path = remove_ros_workspace(obj.OutputPath)
            if rel_path != obj.OutputPath:
                obj.OutputPath = rel_path

    def onDocumentRestored(self, obj: CrossWorkcell) -> None:
        self.__init__(obj)

    def dumps(self):
        return self.Type,

    def __getstate__(self):
        # Deprecated.
        return self.dumps()

    def loads(self, state):
        if state:
            self.Type, = state

    def __setstate__(self, state):
        # Deprecated.
        return self.loads(state)

    def get_xacro_objects(self) -> list[CrossXacroObject]:
        if not self.is_execute_ready():
            return []
        return get_xacro_objects(self.workcell.Group)

    def get_joints(self) -> list[CrossJoint]:
        if not self.is_execute_ready():
            return []
        return get_joints(self.workcell.Group)

    def set_joint_enum(self) -> None:
        """Set the enum for Child and Parent of all joints."""
        if not self.is_execute_ready():
            return
        # We add the empty string to show that the child or parent
        # was not set yet.
        child_links: list[str] = ['']
        parent_links: list[str] = ['']
        if self.workcell.RootLink:
            parent_links.append(self.workcell.RootLink)
        for xacro_object in self.get_xacro_objects():
            child_links.append(xacro_object.Proxy.root_link)
            parent_links += xacro_object.Proxy.get_link_names()
        for joint in self.get_joints():
            # Implementation note: setting to a list sets the enumeration.
            if joint.getEnumerationsOfProperty('Child') != child_links:
                # Avoid recursive recompute.
                # Doesn't change the value if old value in the new enum.
                joint.Child = child_links
            if joint.getEnumerationsOfProperty('Parent') != parent_links:
                # Avoid recursive recompute.
                # Doesn't change the value if old value in the new enum.
                joint.Parent = parent_links

    def get_xacro_object_with_link(self, link_name: str) -> Optional[CrossXacroObject]:
        """Return the xacro object containing a given link."""
        for xacro_object in self.get_xacro_objects():
            if xacro_object.Proxy.has_link(link_name):
                return xacro_object
        return None

    def place_xacro_objects(self) -> None:
        """Set the `Placement` of all xacro objects."""
        for chain in get_xacro_chains(self.get_xacro_objects(), self.get_joints()):
            placement = fc.Placement()
            for attachment in chain:
                xo = attachment.xacro_object
                joint = attachment.attached_by
                link = attachment.attached_to
                if link:
                    placement = link.Placement
                if joint:
                    placement *= joint.Origin
                    if joint.Placement != placement:
                        # Avoid recursive recompute.
                        joint.Placement = placement
                    placement *= joint.Proxy.get_actuation_placement()
                if not is_same_placement(xo.Placement, placement):
                    # Avoid recursive recompute.
                    xo.Placement = placement

    def export_urdf(self, interactive: bool = False) -> Optional[et.Element]:
        if not self.is_execute_ready():
            return None
        obj: CrossWorkcell = self.workcell
        if not obj.OutputPath:
            # TODO: ask the user for OutputPath.
            warn('Property `OutputPath` cannot be empty', True)
            return None

        robot_et = et.fromstring('<robot/>')
        robot_et.attrib['name'] = get_valid_urdf_name(ros_name(obj))
        robot_et.append(xml_comment_element(
            'Generated by CROSS, a ROS Workbench for FreeCAD ('
            'https://github.com/galou/freecad.cross)'))

        xacro_objects = self.get_xacro_objects()
        joints = self.get_joints()
        attachments = get_xacro_object_attachments(xacro_objects, joints)

        # Check that no more than one XacroObject has no parent and whether
        # root link is used in one of the XacroObject.
        xacros_wo_parent: list[CrossXacroObject] = []
        for attachment in attachments:
            joint = attachment.attached_by
            if joint is None:
                xacros_wo_parent.append(attachment.xacro_object)

        if obj.RootLink:
            workcell_root_link = obj.RootLink
        else:
            workcell_root_link = 'NO_ROOT_LINK_SET'

        add_root_link = (((len(xacros_wo_parent) == 1) and obj.RootLink)
                         or (len(xacros_wo_parent) > 1))
        # Warn the user about the added joints.
        if len(xacros_wo_parent) > 1:
            warn('More than one XacroObject has no parent'
                 f', adding joints between link `{workcell_root_link}` and'
                 f' `{"`, `".join([x.Label for x in xacros_wo_parent])}`',
                 True)

        # Add the root link.
        if add_root_link:
            world_link_et = et.fromstring('<link/>')
            world_link_et.attrib['name'] = workcell_root_link
            robot_et.append(world_link_et)

        includes: list[et.Element] = []
        for attachment in attachments:
            xacro_object = attachment.xacro_object
            joint = attachment.attached_by

            # Add the xacro's children.
            xacro_et = xacro_object.Proxy.export_urdf()
            for child_et in xacro_et:
                if ((child_et.tag == 'xmlns:include')
                        and (child_et.attrib['filename'] not in includes)):
                    includes.append(child_et.attrib['filename'])
                robot_et.append(child_et)

            # Add the joint attaching the xacro.
            if joint or add_root_link:
                joint_et = et.fromstring('<joint/>')
                if joint:
                    parent_link = f'{joint.Parent}'
                    child_link = f'{joint.Child}'
                    origin = joint.Origin
                else:
                    parent_link = workcell_root_link
                    root_link = xacro_object.Proxy.root_link
                    child_link = root_link
                    origin = xacro_object.Placement
                joint_et.attrib['name'] = f'{parent_link}_to_{child_link}'
                joint_et.attrib['type'] = 'fixed'
                parent_et = et.fromstring('<parent/>')
                parent_et.attrib['link'] = parent_link
                joint_et.append(parent_et)
                child_et = et.fromstring('<child/>')
                child_et.attrib['link'] = child_link
                joint_et.append(child_et)
                origin_et = urdf_origin_from_placement(origin)
                joint_et.append(origin_et)
                robot_et.append(joint_et)

        template_files = [
            'package.xml',
            'CMakeLists.txt',
            'launch/display.launch.py',
            'rviz/robot_description.rviz',
            ]

        write_files = template_files + [
                'meshes/',
                'urdf/',
                ]

        # Write out files.
        # TODO: also accept OutputPath as package name in $ROS_WORKSPACE/src.
        p, output_path = get_rel_and_abs_path(obj.OutputPath)
        if p != obj.OutputPath:
            obj.OutputPath = p

        if interactive and fc.GuiUp:
            diag = FileOverwriteConfirmationDialog(
                    output_path, write_files)
            ignore, write, overwrite = diag.exec_()
            diag.close()
        if set(ignore) == set(write_files):
            return None
        elif set(write + overwrite) != set(write_files):
            warn(tr('Partial selection of files not supported yet'), True)
            return None
        package_parent, package_name = split_package_path(output_path)
        # TODO: warn if package name doesn't end with `_description`.
        robot_name = ros_name(self.workcell)
        file_base = get_valid_filename(robot_name)
        urdf_file = f'{file_base}.urdf.xacro'
        output_path.mkdir(parents=True, exist_ok=True)
        urdf_path = output_path / f'urdf/{urdf_file}'
        save_xml(robot_et, urdf_path)
        export_templates(template_files,
                         package_parent,
                         package_name=package_name,
                         urdf_file=urdf_file,
                         fixed_frame=workcell_root_link,
                         )
        return robot_et


class _ViewProviderWorkcell(ProxyBase):
    """A view provider for the Link container object """

    def __init__(self, vobj: VPDO):
        super().__init__('view_object', [
            'Visibility',
            ])
        vobj.Proxy = self

    def getIcon(self):
        # Implementation note: "return 'workcell.svg'" works only after
        # workbench activation in GUI.
        return str(ICON_PATH / 'workcell.svg')

    def attach(self, vobj: VPDO):
        self.view_object = vobj

    def updateData(self, obj: VPDO, prop):
        return

    def onChanged(self, vobj: VPDO, prop: str):
        return

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


def make_workcell(name, doc: Optional[fc.Document] = None) -> CrossWorkcell:
    """Add a Cross::Workcell to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    workcell: CrossWorkcell = doc.addObject('App::DocumentObjectGroupPython', name)
    WorkcellProxy(workcell)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        _ViewProviderWorkcell(workcell.ViewObject)

    doc.recompute()
    return workcell
