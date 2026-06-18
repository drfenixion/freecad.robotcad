"""Proxy for Cross::VacuumGripper FreeCAD objects.

A vacuum gripper represents a suction gripper plugin for Gazebo.
It is attached to a link and exports as a plugin in the world SDF.
"""

from __future__ import annotations

from typing import ForwardRef, List, Optional
from pathlib import Path

import FreeCAD as fc

from PySide.QtWidgets import QMenu  # FreeCAD's PySide

from .freecad_utils import ProxyBase
from .freecad_utils import add_property
from .freecad_utils import error
from .wb_utils import ICON_PATH
from .wb_utils import ros_name
from .wb_utils import get_valid_urdf_name

# Stubs and type hints.
DO = fc.DocumentObject
DOList = List[DO]
VPDO = ForwardRef('FreeCADGui.ViewProviderDocumentObject')


class VacuumGripperProxy(ProxyBase):
    """The proxy for VacuumGripper objects."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Cross::VacuumGripper'

    def __init__(self, obj: DO):
        super().__init__(
            'vacuum_gripper', [
            '_Type',
            ],
        )

        if obj.Proxy is not self:
            obj.Proxy = self
        self.vacuum_gripper = obj

        self._init_properties(obj)

    def _init_properties(self, obj: DO):
        add_property(
            obj, 'App::PropertyString', '_Type', 'Internal',
            'The type',
        )
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        # --- Configuration Parameters ---

        add_property(
            obj, 'App::PropertyString', 'SuctionTopic', 'Vacuum Gripper',
            'Topic to enable/disable suction',
            '/suction/enable',
        )

        add_property(
            obj, 'App::PropertyString', 'LinkName', 'Vacuum Gripper',
            'Name of the link to attach to (auto-filled from parent link)',
            '',
        )
        obj.setPropertyStatus('LinkName', ['ReadOnly'])

        add_property(
            obj, 'App::PropertyString', 'FilterName', 'Vacuum Gripper',
            'Substring to match target model names',
            'box',
        )

        add_property(
            obj, 'App::PropertyBool', 'UseSuctionRadius', 'Vacuum Gripper',
            'Set to True for Vacuum Mode, False for Contact Mode',
            True,
        )

        add_property(
            obj, 'App::PropertyFloat', 'SuctionRadius', 'Vacuum Gripper',
            'Range (meters) to apply suction force',
            0.1,
        )

        add_property(
            obj, 'App::PropertyFloat', 'SuctionForce', 'Vacuum Gripper',
            'Force (Newtons) to pull objects',
            20.0,
        )

    def execute(self, obj: DO) -> None:
        pass

    def onChanged(self, obj: DO, prop: str) -> None:
        """Update LinkName when parent link changes."""
        if prop == 'SuctionTopic':
            # Keep the property in sync if needed.
            pass

    def onDocumentRestored(self, obj):
        """Restore attributes because __init__ is not called on restore."""
        self.__init__(obj)

    def dumps(self):
        return None

    def loads(self, state) -> None:
        pass

    def update_link_name(self, parent_link_name: str) -> None:
        """Update the LinkName property from the parent link."""
        obj = self.vacuum_gripper
        if hasattr(obj, 'LinkName'):
            obj.LinkName = get_valid_urdf_name(ros_name(parent_link_name))


class _ViewProviderVacuumGripper(ProxyBase):
    """A view provider for the VacuumGripper container object."""

    def __init__(self, vobj: VPDO):
        super().__init__(
            'view_object',
            [
                'Visibility',
            ],
        )
        vobj.Proxy = self

    def getIcon(self):
        return str(ICON_PATH / 'vacuum_gripper.svg')

    def attach(self, vobj: VPDO):
        self.view_object = vobj
        self.vacuum_gripper = vobj.Object

    def updateData(self, obj: DO, prop: str):
        return

    def onChanged(self, vobj: VPDO, prop: str):
        pass

    def setupContextMenu(self, vobj: VPDO, menu: QMenu) -> None:
        return

    def doubleClicked(self, vobj: VPDO):
        gui_doc = vobj.Document
        if not gui_doc.getInEdit():
            gui_doc.setEdit(vobj.Object.Name)
        else:
            error('Task dialog already active')
        return True

    def setEdit(self, vobj: VPDO, mode):
        return False

    def unsetEdit(self, vobj: VPDO, mode):
        import FreeCADGui as fcgui
        fcgui.Control.closeDialog()

    def dumps(self):
        return None

    def loads(self, state) -> None:
        pass

    def on_context_menu(self, vobj: VPDO) -> None:
        pass


def make_vacuum_gripper(name: str = 'VacuumGripper') -> Optional[DO]:
    """Add a Cross::VacuumGripper to the current document."""
    doc = fc.activeDocument()
    if doc is None:
        error('No active document', True)
        return None

    vacuum_gripper: DO = doc.addObject('App::FeaturePython', name)
    VacuumGripperProxy(vacuum_gripper)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        _ViewProviderVacuumGripper(vacuum_gripper.ViewObject)

    return vacuum_gripper


def add_vacuum_gripper_to_link(
    link: DO,
    name: str = 'VacuumGripper',
) -> Optional[DO]:
    """Create a vacuum gripper and add it to the given link.

    This is the main helper to be used by the command.

    Parameters
    ----------
    - link: The Cross::Link to attach the gripper to.
    - name: The name for the new gripper object.

    Returns
    -------
    - The newly created vacuum gripper object, or None on failure.
    """
    doc = fc.activeDocument()
    if doc is None:
        error('No active document', True)
        return None
    if link is None:
        error('No link provided', True)
        return None

    doc.openTransaction('Add Vacuum Gripper')
    vacuum_gripper = make_vacuum_gripper(name)
    if vacuum_gripper is None:
        doc.abortTransaction()
        return None

    link.addObject(vacuum_gripper)
    vacuum_gripper.Proxy.update_link_name(link)
    doc.commitTransaction()
    doc.recompute()
    return vacuum_gripper


def get_vacuum_gripper_plugin_xml(vacuum_gripper: DO, link_urdf_name: str) -> str:
    """Get the SDF plugin XML string for a vacuum gripper.

    Parameters
    ----------
    - vacuum_gripper: The Cross::VacuumGripper object.
    - link_urdf_name: The valid URDF name of the parent link.

    Returns
    -------
    - SDF plugin XML string.
    """
    import xml.etree.ElementTree as et
    from xml.dom import minidom

    plugin = et.SubElement(
        et.Element('dummy'),
        'plugin',
        attrib={
            'filename': 'libsuction_plugin.so',
            'name': 'gz::sim::v8::systems::SuctionPlugin',
        },
    )

    # Suction topic
    suction_topic = et.SubElement(plugin, 'suction_topic')
    suction_topic.text = vacuum_gripper.SuctionTopic

    # Link name
    link_name_elem = et.SubElement(plugin, 'link_name')
    link_name_elem.text = link_urdf_name

    # Filter name
    filter_name = et.SubElement(plugin, 'filter_name')
    filter_name.text = vacuum_gripper.FilterName

    # Use suction radius
    use_suction_radius = et.SubElement(plugin, 'use_suction_radius')
    use_suction_radius.text = str(vacuum_gripper.UseSuctionRadius).lower()

    # Suction radius
    suction_radius = et.SubElement(plugin, 'suction_radius')
    suction_radius.text = str(vacuum_gripper.SuctionRadius)

    # Suction force
    suction_force = et.SubElement(plugin, 'suction_force')
    suction_force.text = str(vacuum_gripper.SuctionForce)

    raw_xml = et.tostring(plugin).decode('utf-8')
    # Format with indentation.
    pretty_xml = minidom.parseString(
        f'<dummy>{raw_xml}</dummy>'
    ).toprettyxml(indent='  ', encoding='utf-8').decode('utf-8')
    pretty_xml = pretty_xml.replace('<dummy>', '').replace('</dummy>', '')
    # Remove xml declaration.
    lines = pretty_xml.split('\n')
    if lines[0].startswith('<?xml'):
        lines = lines[1:]
    pretty_xml = '\n'.join(lines).strip()

    return pretty_xml
