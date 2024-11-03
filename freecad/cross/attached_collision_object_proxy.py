# An object temporarily attached to a robot via a fixed joint.
#
# This file is part of the CROSS workbench for FreeCAD.

from __future__ import annotations

from typing import Any
from typing import NewType
from typing import Optional
from typing import cast

import FreeCAD as fc

from .fpo import PropertyLink
from .fpo import PropertyLinkList
from .fpo import PropertyMode
from .fpo import PropertyPlacement
from .fpo import PropertyString
from .fpo import proxy  # Cf. https://github.com/mnesarco/fcapi
from .fpo import view_proxy
from .freecad_utils import is_link as is_freecad_link
from .freecad_utils import warn
from .utils import warn_unsupported
from .wb_utils import ICON_PATH
from .wb_utils import is_joint
from .wb_utils import is_link
from .wb_utils import is_robot

# Typing hints.
from .attached_collision_object import AttachedCollisionObject as CrossAttachedCollisionObject  # A Cross::AttachedCollisionObject, i.e. a DocumentObject with Proxy "AttachedCollisionObject". # noqa: E501
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
DO = fc.DocumentObject
DOList = list[DO]
AppLink = DO  # TypeId == 'App::Link'.
VPDO = NewType('FreeCADGui.ViewProviderDocumentObject', DO)  # Don't want to import FreeCADGui here. # noqa: E501


def _add_fc_links(
        attached_collision_object: CrossAttachedCollisionObject,
        objects: DOList,
) -> list[AppLink]:
    """Create FreeCAD links to elements in `attached_collision_object.Objects`.

    Return the list of created FreeCAD link objects.
    The objects are not added to the attached_collision_object (it's a group),
    just to the document.

    Parameters
    ----------
    - attached_collision_object: a FreeCAD object of type
                                 Cross::CrossAttachedCollisionObject.
    - objects: the list of objects to potentially add.

    """
    doc = attached_collision_object.Document
    fc_links: DOList = []
    for o in objects:
        name = f'_{attached_collision_object.Label}'
        app_link = doc.addObject('App::Link', name)
        app_link.Label = name
        app_link.LinkPlacement = attached_collision_object.Placement
        app_link.setLink(o)
        app_link.adjustRelativeLinks(attached_collision_object)
        fc_links.append(app_link)
    return fc_links


def _skim_links_joints_from(group) -> tuple[DOList, DOList]:
    """Remove all Cross::Link and Cross::Joint from the list.

    `group` is a property that looks like a list but behaves differently
    (behaves like a tuple and is a copy of the original property content,
    so cannot be changed here).

    Return (kept_objects, removed_objects).

    """
    removed_objects: DOList = []
    kept_objects: DOList = list(group)
    # Implementation note: reverse order required.
    for i, o in reversed(list(enumerate(kept_objects))):
        if is_link(o) or is_joint(o):
            warn_unsupported(o, by='CROSS::AttachedCollisionObject', gui=True)
            # Implementation note: cannot use `kept_objects.remove`, this would
            # lose the object.
            removed_objects.append(kept_objects.pop(i))
    return kept_objects, removed_objects


_OBJECT_TYPE = 'Cross::AttachedCollisionObject'


@view_proxy(
    icon=str(ICON_PATH / 'attached_collision_object.svg'),
    extensions=['Gui::ViewProviderGroupExtensionPython'],
)
class AttachedCollisionObjectViewProxy:

    def on_changed(self, vobj: VPDO, prop: str):
        if prop == 'Visibility':
            for o in vobj.Object.Group:
                o.ViewObject.Visibility = vobj.Visibility


@proxy(
    object_type='App::FeaturePython',
    extensions=['App::GroupExtensionPython'],
    subtype=_OBJECT_TYPE,
    view_proxy=AttachedCollisionObjectViewProxy,
)
class AttachedCollisionObjectProxy:

    type = PropertyString(
            name='_Type',
            default=_OBJECT_TYPE,
            section='Internal',
            description='The type of the object',
            mode=PropertyMode.ReadOnly + PropertyMode.Hidden,
    )

    link = PropertyLink(
            name='Link',
            section='Elements',
            description='The link to attach to',
    )

    objects = PropertyLinkList(
            name='Objects',
            section='Elements',
            description='The objects to attach',
    )

    placement = PropertyPlacement(
            name='Placement',
            section='Internal',
            description='Placement of elements in the robot frame',
    )

    def __init__(self):
        # List to keep track of the objects that were added to the
        # AttachedCollisionObject.
        self._fc_links: DOList = []

        # Save the robot to speed-up `self.robot`.
        self._robot: Optional[CrossRobot] = None

    def on_create(self, obj: CrossAttachedCollisionObject):
        self._set_editor_modes()

    def on_start(self, obj: CrossAttachedCollisionObject):
        self._fix_lost_fc_links()
        self._fill_fc_link_lists()

    def on_serialize(self, state: dict[str, Any]) -> tuple[str]:
        return self.Type,

    def on_deserialize(self, state: tuple[str]) -> None:
        # Implementation note: `self.__init__()` is not called
        # on document restore, call it manually.
        self.__init__()
        if state:
            self.Type, = state

    def on_changed(self, obj: CrossAttachedCollisionObject, prop: str) -> None:
        """Manage property changes for properties not defined by us."""
        if prop == 'Group':
            self._cleanup_children()
        self._set_editor_modes()

    @objects.observer
    def on_objects_changed(self, obj: CrossAttachedCollisionObject) -> None:
        self._cleanup_children()
        self._update_fc_links()

    @placement.observer
    def on_placement_changed(
            self,
            obj: CrossAttachedCollisionObject,
            new_placement: fc.Placement,
            old_placement: fc.Placement,
    ) -> None:
        if self.placement != new_placement:
            self.placement = new_placement
        for fclink in self.Object.Group:
            if not is_freecad_link(fclink):
                # Should not happen.
                continue
            link_placement = fclink.LinkedObject.Placement * new_placement
            if fclink.LinkPlacement != link_placement:
                # Avoid recursion.
                fclink.LinkPlacement = link_placement

    @property
    def robot(self) -> Optional[CrossRobot]:
        """Return the Cross::Robot this AttachedCollisionObject belongs to."""
        if (
            hasattr(self, '_robot')
            and self._robot
            and hasattr(self._robot, 'Group')
            and (self.Object in self._robot.Group)
        ):
            return self._robot
        for o in self.Object.InList:
            if is_robot(o):
                self._robot = cast(CrossRobot, o)
                return self._robot
        return None

    def _set_editor_modes(self) -> None:
        """Set the modes of the properties."""
        self.Object.setEditorMode('Group', ['ReadOnly'])
        if self.robot:
            # Placement is managed by the robot.
            self.Object.setEditorMode('Placement', ['ReadOnly'])
        else:
            self.Object.setEditorMode('Placement', [])

    def _fix_lost_fc_links(self) -> None:
        """Fix linked objects in AttachedCollisionObject lost on restore.

        Probably because these elements are restored
        beforAttachedCollisionObject objects.

        """
        aco: CrossAttachedCollisionObject = self.Object
        for obj in aco.Document.Objects:
            if (not hasattr(obj, 'InList')) or (len(obj.InList) != 1):
                continue
            potential_self = obj.InList[0]
            if ((obj is aco)
                    or (potential_self is not aco)
                    or (obj in aco.Group)
                    or (obj in aco.Objects)):
                continue
            aco.addObject(obj)  # aco is a group.

    def _fill_fc_link_lists(self) -> None:
        """Fill the lists of FreeCAD links.

        The list `_fc_links` is empty on document restore
        and needs to be filled up.

        Must be called after `_fix_lost_fc_links`.
        Not very elegant but the lists cannot be serialized easily.

        """
        for o in self.Object.Group:
            if o.Label.startswith('_'):
                self._fc_links.append(o)

    def _cleanup_children(self) -> DOList:
        """Remove and return all objects not supported by AttachedCollisionObject."""
        removed_objects: set[DO] = set()
        # Group is managed by us.
        for o in self.Object.Group:
            if is_freecad_link(o):
                # Supported, and managed by us.
                continue
            warn_unsupported(o, by='CROSS::AttachedCollisionObject', gui=True)
            # implementation note: removeobject doesn't raise any exception
            # and `o` exists even if already removed from the group.
            removed_objects.update(self.Object.removeObject(o))

        # Clean-up `Objects`.
        kept, removed = _skim_links_joints_from(self.Object.Objects)
        if self.Object.Objects != kept:
            self.Object.Objects = kept
        warn_unsupported(removed, by='CROSS::AttachedCollisionObject', gui=True)
        removed_objects.update(removed)

        return list(removed_objects)

    def _update_fc_links(self) -> None:
        """Update the FreeCAD link according to the level of details."""
        aco: CrossAttachedCollisionObject = self.Object
        if not hasattr(aco, 'ViewObject'):
            # No need to change `Group` without GUI.
            return
        vaco = aco.ViewObject
        if vaco is None:
            return

        # Old objects that will be removed after having been excluded from
        # `Group`, to avoid recursive calls.
        old_fc_links: DOList = self._fc_links

        # Free the labels because of delayed removal.
        for o in old_fc_links:
            # Free the label.
            o.Label = 'to_be_removed'

        # Clear the lists that are regenerated right after and create new
        # objects.
        self._fc_links.clear()

        # Create new objects.
        self._fc_links = _add_fc_links(aco, aco.Objects)

        # Reset the group.
        if self._fc_links != aco.Group:
            aco.Group = self._fc_links

        # Remove old objects.
        doc = aco.Document
        for o in old_fc_links:
            doc.removeObject(o.Name)


def make_attached_collision_object(
        name: str,
        doc: Optional[fc.Document] = None,
) -> CrossAttachedCollisionObject:
    """Add a Cross::AttachedCollisionObject to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document and cannot create a new document, doing nothing', True)
        return
    obj: CrossAttachedCollisionObject = AttachedCollisionObjectProxy.create(
            name=name,
            doc=doc,
    )
    doc.recompute()
    return obj
