from __future__ import annotations

from typing import Optional

import FreeCAD as fc

from .freecad_utils import warn
from .wb_utils import ICON_PATH
from .wb_utils import is_link
from .wb_utils import is_robot
from .wb_utils import ros_name
from freecad.cross.vendor.fcapi import fpo  # Cf. https://github.com/mnesarco/fcapi

# Stubs and type hints.
from .link import Link as CrossLink  # A Cross::Link, i.e. a DocumentObject with Proxy "Link". # noqa: E501
from .pose import Pose as CrossPose  # A Cross::Pose, i.e. a DocumentObject with Proxy "Pose". # noqa: E501
from .pose import ViewProviderPose as VP


def _get_potential_end_effectors(pose: CrossPose) -> list[str]:
    """Return the list of potential end-effectors."""
    if ((not hasattr(pose, 'Proxy'))
            or (pose.Proxy is None)
            or (not hasattr(pose, 'Robot'))):
        return []
    return pose.Proxy.get_potential_end_effectors()


@fpo.view_proxy(
        icon=str(ICON_PATH / 'pose.svg'),
)
class _ViewProviderPose:
    """The view provider for CROSS::Pose objects."""

    show_end_effector = fpo.PropertyBool(
            name='ShowEndEffector',
            section='ROS Display Options',
            description=(
                'Whether to show the end-effector link or a symbolic'
                'representation of the pose'
                ),
            default=True,
     )

    axis_length = fpo.PropertyLength(
            name='AxisLength',
            section='ROS Display Options',
            description="Length of the rods for the joint's axes",
            default=500.0,  # mm.
    )

    shaded_display_mode = fpo.DisplayMode(name='Shaded', is_default=True)
    wireframe_display_mode = fpo.DisplayMode(name='Wireframe')

    def on_object_change(self) -> None:
        """Callback when the data object changes."""
        self.draw()

    @show_end_effector.observer
    def _show_end_effector_onchange(self) -> None:
        """Callback when the `ShowEndEffector` property changed."""
        self.draw()

    @axis_length.observer
    def _axis_length_onchange(self) -> None:
        """Callback when the `AxisLength` property changed."""
        self.draw()

    def on_change(self, event: fpo.events.PropertyChangedEvent) -> None:
        """Callback when a property changed.

        We need this for properties defined by mother classes.
        """
        if event.property_name == 'Visibility':
            self.draw()

    def draw(self) -> None:
        from pivy import coin
        from .coin_utils import tcp_group
        from .coin_utils import transform_from_placement

        vobj: VP = self.ViewObject
        obj: CrossPose = vobj.Object

        if not hasattr(vobj, 'RootNode'):
            return
        root = vobj.RootNode
        root.removeAllChildren()

        shaded = coin.SoSeparator()
        root.addChild(shaded)
        style = coin.SoDrawStyle()
        style.style = coin.SoDrawStyle.LINES
        wireframe = coin.SoSeparator()
        wireframe.addChild(style)
        root.addChild(wireframe)

        if not vobj.Visibility:
            return
        sep = coin.SoSeparator()
        sep.addChild(transform_from_placement(obj.Placement))
        sep.addChild(
            tcp_group(
            tcp_length_mm=0.66 * vobj.AxisLength,
            tcp_diameter_ratio_to_length=0.17,
            tcp_color=(0.7, 0.7, 0.7),
            axis_length_mm=vobj.AxisLength,
            axis_diameter_ratio_to_length=0.03,
            ),
        )

        if obj.Robot and self.show_end_effector:
            robot_proxy = obj.Robot.Proxy
            link = robot_proxy.get_link(obj.EndEffector)
            if link is not None:
                # TODO: only works if robot at null joint-space pose.
                # TODO: get the correct transform.
                sep.addChild(_get_link_separator(link))

        shaded.addChild(sep)
        wireframe.addChild(sep)


@fpo.proxy(
        object_type='App::GeometryPython',
        subtype='Cross::Pose',
        view_proxy=_ViewProviderPose,
)
class PoseProxy:
    """The proxy for CROSS::Pose objects."""

    robot = fpo.PropertyLink(
            name='Robot',
            section='Robot',
            description='The associated robot',
    )

    end_effector, end_effector_meta = fpo.PropertyOptions(
            name='EndEffector',
            section='Robot',
            description='End-effector link (from CROSS) to bring to the pose',
            meta=True,
            # Implementation note: no `self` available here.
            options_provider=_get_potential_end_effectors,
    )

    allow_non_leaf_link = fpo.PropertyBool(
         name='AllowNonLeafLink',
         section='Robot',
         description='Whether to list non-leaf links in `EndEffector`',
         default=False,
     )

    def on_attach(self) -> None:
        """Callback when the proxy is attached to the object."""
        self._end_effectors: list[str] = []

    @robot.observer
    def _robot_onchange(self) -> None:
        """Callback when the `Robot` property changed."""
        self._update_end_effector_list()

    @allow_non_leaf_link.observer
    def _allow_non_leaf_onchange(self) -> None:
        """Callback when the `AllowNonLeafLink` property changed."""
        self._update_end_effector_list()

    def _update_end_effector_list(self) -> None:
        """Update the list of potential end-effectors."""
        pose_obj = self.Object
        if not pose_obj.Robot:
            pose_obj.EndEffector = []
            return
        robot = pose_obj.Robot
        if ((not hasattr(robot, 'Proxy'))
                or (robot.Proxy is None)
                or (not robot.Proxy.is_execute_ready())):
            return
        chains = robot.Proxy.get_chains()
        links: set[CrossLink] = set()
        for chain in chains:
            if pose_obj.AllowNonLeafLink:
                links.update([ln for ln in chain if is_link(ln)])
            else:
                links.add(chain[-1])
        # Update the list of choices of the enumeration.
        # If the current value is in the new list, it'll be kept.
        # Otherwise, the new value will be set to the first entry.
        self.end_effector_meta.options = sorted([ros_name(ln) for ln in links])

    def get_potential_end_effectors(self) -> list[str]:
        self. _update_end_effector_list()
        return self._end_effectors


def make_pose(
        name: str,
        doc: Optional[fc.Document] = None,
) -> CrossPose:
    """Add a Cross::Pose to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj: CrossPose = PoseProxy.create(name=name, doc=doc)
    obj.Label2 = name

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        import FreeCADGui as fcgui

        _ViewProviderPose(obj.ViewObject)

        # Set `obj.Robot` and possibly `obj.EndEffector` if the selected
        # object is a robot or a link.
        sel = fcgui.Selection.getSelection()
        if sel:
            candidate = sel[0]
            if is_robot(candidate):
                obj.Robot = candidate
            elif is_link(candidate) and (hasattr(candidate, 'Proxy')):
                robot = candidate.Proxy.get_robot()
                if robot:
                    obj.Robot = robot
                    link_name = ros_name(candidate)
                    if link_name not in obj.getEnumerationsOfProperty('EndEffector'):
                        obj.AllowNonLeafLink = True
                    obj.EndEffector = ros_name(candidate)
                    obj.Placement = candidate.Placement
    return obj


def _get_group_separator(link: CrossLink) -> 'coin.SoSeparator':
    """Return the SoSeparator with all element in `link.Group`.

    Elements in `link.Group` are FreeCAD links. Their inventor representation
    contains an SoTransform node with their location in the 3D scene and an
    SoSwitch node for the different display nodes.
    For each element, we ignore the SoTransform and add only the SoSwitch to
    the separator.

    """
    from pivy import coin
    from .coin_utils import transform_from_placement

    sep = coin.SoSeparator()
    sep.addChild(transform_from_placement(link.MountedPlacement))
    for fc_link in link.Group:
        if fc_link.ViewObject is None:
            continue
        for node in fc_link.ViewObject.RootNode.getChildren():
            if isinstance(node, coin.SoSwitch):
                sep.addChild(node)
                break
    return sep


def _get_link_separator(link: CrossLink) -> 'coin.SoSeparator':
    """Return the SoSeparator of the link.

    Apply the transform `link.MountedPlacement` and add an SoSeparator with all
    elements in `link.Group` and all CROSS::Link objects that are fixed to
    `link`.

    """
    from pivy import coin
    from .coin_utils import transform_from_placement

    sep = coin.SoSeparator()

    if not link.Proxy.is_execute_ready():
        return sep
    robot = link.Proxy.get_robot()
    if robot is None:
        # A link outside of a robot (though it should not happen).
        sep.addChild(_get_group_separator(link))
        return sep
    for fixed_with_link in robot.Proxy.get_links_fixed_with(ros_name(link)):
        ln_sep = coin.SoSeparator()
        fixed_transform = robot.Proxy.get_transform(
            ros_name(link),
            ros_name(fixed_with_link),
        )
        ln_sep.addChild(transform_from_placement(fixed_transform))
        ln_sep.addChild(_get_group_separator(fixed_with_link))
        sep.addChild(ln_sep)
    return sep
