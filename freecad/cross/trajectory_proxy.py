# A trajectory object to store and replay trajectories as list of joint-space
# points.
#
# This file is part of the CROSS workbench for FreeCAD.

from __future__ import annotations

from pathlib import Path
from typing import Any
from typing import NewType
from typing import Optional
from typing import TYPE_CHECKING
from typing import Union

import FreeCAD as fc

from PySide.QtWidgets import QFileDialog  # FreeCAD's PySide
from PySide.QtWidgets import QMenu  # FreeCAD's PySide

try:
    from moveit_msgs.msg import RobotState
    from trajectory_msgs.msg import JointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
except ImportError:
    RobotState = Any
    JointTrajectory = Any
    JointTrajectoryPoint = Any

from .fpo import PropertyMode
from .fpo import PropertyEditorMode
from .fpo import PropertyIntegerConstraint
from .fpo import PropertyLink
from .fpo import PropertyString
from .fpo import proxy  # Cf. https://github.com/mnesarco/fcapi
from .fpo import view_proxy
from .freecad_utils import add_property
from .freecad_utils import message
from .freecad_utils import warn
from .ui.replay_trajectory_dialog import ReplayTrajectoryDialog
from .ui.choose_trajectory_dialog import ChooseTrajectoryDialog
from .utils import i_th_item
from .wb_utils import ICON_PATH
from .wb_utils import is_robot
from .wb_utils import ros_name

# Typing hints.
from .trajectory import Trajectory as CrossTrajectory  # A Cross::Trajectory, i.e. a DocumentObject with Proxy "Trajectory". # noqa: E501

if TYPE_CHECKING and hasattr(fc, 'GuiUp') and fc.GuiUp:
    from FreeCADGui import ViewProviderDocumentObject as VPDO
else:
    VPDO = NewType('VPDO', object)


@view_proxy(
        icon=str(ICON_PATH / 'trajectory.svg'),
)
class TrajectoryViewProxy:

    def on_context_menu(
            self,
            menu: QMenu,
    ) -> None:
        menu.addAction('Load Trajectory from YAML file...', self.load_yaml)
        menu.addAction('Replay...', self.replay)

    def load_yaml(self):
        import FreeCADGui as fcgui
        import yaml

        dialog = QFileDialog(
            fcgui.getMainWindow(),
            'Select a multi-doc YAML file to import a trajectory from',
        )
        dialog.setNameFilter('YAML *.yaml *.yml;;All files (*.*)')
        if dialog.exec_():
            filename = str(dialog.selectedFiles()[0])
        else:
            return

        # Check and warn if the multi-doc YAML contains more than one message
        # (1 message == 1 document) with `ros2 topic echo`.
        # TODO: open a dialog to ask to choose a trajectory.
        display_trajs = yaml.safe_load_all(open(filename))

        dialog = ChooseTrajectoryDialog(
                display_trajs,
                self.Object.Robot,
                fcgui.getMainWindow(),
        )
        message_index, trajectory_index = dialog.exec()
        if message_index >= 0:
            self.Object.Proxy.load_yaml(filename, message_index)

    def replay(self):
        old_index = self.Object.Proxy.point_index
        diag = ReplayTrajectoryDialog(self.Object)
        index = diag.exec_()
        diag.close()
        if index == -1:
            self.Object.Proxy.point_index = old_index


_TRAJECTORY_TYPE = 'Cross::Trajectory'


@proxy(
    object_type='App::FeaturePython',
    subtype=_TRAJECTORY_TYPE,
    view_proxy=TrajectoryViewProxy,
)
class TrajectoryProxy:

    type = PropertyString(
            name='_Type',
            default=_TRAJECTORY_TYPE,
            section='Internal',
            description='The type of the object',
            mode=PropertyMode.ReadOnly + PropertyMode.Hidden,
    )

    robot = PropertyLink(
            name='Robot',
            section='Robot',
            description='The associated robot',
    )

    point_index = PropertyIntegerConstraint(
            name='PointIndex',
            section='Robot',
            description=(
                'The index of the current trajectory'
                ' point to assign to the robot'
            ),
            mode=PropertyEditorMode.Hidden,
    )

    def __init__(self):
        super().__init__()

        # The map of joint names to FreeCAD properties, {joint_name: property}.
        # It's more practical than a property `App::PropertyMap` because the
        # property cannot be updated with indexing or `merge()`.
        self._joint_map: dict[str, str] = {}

    def on_create(self, obj: CrossTrajectory):
        self._set_editor_mode()

    def on_execute(self, obj: CrossTrajectory):
        if is_robot(self.robot):
            self._update_robot_joint_values()

    def on_serialize(self, state: dict[str, Any]) -> None:
        state['_joint_map'] = self._joint_map

    def on_deserialize(self, state: dict[str, Any]) -> None:
        self._joint_map = state['_joint_map']

    @robot.observer
    def on_robot_changed(self, new, old) -> None:
        if self.robot and (not is_robot(self.robot)):
            warn('The selected object is not a robot', True)
            self.robot = None
        if not self.robot:
            self._joint_map.clear()
            return
        # We do not clear `self._joint_map` because we want to keep the
        # values for the joints that are still present in the new robot.
        new_joint_names = [
            ros_name(j)
            for j in self.robot.Proxy.get_joints()
            if not j.Proxy.is_fixed()
        ]
        # Remove joints not present in the new robot.
        for name in self._joint_map.keys():
            if name not in new_joint_names:
                self._joint_map.pop(name)
                self.Object.remove_property(name)
        # Add missing properties.
        for name in new_joint_names:
            if name not in self._joint_map:
                self._add_joint(name)
        self._set_editor_mode()

    @point_index.observer
    def on_point_index_changed(self, new, old):
        if not (0 <= new < self.point_count):
            warn(
                f'Invalid point index: {new}, must be within [0, {self.point_count})',
                True,
            )
            if 0 <= old < self.point_count:
                self.point_index = old
            else:
                self.point_index = 0

    @property
    def point_count(self) -> int:
        count = 0
        for name, prop in self._joint_map.items():
            count = max(count, len(getattr(self.Object, prop)))
        return count

    def update_trajectory(
            self,
            trajectory: JointTrajectory,
            robot_state: Optional[RobotState] = None,
    ) -> None:
        try:
            if robot_state is None:
                start_state = {}
            else:
                start_state = {
                    j: p for j, p in zip(
                            robot_state.joint_state.name,
                            robot_state.joint_state.position,
                    )
                }

            traj: dict[str, list[float]] = {}
            for n in trajectory.joint_names:
                traj[n] = []
            for p in trajectory.points:
                for n, pos in zip(trajectory.joint_names, p.positions):
                    traj[n].append(pos)
        except AttributeError as e:
            warn(f'Invalid trajectory message: {e}', True)
            return

        # TODO: deactivate callbacks on property change.
        # Reset joint values not used in the trajectory.
        for joint_name, prop_name in self._joint_map.items():
            if joint_name not in traj.keys():
                setattr(self.Object, prop_name, [])
        for n in traj.keys():
            prop_name = self._joint_map[n]
            # Verify that the joint belongs to the robot (i.e. exists as
            # property of the trajectory).
            if not hasattr(self.Object, prop_name):
                warn(f'Ignoring positions for unknown joint: {n}', True)
                continue
            setattr(self.Object, prop_name, traj[n])
        for n, pos in start_state.items():
            start_prop_name = self._start_state_property_name(prop_name)
            if not hasattr(self.Object, start_prop_name):
                warn(f'Ignoring start state for unknown joint: {n}', True)
                continue
            setattr(self.Object, start_prop_name, start_state[n])
        # TODO: reactivate callbacks on property change.
        # Reset the point index (value, min, max, step).
        self.point_index = (0, 0, self.point_count - 1, 1)
        self._set_editor_mode()

    def load_yaml(
        self,
        file: Union[Path, str],
        message_index: int = 0,
        trajectory_index: int = 0,
    ) -> None:
        """Load a trajectory from a multi-doc YAML file.

        Such files are generated with `ros2 topic echo`.

        """
        import yaml

        display_trajs = yaml.safe_load_all(open(file))
        try:
            display_traj = i_th_item(display_trajs, message_index)
        except StopIteration:
            warn(
                 (
                     f'Invalid document index {message_index}'
                     ' (i.e. message index),'
                     ' not enough trajectories in the multi-doc file'
                 ),
                 True,
            )
            return

        self.load_display_trajectory_dict(display_traj, trajectory_index)

    def load_display_trajectory_dict(
            self,
            display_trajectory: dict[str, Any],
            trajectory_index: int = 0,
    ) -> None:
        """Load a trajectory from a `DisplayTrajectory` message as dict.

        `display_trajectory` must contain:
            - `trajectory` with a list of `JointTrajectory` messages as dict,
            - `trajectory_start` with a `RobotState` message as dict.
        """
        if 'trajectory' not in display_trajectory:
            warn('Wrongly formatted DisplayTrajectory message', True)
            return

        if not display_trajectory['trajectory']:
            message('No trajectory in the DisplayTrajectory message', False)
            self.Object.Proxy.update_trajectory(JointTrajectory())

        if len(display_trajectory['trajectory']) > (trajectory_index + 1):
            message(
                    (
                        'The DisplayTrajectory does not contain a RobotTrajectory'
                        f' with index {trajectory_index} as it has only'
                        f' {len(display_trajectory["trajectory"])} trajectories'
                    ),
                    True,
            )

        traj = JointTrajectory()
        joint_traj = display_trajectory['trajectory'][trajectory_index]['joint_trajectory']
        traj.joint_names = joint_traj['joint_names']
        for point in joint_traj['points']:
            traj_point = JointTrajectoryPoint()
            traj_point.positions = point['positions']
            traj.points.append(traj_point)

        robot_state = RobotState()
        js = display_trajectory['trajectory_start']['joint_state']
        robot_state.joint_state.name = js['name']
        robot_state.joint_state.position = js['position']

        self.update_trajectory(traj, robot_state)

    def _set_editor_mode(self) -> None:
        if self.robot and (self.point_count > 0):
            self.Object.setEditorMode('PointIndex', PropertyEditorMode.Default)
        else:
            self.Object.setEditorMode('PointIndex', PropertyEditorMode.Hidden)

    def _start_state_property_name(self, prop_name: str) -> str:
        """Return `start_q0` for joint `q0`."""
        return f'start_{prop_name}'

    def _add_joint(self, name: str) -> None:
        """Add `q0` and `start_q0` properties for joint `q0`.

        Add `q0` and `start_q0` properties for joint `q0` to the trajectory.

        """
        _, prop_name = add_property(
                self.Object,
                'App::PropertyFloatList',
                name,
                'Trajectory',
                f'The joint position for `{name}`',
        )
        add_property(
                self.Object,
                'App::PropertyFloat',
                self._start_state_property_name(prop_name),
                'Start State',
                f'The default position of `{name}`',
        )
        self._joint_map[name] = prop_name

    def _update_robot_joint_values(self):
        if not self.robot:
            return
        joint_names = self._joint_map.keys()
        # Get the current trajectory point.
        positions = {
            self.robot.Proxy.get_joint(n): getattr(self.Object, self._joint_map[n])[self.point_index]
            for n in joint_names
            if len(getattr(self.Object, self._joint_map[n])) > self.point_index
        }
        if not positions:
            return
        # Get defaults from the robot state.
        start_state = {
            self.robot.Proxy.get_joint(n): getattr(
                   self.Object,
                   self._start_state_property_name(self._joint_map[n]),
            )
            for n in joint_names
        }
        # Add missing joint values.
        state = start_state
        state.update(positions)
        self.robot.Proxy.set_joint_values(state)


def make_trajectory(
        name: str,
        doc: Optional[fc.Document] = None,
) -> CrossTrajectory:
    """Add a Cross::Trajectory to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj: CrossTrajectory = TrajectoryProxy.create(name=name, doc=doc)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        import FreeCADGui as fcgui

        # Set `obj.Robot` and possibly `obj.EndEffector` if the selected
        # object is a robot or a link.
        sel = fcgui.Selection.getSelection()
        if sel:
            candidate = sel[0]
            if is_robot(candidate):
                obj.Robot = candidate
    return obj
