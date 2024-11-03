from __future__ import annotations

from typing import Iterable, Optional

import FreeCAD as fc
import FreeCADGui as fcgui

from PySide import QtWidgets  # FreeCAD's PySide!

from ..wb_utils import UI_PATH
from .set_joints_from_trajectory import SetJointsFromTrajectory

# Stubs and type hints.
from ..robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
DO = fc.DocumentObject  # A FreeCAD DocumentObject.


class ChooseTrajectoryDialog(QtWidgets.QDialog):
    """A dialog to choose a trajectory.

    """

    def __init__(
            self,
            yamls: Iterable[dict],
            robot: Optional[CrossRobot] = None,
            parent: Optional[QtWidgets.QWidget] = None,
    ):
        """Constructor from the result of yaml.load_all() from DisplayTrajectory.

        Constructor from the result of yaml.load_all() with a multi-doc YAML
        file of moveit_msgs.msg.DisplayTrajectory messages, typically from
        `ros2 topic echo /display_planned_path`.

        """

        super().__init__(parent)

        # Make a list from the generator.
        # The last document in the multi-doc yaml is typically empty when using
        # `ros2 topic echo` so that `list(yamls)` adds an undisered empty dict.
        self._yamls = [y for y in yamls if y]

        self._message_index = 0 if self._yamls else -1
        self._trajectory_index = 0 if self._yamls else -1
        self._point_index = 0 if self._yamls else -1

        self.robot = robot

        self.form = fcgui.PySideUic.loadUi(
                str(UI_PATH / 'choose_trajectory_dialog.ui'), self)
        self._set_up_gui()
        self.dialog_confirmed = False

    def _set_up_gui(self) -> None:
        """Set up the GUI."""
        display_trajectory = self._yamls[self._message_index] if self._yamls else None
        t_idx = self._trajectory_index
        robot_trajectory = (display_trajectory['trajectory'][t_idx]
                            if (display_trajectory and (t_idx >= 0))
                            else None
                            )
        start_state = display_trajectory['trajectory_start'] if display_trajectory else None
        self.table_joint_values = SetJointsFromTrajectory(
                trajectory=robot_trajectory,
                point_index=self._point_index,
                start_state=start_state,
                robot=self.robot,
                parent=self.form.table_container_widget,
        )
        self.form.table_container_widget.layout().addWidget(
                self.table_joint_values)
        self._establish_connections()
        self._update_gui()

    def _establish_connections(self) -> None:
        self.form.message_number_spin_box.valueChanged.connect(self._on_message_number_spin_box_changed)
        self.form.traj_number_spin_box.valueChanged.connect(self._on_traj_number_spin_box_changed)
        self.form.point_number_spin_box.valueChanged.connect(self._on_point_number_spin_box_changed)
        self.form.point_number_slider.valueChanged.connect(self._on_point_number_slider_changed)
        self.form.button_box.accepted.connect(self._on_accept)
        self.form.button_box.rejected.connect(self._on_cancel)

    def _update_gui(self) -> None:
        if not self._yamls:
            self.form.message_number_spin_box.setEnabled(False)
            self.form.traj_number_spin_box.setEnabled(False)
            self.form.point_number_spin_box.setEnabled(False)
            return

        self.form.message_number_spin_box.setMaximum(len(self._yamls))  # 1-based.

        trajectories = self._yamls[self._message_index]['trajectory']
        try:
            trajectory = trajectories[self._trajectory_index]
        except TypeError:
            raise RuntimeError(
                    f'Message index {self._message_index}'
                    ' does not have the correct format'
            )
        except IndexError:
            raise RuntimeError(
                    f'Message index {self._message_index}'
                    ' contains {len(trajectories)} trajectories,'
                    ' trajectory index {self._trajectory_index}'
                    ' is out of range'
            )
        if len(trajectories) == 1:
            self.form.traj_number_spin_box.setValue(1)  # 1-based.
            self.form.traj_number_spin_box.setEnabled(False)
            self.form.traj_number_widget.setVisible(False)
        else:
            self.form.traj_number_spin_box.setValue(self._trajectory_index + 1)  # 1-based.
            self.form.traj_number_spin_box.setEnabled(True)
            self.form.traj_number_widget.setVisible(True)

        self.form.traj_number_spin_box.setMaximum(len(trajectories))  # 1-based.

        try:
            joint_trajectory = trajectory['joint_trajectory']
        except KeyError:
            raise RuntimeError(
                f'Trajectory index {self._trajectory_index}'
                ' does not have a joint_trajectory'
            )

        try:
            point_count = len(joint_trajectory['points'])
        except KeyError:
            raise RuntimeError(
                    f'Trajectory index {self._trajectory_index}'
                    ' does not have points'
            )

        self.form.point_number_spin_box.setMaximum(point_count)  # 1-based.
        self.form.point_number_slider.setMaximum(point_count)  # 1-based.

    def _on_message_number_spin_box_changed(self, value: int) -> None:
        self._message_index = value - 1  # `self._message_index` is 0-based.
        display_trajectory = self._yamls[self._message_index]
        self._trajectory_index = min(
                self._trajectory_index,
                len(display_trajectory['trajectory']) - 1
        )
        if self._trajectory_index >= 0:
            robot_trajectory = display_trajectory['trajectory'][self._trajectory_index]
            self._point_index = 0 if robot_trajectory['joint_trajectory']['points'] else - 1
            self.form.traj_number_spin_box.setValue(self._trajectory_index + 1)
            self.form.point_number_spin_box.setValue(self._point_index + 1)
            self.table_joint_values.trajectory = robot_trajectory
        else:
            self._point_index = -1
        self._update_gui()

    def _on_traj_number_spin_box_changed(self, value: int) -> None:
        robot_trajectories = self._yamls[self._message_index]['trajectory']
        robot_trajectory = robot_trajectories[value - 1]  # `value` is 1-based.
        self._point_index = min(self._point_index, len(robot_trajectory) - 1)
        # TODO: disable update of table_joint_values
        self.table_joint_values.point_index = self._point_index
        # TODO: re-enable update of table_joint_values.
        self.table_joint_values.trajectory = robot_trajectory
        self._update_gui()

    def _on_point_number_spin_box_changed(self, value: int) -> None:
        self._point_index = value - 1  # `point_index` is 0-based.
        self.table_joint_values.point_index = self._point_index
        self.form.point_number_slider.blockSignals(True)
        self.form.point_number_slider.setValue(value)
        self.form.point_number_slider.blockSignals(False)

    def _on_point_number_slider_changed(self, value: int) -> None:
        self.form.point_number_spin_box.setValue(value)  # 1-based.

    def _on_accept(self) -> None:
        self.dialog_confirmed = True
        self.form.close()

    def _on_cancel(self) -> None:
        self.dialog_confirmed = False
        self.form.close()

    def exec(self) -> int:
        """Return the 0-based message and trajectory indices.

        Return (message_index, trajectory_index) if the dialog was accepted, as
        0-based indices.
        Return (-1, -1) if the dialog was cancelled

        """
        self.form.exec_()
        if not self.dialog_confirmed:
            return -1, -1
        return (
                int(self.form.message_number_spin_box.value()) - 1,
                int(self.form.traj_number_spin_box.value()) - 1,
                )

    def close(self) -> None:
        self.form.close()
