from __future__ import annotations

from typing import Optional

from PySide import QtCore  # FreeCAD's PySide!
from PySide import QtWidgets  # FreeCAD's PySide!

from ..robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
from ..freecad_utils import message
from ..freecad_utils import warn


class SetJointsFromTrajectory(QtWidgets.QTableWidget):

    def __init__(
            self,
            trajectory: Optional[dict],
            point_index: int = -1,
            start_state: Optional[dict] = None,
            robot: Optional[CrossRobot] = None,
            parent: Optional[QtWidgets.QWidget] = None,
    ):
        """Constructor from a trajectory dictionary (RobotTrajectory).

        Constructor from a trajectory dictionary, equivalent to a
        moveit_msgs/RobotTrajectory message.

        """
        super().__init__(parent)
        self._trajectory = trajectory
        self._point_index = point_index  # 0-based.
        self.robot = robot

        self._fill_table()

    @property
    def trajectory(self) -> Optional[dict]:
        return self._trajectory

    @trajectory.setter
    def trajectory(self, trajectory: dict) -> None:
        self._trajectory = trajectory
        self._fill_table()

    @property
    def point_index(self) -> int:
        """Return the 0-based index of the currently displayed point."""
        return self._point_index

    @point_index.setter
    def point_index(self, point_index: int) -> None:
        """Set the 0-based point index of the point to display.

        Set the 0-based point index of the point to display and fill the table.

        """
        self._point_index = point_index
        self._fill_table()

    def _fill_table(self) -> None:
        self.setRowCount(0)
        self.setColumnCount(2)
        if not self._trajectory:
            return
        try:
            joint_trajectory = self._trajectory['joint_trajectory']
            self.setRowCount(len(joint_trajectory['joint_names']))
        except KeyError:
            raise RuntimeError(
                    'The input trajectory does not have the expected'
                    ' format (moveit_msgs/RobotTrajectory)'
                    )

        try:
            if not 0 <= self.point_index < len(joint_trajectory['points']):
                warn(
                        (
                            'Point index out of range, should be'
                            f' <= {len(joint_trajectory["points"]) - 1},'
                            f' got {self.point_index}'
                        ),
                        gui=True
                )
                return
        except KeyError:
            raise RuntimeError(
                    'The input trajectory does not have the expected'
                    ' format (moveit_msgs/RobotTrajectory)'
                    )

        point = joint_trajectory['points'][self.point_index]
        joint_values: dict[str, float] = {}
        for i, (joint_name, joint_value) in enumerate(
                zip(joint_trajectory['joint_names'], point['positions'])):
            self._set_row(i, joint_name, joint_value)
            if self.robot:
                joint = self.robot.Proxy.get_joint(joint_name)
                if joint:
                    joint_values[joint] = joint_value
        if self.robot:
            self.robot.Proxy.set_joint_values(joint_values)
            self.robot.Document.recompute()

    def _set_row(self, row: int, joint_name: str, joint_value: float) -> None:
        """Populate with name and joint value.

        Populate the first column of the table with the joint names, the
        second column with the joint values.

        """
        # TODO: Add the unit column.
        self.setRowCount(row + 1)
        name_item = QtWidgets.QTableWidgetItem(joint_name)
        name_item.setFlags(name_item.flags() & ~QtCore.Qt.ItemIsEditable)
        self.setItem(row, 0, name_item)
        value_item = QtWidgets.QTableWidgetItem(f'{joint_value}')
        value_item.setFlags(value_item.flags() & ~QtCore.Qt.ItemIsEditable)
        self.setItem(row, 1, value_item)
