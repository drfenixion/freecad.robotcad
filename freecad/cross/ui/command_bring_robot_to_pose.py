# Bring a Cross::Robot to a Cross::Pose.

from contextlib import suppress
from typing import TypeAlias

from PySide import QtGui  # FreeCAD's PySide!
import FreeCAD as fc
import FreeCADGui as fcgui

# Typing hints.
from freecad.cross.pose import Pose as CrossPose  # A Cross::Pose, i.e. a DocumentObject with Proxy "Pose". # noqa: E501
from freecad.cross.robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
DO: TypeAlias = fc.DocumentObject


class _BringRobotToPoseCommand:
    """The command definition to bring a Robot to a Pose."""

    def __init__(self) -> None:
        self._form: 'freecad.cross.ui.object_selector_dialog.ObjectSelector | None' = None
        self._doc: fc.Document | None = None
        self._robot: CrossRobot | None = None
        self._placement: fc.Placement | None = None
        self._endeffector: str = ''
        self._fixed_joint_widgets: list[tuple[QtGui.QComboBox, QtGui.QDoubleSpinBox]] = []
        self._dialog_confirmed = False

    def GetResources(self):
        return {
            'Pixmap': 'bulb',
            'MenuText': 'Bring Robot to Pose',
            'Accel': 'B, R',
            'ToolTip': 'Bring a CROSS::Robot to a CROSS::Pose. Select a CROSS::Robot and a CROSS::Pose.',
        }

    def IsActive(self):
        return fc.activeDocument() is not None

    def Activated(self):
        self._doc = fc.activeDocument()
        sel = fcgui.Selection.getSelection()
        self._get_input(sel)
        if self._robot and self._placement and self._endeffector:
            fixed_joints = {c.currentText(): v.value() for c, v in self._fixed_joint_widgets}
            bring_to_placement(self._robot, self._placement, self._endeffector, fixed_joints)

    def _get_input(
            self,
            selected: list[DO],
    ) -> None:
        """Get required info from user and set members."""
        # Import late to avoid slowing down workbench start-up.
        from ..freecad_utils import validate_types
        from ..wb_utils import UI_PATH
        robot: CrossRobot | None = None
        pose: CrossPose | None = None
        if len(selected) >= 2:
            robot_and_pose = validate_types(selected, ['Cross::Robot', 'Cross::Pose'])
            robot, pose = robot_and_pose
        elif len(selected) == 1:
            with suppress(RuntimeError):
                robot = validate_types(selected, ['Cross::Robot'])[0]
            with suppress(RuntimeError):
                pose = validate_types(selected, ['Cross::Pose'])[0]

        self._form = fcgui.PySideUic.loadUi(
                str(UI_PATH / 'bring_robot_to_pose_dialog.ui'),
                fcgui.getMainWindow(),
        )
        if not self._form:
            raise RuntimeError('Failed to load UI form for BringRobotToPose')

        f = self._form
        # Link the radio buttons.
        f.radio_button_group = QtGui.QButtonGroup(f)
        f.radio_button_group.addButton(f.existing_pose_radio_button)
        f.radio_button_group.addButton(f.manual_pose_radio_button)

        # Set initial state of widgets.
        self._toggle_radios()
        self._setup_fixed_joint_widget()

        # Connect signals.
        f.robot_line_edit.textChanged.connect(self._robot_changed)
        f.robot_push_button.clicked.connect(self._pick_robot)
        f.existing_pose_line_edit.textChanged.connect(self._check_input)
        f.existing_pose_push_button.clicked.connect(self._pick_pose)
        f.endeffector_line_edit.textChanged.connect(self._check_input)
        f.endeffector_push_button.clicked.connect(self._pick_link)
        f.existing_pose_radio_button.toggled.connect(self._toggle_radios)
        f.manual_pose_radio_button.toggled.connect(self._toggle_radios)
        f.button_box.accepted.connect(self._on_accept)
        f.button_box.rejected.connect(f.close)

        # Completers.
        # f.robot_line_edit.setCompleter(QtGui.QCompleter(...))
        if robot:
            f.robot_line_edit.setText(robot.Label)
        if pose:
            f.existing_pose_line_edit.setText(pose.Label)
            f.endeffector_line_edit.setText(pose.EndEffector)

        self._check_input()
        if f.exec():
            if not self._dialog_confirmed:
                self._robot = None
                self._placement = None
                self._endeffector = ''
                self._fixed_joint_widgets = []
            else:
                self._robot = self._get_robot_from_line_edit()

    def _toggle_radios(self) -> None:
        f = self._form
        use_existing_pose = f.existing_pose_radio_button.isChecked()
        f.existing_pose_line_edit.setEnabled(use_existing_pose)
        f.endeffector_line_edit.setEnabled(not use_existing_pose)
        f.endeffector_push_button.setEnabled(not use_existing_pose)
        f.x_spinbox.setEnabled(not use_existing_pose)
        f.y_spinbox.setEnabled(not use_existing_pose)
        f.z_spinbox.setEnabled(not use_existing_pose)
        f.qx_spinbox.setEnabled(not use_existing_pose)
        f.qy_spinbox.setEnabled(not use_existing_pose)
        f.qz_spinbox.setEnabled(not use_existing_pose)
        f.qw_spinbox.setEnabled(not use_existing_pose)
        self._check_input()

    def _get_robot_from_line_edit(self) -> CrossRobot | None:
        """Return the CROSS::Robot from the line edit if possible."""
        from ..wb_utils import is_robot

        robot_label = self._form.robot_line_edit.text()
        robots = self._doc.getObjectsByLabel(robot_label)
        for robot in robots:
            if is_robot(robot):
                return robot
        return None

    def _get_pose_from_line_edit(self) -> CrossPose | None:
        """Return the CROSS::Pose from the line edit if possible."""
        from ..wb_utils import is_pose

        pose_label = self._form.existing_pose_line_edit.text()
        poses = self._doc.getObjectsByLabel(pose_label)
        for pose in poses:
            if is_pose(pose):
                return pose
        return None

    def _fill_placement_from_existing_pose(self) -> None:
        f = self._form
        pose = self._get_pose_from_line_edit()
        if not pose:
            return
        f.x_spinbox.setValue(pose.Placement.Base.x)
        f.y_spinbox.setValue(pose.Placement.Base.y)
        f.z_spinbox.setValue(pose.Placement.Base.z)
        f.qx_spinbox.setValue(pose.Placement.Rotation.Q[0])
        f.qy_spinbox.setValue(pose.Placement.Rotation.Q[1])
        f.qz_spinbox.setValue(pose.Placement.Rotation.Q[2])
        f.qw_spinbox.setValue(pose.Placement.Rotation.Q[3])

    def _robot_changed(self) -> None:
        self._setup_fixed_joint_widget()
        self._check_input()

    def _check_input(self) -> None:
        """
        Toggle and fills up elements based on validity.

        - Enable/disable the accept button based on validity of user input.
        - Hide/show the fixed joints widgets.
        - fill the placement fields and end-effector if an existing pose
          is selected and valid.
        """
        from ..wb_utils import is_robot

        f = self._form
        input_ok = True
        robot = self._get_robot_from_line_edit()
        if not robot:
            f.fixed_joints_frame.setVisible(False)
            input_ok = False
        if f.existing_pose_radio_button.isChecked():
            pose = self._get_pose_from_line_edit()
            if pose:
                self._placement = pose.Placement
                self._endeffector = pose.EndEffector
                self._fill_placement_from_existing_pose()
                f.endeffector_line_edit.setText(pose.EndEffector)
            else:
                input_ok = False
        if f.manual_pose_radio_button.isChecked():
            self._endeffector = f.endeffector_line_edit.text()
            if is_robot(robot):
                endeffector_link = robot.Proxy.get_link(self._endeffector)
                if not endeffector_link:
                    input_ok = False
            self._placement = fc.Placement(
                    fc.Vector(
                        f.x_spinbox.value(),
                        f.y_spinbox.value(),
                        f.z_spinbox.value(),
                    ),
                    fc.Rotation(
                        f.qx_spinbox.value(),
                        f.qy_spinbox.value(),
                        f.qz_spinbox.value(),
                        f.qw_spinbox.value(),
                    ),
            )

        f.button_box.button(QtGui.QDialogButtonBox.StandardButton.Ok).setEnabled(input_ok)
        self._setup_fixed_joint_widget()
        return

    def _setup_fixed_joint_widget(self) -> None:
        # Import late to avoid slowing down workbench start-up.
        from ..wb_utils import ros_name

        f = self._form
        f.fixed_joints_frame.setVisible(False)
        children = []
        for i in range(f.fixed_joints_grid_layout.count()):
            child = f.fixed_joints_grid_layout.itemAt(i).widget()
            if child:
                children.append(child)
        for child in children:
            f.fixed_joints_grid_layout.removeWidget(child)
            child.setParent(None)
            child.deleteLater()
        self._fixed_joint_widgets.clear()
        robot = self._get_robot_from_line_edit()
        if not robot:
            return
        endeffector = f.endeffector_line_edit.text()
        actuated_chain_joints = robot.Proxy.get_actuated_joints_to(endeffector)
        if not actuated_chain_joints:
            return
        potential_fixed_joints = [ros_name(j) for j in actuated_chain_joints]
        number_of_fixed_joints = len(potential_fixed_joints) - 6
        if number_of_fixed_joints <= 0:
            return
        for _ in range(number_of_fixed_joints):
            combo_box = QtGui.QComboBox(self._form)
            for joint in potential_fixed_joints:
                combo_box.addItem(joint)
            spin_box = QtGui.QDoubleSpinBox(self._form)
            self._fixed_joint_name_changed(combo_box, spin_box)
            combo_box.currentTextChanged.connect(lambda: self._fixed_joint_name_changed(combo_box, spin_box))
            f.fixed_joints_grid_layout.addWidget(combo_box)
            f.fixed_joints_grid_layout.addWidget(spin_box)
            self._fixed_joint_widgets.append((combo_box, spin_box))

        f.fixed_joints_frame.setVisible(True)

    def _fixed_joint_name_changed(
        self,
        combo_box: QtGui.QComboBox,
        spin_box: QtGui.QDoubleSpinBox,
    ) -> None:
        """Set-up the spin box based on the joint type."""
        robot = self._get_robot_from_line_edit()
        if not robot:
            return
        joint_name = combo_box.currentText()
        joint = robot.Proxy.get_joint(joint_name)
        if not joint:
            # Should not happen.
            return
        if joint.Type in ['prismatic', 'revolute']:
            spin_box.setRange(joint.LowerLimit, joint.UpperLimit)
        if joint.Type == 'prismatic':
            spin_box.setSuffix(' mm')
        if joint.Type in ['revolute', 'continuous']:
            spin_box.setSuffix(' °')
        # Get the value as float(fc.Units.Quantity), i.e. in FC unit
        # (mm or deg).
        spin_box.setValue(float(robot.Proxy.get_joint_values()[joint]))
        # TODO(Gaël): nice to have, change other fixed joints to another joint

    def _pick_robot(self) -> None:
        from .object_selector_dialog import ObjectSelector
        from ..wb_utils import is_robot

        dialog = ObjectSelector(
                doc=self._doc,
                filter_func=is_robot,
                parent=self._form,
        )
        if dialog.exec():
            robot = dialog.get_selected_object()
        if robot:
            self._form.robot_line_edit.setText(robot.Label)
        else:
            self._form.robot_line_edit.setText('')

    def _pick_pose(self) -> None:
        from .object_selector_dialog import ObjectSelector
        from ..wb_utils import is_pose

        dialog = ObjectSelector(
                doc=self._doc,
                filter_func=is_pose,
                parent=self._form,
        )
        if dialog.exec():
            pose = dialog.get_selected_object()
            if pose:
                self._form.existing_pose_line_edit.setText(pose.Label)
            else:
                self._form.existing_pose_line_edit.setText('')

    def _pick_link(self) -> None:
        from .object_selector_dialog import ObjectSelector

        robot = self._get_robot_from_line_edit()
        if not robot:
            return
        # TODO(Gaël): propose only ROS names.
        dialog = ObjectSelector(
                doc=self._doc,
                filter_func=lambda obj: robot.Proxy.get_link(obj.Label) is not None,
                parent=self._form,
        )
        if dialog.exec():
            link = dialog.get_selected_object()
            if link:
                self._form.endeffector_line_edit.setText(link.Label)
            else:
                self._form.endeffector_line_edit.setText('')

    def _on_accept(self) -> None:
        self._dialog_confirmed = True
        self._form.close()


def bring_to_pose(robot: CrossRobot, pose: CrossPose):
    # Import late to avoid slowing down workbench start-up.
    from ..freecad_utils import warn

    if not pose.EndEffector:
        warn(f'Pose "{pose.Label}" has no end-effector link', True)
        return
    bring_to_placement(robot, pose.Placement, pose.EndEffector)


def bring_to_placement(
    robot: CrossRobot,
    placement: fc.Placement,
    endeffector: str,
    fixed_joints: dict[str, float] = {}
) -> None:
    # Import late to avoid slowing down workbench start-up.
    from ..freecad_utils import warn
    from ..ik import ik
    from ..wb_utils import joint_values_si_units_from_freecad as wb_si_from_fc

    chain_joints = robot.Proxy.get_actuated_joints_to(endeffector)
    if not chain_joints:
        warn(f'Could not find a kinematic chain from the root link "{robot.Proxy.get_root_link()}" to the end-effector link "{endeffector}" in robot "{robot.Label}"', True)
        return

    sols = ik(
            robot=robot,
            from_link=chain_joints[0].Parent,
            to_link=endeffector,
            target=placement,
            # algorithm: IKAlgorithm = IKAlgorithm.PINOCCHIO_SINGLE,
            fixed_joints=fixed_joints,
    )
    if not sols:
        warn(f'IK failed to find a solution for robot "{robot.Label}" to reach {placement} with end-effector "{endeffector}"', True)
        return

    first_sol_si_dict = wb_si_from_fc({j: sol for j, sol in zip(chain_joints, sols[0])})
    robot.Proxy.set_joint_values(first_sol_si_dict)
    robot.Document.recompute()


fcgui.addCommand('BringRobotToPose', _BringRobotToPoseCommand())

