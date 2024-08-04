from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from PySide import QtGui  # FreeCAD's PySide!
from PySide import QtCore  # FreeCAD's PySide!

from ..freecad_utils import quantity_as
from ..gui_utils import tr
from ..utils import values_from_string
from ..wb_utils import UI_PATH
from ..wb_utils import ros_name
from .set_joints_manual_input_table import SetJointsManualInputTable

# Stubs and type hints.
from ..joint import Joint as CrossJoint  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint". # noqa: E501
from ..robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
DO = fc.DocumentObject  # A FreeCAD DocumentObject.


class SetJointsDialog(QtGui.QDialog):
    """A dialog to input DH and KK parameters.

    A dialog to set some of the joint values of the robot passed to the
    constuctor.

    """

    def __init__(self, robot: CrossRobot, *args):
        """Constructor with a CROSS::Robot."""
        super().__init__(args[0] if args else None)
        self.robot = robot

        self.form = fcgui.PySideUic.loadUi(str(UI_PATH / 'set_joints.ui'), self)
        self.dialog_confirmed = False
        self.table_manual_input = SetJointsManualInputTable(self.robot, self.form)
        self._set_up_gui()
        self._set_up_callback()
        # Set the conversion buttons after filling up the table (in its constructor).
        self._set_conversion_buttons()

        # Joint values for the changed joints in meters and radians.
        self._joint_values: dict[CrossJoint, fc.Units.Quantity] = {}

        self.table_manual_input.reorder_values = (
                self.form.check_box_reorder_values.checkState() == QtCore.Qt.Checked
        )

        # Deactivate the `tab_input_topic` tab until supported.
        # TODO: implement the `tab_input_topic` tab.
        self.form.tab_input_type.removeTab(1)

    @property
    def joint_values(self) -> dict[CrossJoint, float]:
        """Joint values for the changed joints in meters and radians."""
        return self._joint_values

    def _set_up_gui(self) -> None:
        """Set up the GUI."""
        # Remove the default focus on the OK button (and the Cancel button).
        self.form.button_box.setFocusPolicy(QtCore.Qt.NoFocus)
        # buttonBox->button(QDialogButtonBox::Save)->setDefault(false);
        # buttonBox->button(QDialogButtonBox::Cancel)->setDefault(false);
        self.form.table_container_widget.layout().addWidget(self.table_manual_input)
        self.table_manual_input.setToolTip(tr('Drag and drop to reorder'))

    def _set_up_callback(self) -> None:
        """Set up the callback to update the joint values."""
        self.form.button_box.accepted.connect(self._on_accept)
        self.form.button_box.rejected.connect(self._on_cancel)
        self.table_manual_input.itemChanged.connect(self._on_table_edited)
        self.form.line_edit_joint_values.textChanged.connect(
                self._on_line_edited,
        )
        self.form.line_edit_joint_values.editingFinished.connect(
                self._on_line_edited,
        )

        # TODO: Use only two dynamic buttons depending on the table content.
        # `pushbutton_unit_x_to_x` and `pushbutton_unit_x_to_y` are connected
        # dynamically in `_set_conversion_buttons()`.

        def _on_check_box_reorder_values_state_changed(state: int) -> None:
            """Set the reorder values flag."""
            self.table_manual_input.reorder_values = (state == QtCore.Qt.Checked)
        self.form.check_box_reorder_values.stateChanged.connect(
                _on_check_box_reorder_values_state_changed,
        )

    def exec_(self) -> dict[CrossJoint, float]:
        self.form.exec_()
        if self.dialog_confirmed:
            return self.joint_values
        return {}

    def close(self) -> None:
        self.form.close()

    def _on_accept(self) -> None:
        self.dialog_confirmed = True
        self.form.close()

    def _on_cancel(self) -> None:
        self._joint_values.clear()
        self.dialog_confirmed = False
        self.form.close()

    def _on_line_edited(self) -> None:
        """Update the joint values from the line edit."""
        joint_name_column = 0
        value_column = 1
        unit_column = 2
        table = self.table_manual_input
        joint_values_str = self.form.line_edit_joint_values.text()
        joint_values = values_from_string(joint_values_str)
        joint_names = [item.text() for item in column_items(table, joint_name_column)]
        joints: list[CrossJoint] = self.robot.Proxy.joint_variables.keys()
        for i, (joint, value) in enumerate(zip(joints, joint_values)):
            joint_row = joint_names.index(ros_name(joint))
            unit = table.item(joint_row, unit_column).text()
            quantity = fc.Units.Quantity(f'{value} {unit}')
            if joint.Type == 'prismatic':
                value = quantity_as(quantity, 'm')
            else:
                # joint.Type in ['revolute', 'continuous']
                value = quantity_as(quantity, 'rad')
            # We set the joint value directly, without going through the
            # robot in order to avoid recomputing the robot at each joint value
            # change.
            item = QtGui.QTableWidgetItem(f'{quantity_as(quantity, unit)}')
            table.setItem(i, value_column, item)

    def _on_table_edited(self) -> None:
        """Update the joint values from the table."""
        joint_name_column = 0
        value_column = 1
        unit_column = 2
        table = self.table_manual_input
        joint_names = [item.text() for item in column_items(table, joint_name_column) if hasattr(item, 'text')]
        joints: list[CrossJoint] = self.robot.Proxy.joint_variables.keys()
        for i, joint in enumerate(joints):
            if ros_name(joint) not in joint_names:
                warning = tr(f'Joint {joint.Name} not found in the table')
                continue
            joint_row = joint_names.index(ros_name(joint))
            unit = table.item(joint_row, unit_column).text()
            value = table.item(joint_row, value_column).text()
            quantity = fc.Units.Quantity(f'{value} {unit}')
            self._joint_values.update({joint: quantity})
        self.robot.Proxy.set_joint_values(self._joint_values)
        self.robot.Proxy.compute_poses()
        self.robot.Document.recompute()
        self._set_conversion_buttons()

    def _set_conversion_buttons(self) -> None:
        """Set the conversion buttons."""
        row_count = self.table_manual_input.rowCount()
        if row_count == 0:
            return
        unit_column = 2
        unit = self.table_manual_input.item(0, unit_column).text()
        if unit in ['mm', 'deg']:
            self.form.push_button_unit_x_to_x.setText(tr(f'1° → 1 rad'))
            self.form.push_button_unit_x_to_y.setText(tr(f'180° → π rad'))
            # self.form.push_button_unit_x_to_x.clicked.disconnect()
            # self.form.push_button_unit_x_to_y.clicked.disconnect()
            self.form.push_button_unit_x_to_x.clicked.connect(
                    lambda: self.table_manual_input.to_m_rad(False),
            )
            self.form.push_button_unit_x_to_y.clicked.connect(
                    lambda: self.table_manual_input.to_m_rad(True),
            )
        elif unit in ['m', 'rad']:
            self.form.push_button_unit_x_to_x.setText(tr(f'1 rad → 1°'))
            self.form.push_button_unit_x_to_y.setText(tr(f'π rad → 180°'))
            # self.form.push_button_unit_x_to_x.clicked.disconnect()
            # self.form.push_button_unit_x_to_y.clicked.disconnect()
            self.form.push_button_unit_x_to_x.clicked.connect(
                    lambda: self.table_manual_input.to_mm_deg(False),
            )
            self.form.push_button_unit_x_to_y.clicked.connect(
                    lambda: self.table_manual_input.to_mm_deg(True),
            )


def column_items(
    table: QtGui.QTableWidget,
    column: int,
) -> list[QtGui.QTableWidgetItem]:
    """Return the items in a column of a table."""
    row_count = table.rowCount()
    return [table.item(row, column) for row in range(row_count)]
