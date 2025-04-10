from __future__ import annotations

from typing import Optional

import FreeCAD as fc
import FreeCADGui as fcgui

from ..kk_robot import KKRobot
from ..wb_utils import UI_PATH
from .dh_model import DHModel
from .kk_model import KKModel

# Stubs and type hints.
from ..robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
DO = fc.DocumentObject  # A FreeCAD DocumentObject.


class KKDialog:
    """A dialog to input DH and KK parameters.

    A dialog to input Denavit-Hartenberg (DH) and
    Khalil-Kleinfinger (KK) parameters.

    """

    def __init__(self, robot: CrossRobot):
        """Constructor with a CROSS::Robot."""
        self.robot = robot
        self.kk_robot = KKRobot()
        self.kk_robot.set_from_robot(robot)

        self.form = fcgui.PySideUic.loadUi(
                str(UI_PATH / 'kk_dialog.ui'),
                fcgui.getMainWindow(),
        )
        self.dialog_confirmed = False

        # Disable the KK tab until supported.
        self.form.tab_widget.removeTab(1)

        self.dh_robot_model = DHModel(self.kk_robot)
        self.kk_robot_model = KKModel(self.kk_robot)
        self.form.table_view_dh.setModel(self.dh_robot_model)
        self.form.table_view_kk.setModel(self.kk_robot_model)

        self.form.push_button_add_joint.clicked.connect(self._on_add_joint)
        self.form.button_box.accepted.connect(self._on_accept)
        self.form.button_box.rejected.connect(self._on_cancel)

    def exec_(self) -> Optional[KKRobot]:
        self.form.exec_()
        if not self.dialog_confirmed:
            return
        return self.kk_robot

    def close(self) -> None:
        self.form.close()

    def _on_add_joint(self) -> None:
        # Call add_joint() only on one of self.kk_robot_model
        # or self.dh_robot_model because they share the same kk_robot.
        self.kk_robot_model.add_joint()
        # The modelReset() signal must be emitted on both, though.
        self.dh_robot_model.modelReset.emit()
        self.kk_robot_model.modelReset.emit()

    def _on_accept(self) -> None:
        self.dialog_confirmed = True
        self.form.close()

    def _on_cancel(self) -> None:
        self.dialog_confirmed = False
        self.form.close()
