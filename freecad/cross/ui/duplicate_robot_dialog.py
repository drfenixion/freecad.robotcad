from __future__ import annotations

import FreeCADGui as fcgui


class DuplicateRobotDialog:
    """A dialog to get the parameters for robot duplication.

    """

    def __init__(self, base_name: str = 'robot'):
        """Constructor with a CROSS::Robot."""
        # Import late to avoid slowing down workbench start-up.
        from ..wb_utils import UI_PATH

        self.number_of_duplicates = 0
        self.base_name = base_name

        self.form = fcgui.PySideUic.loadUi(
                str(UI_PATH / 'duplicate_robot_dialog.ui'),
                fcgui.getMainWindow(),
        )

        self.form.line_edit_basename.setText(base_name)

        self.form.button_box.accepted.connect(self._on_accept)
        self.form.button_box.rejected.connect(self._on_cancel)

    def exec(self) -> int:
        self.form.exec()
        return self.number_of_duplicates

    def close(self) -> None:
        self.form.close()

    def _on_accept(self) -> None:
        self.number_of_duplicates = self.form.spinbox_number.value()
        self.base_name = self.form.line_edit_basename.text()
        self.form.close()

    def _on_cancel(self) -> None:
        self.form.close()
