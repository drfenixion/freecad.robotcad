from __future__ import annotations

from pathlib import Path
from typing import Optional

import FreeCADGui as fcgui

from PySide import QtCore  # FreeCAD's PySide!
from PySide import QtGui  # FreeCAD's PySide!

from ..wb_utils import UI_PATH
from ..gui_utils import tr


class CheckBoxAndTextItem(QtGui.QListWidgetItem):
    """A wrapper around QListWidgetItem for a more convenient constructor."""

    def __init__(
        self,
        text: str,
        checked: bool = False,
        parent: Optional[QtGui.QListWidget] = None,
    ) -> None:
        super().__init__(text, parent)
        self.setCheckState(QtCore.Qt.Checked if checked else QtCore.Qt.Unchecked)

    def is_checked(self) -> bool:
        return self.checkState() == QtCore.Qt.Checked

    def setToolTipWithoutSignal(self, text: str) -> None:
        # `with QtCore.QSignalBlocker(self.listWidget()):` does not work.
        blocker = QtCore.QSignalBlocker(self.listWidget())
        blocker.reblock()
        self.setToolTip(text)
        blocker.unblock()


class FileOverwriteConfirmationDialog:
    """A dialog that asks the user to confirm overwriting files."""

    def __init__(
        self,
        path: [Path | str],
        filenames: list[str],
    ):
        """Constructor with a path and relative file names."""
        self.path = Path(path)
        self.filenames = filenames

        # Dialog output, internally as set, returned as list.
        self.files_to_ignore: set[str] = set()  # No write, unchecked.
        self.files_to_write: set[str] = set()  # Non-existing, checked.
        self.files_to_overwrite: set[str] = set()  # Existing, checked.

        self.form = fcgui.PySideUic.loadUi(
                str(UI_PATH / 'file_overwrite_confirmation_dialog.ui'), self,
        )

        self.form.line_edit_save_path.setText(str(self.path))

        # Checkable file entries.
        self.checkable_file_items: list[CheckBoxAndTextItem] = []
        self._fill_file_list()

        self.form.line_edit_save_path.textChanged.connect(self._on_path_changed)
        self.form.push_button_browse.clicked.connect(self._on_browse_clicked)
        self.form.select_all_check_box.stateChanged.connect(self._on_select_all_changed)
        self.form.list_widget_files.itemChanged.connect(self._on_list_item_changed)
        self.form.push_button_generate.clicked.connect(self._on_generate)
        self.form.push_button_cancel.clicked.connect(self._on_cancel)

    def _fill_file_list(self) -> None:
        """Fills out self.form.list_widget_files."""
        self.checkable_file_items.clear()
        self.form.list_widget_files.clear()
        self.files_to_ignore.clear()
        self.files_to_write.clear()
        self.files_to_overwrite.clear()
        for f in self.filenames:
            abs_path = self.path / f
            checkable_file_item = CheckBoxAndTextItem(
                    f, not abs_path.exists(),
                    self.form.list_widget_files,
            )
            if abs_path.exists():
                self.files_to_ignore.add(f)
                checkable_file_item.setToolTipWithoutSignal(tr('This file will be ignored.'))
            else:
                self.files_to_write.add(f)
                checkable_file_item.setToolTipWithoutSignal(tr('This file will be generated.'))
            self.checkable_file_items.append(checkable_file_item)
            self.form.list_widget_files.addItem(checkable_file_item)

        self._set_select_all_state()

    def exec_(self) -> tuple[list[str], list[str], list[str]]:
        self.form.exec_()
        return (
                list(self.files_to_ignore),
                list(self.files_to_write),
                list(self.files_to_overwrite),
        )

    def close(self) -> None:
        self.form.close()

    def _on_path_changed(self, path: str) -> None:
        """Updates the file list when the path is changed."""
        self.path = Path(path)
        self._fill_file_list()

    def _on_browse_clicked(self) -> None:
        """Opens a file dialog to select a directory."""
        path = QtGui.QFileDialog.getExistingDirectory(
                None, tr('Select a directory'), str(self.path),
        )
        if path:
            self.path = Path(path)
            self.form.line_edit_save_path.setText(str(self.path))
            self._fill_file_list()

    def _on_select_all_changed(self, state: QtCore.Qt.CheckState) -> None:
        """Checks or unchecks all items."""
        if state == QtCore.Qt.PartiallyChecked:
            return
        for item in self.checkable_file_items:
            # Implementation note: state is actually an int because
            # self.form.select_all_check_box has 3 states, not allowed.
            item.setCheckState(
                QtCore.Qt.Checked if state == QtCore.Qt.Checked
                else QtCore.Qt.Unchecked,
            )

    def _set_select_all_state(self) -> None:
        """Sets the state of `self.form.select_all_check_box`."""
        # Implementation note: `with QtCore.QSignalBlocker(self.form.select_all_check_box):` does not work.
        self.form.select_all_check_box.blockSignals(True)
        checked = False
        unchecked = False
        for item in self.checkable_file_items:
            if item.is_checked():
                checked = True
            else:
                unchecked = True

        if checked and unchecked:
            self.form.select_all_check_box.setCheckState(QtCore.Qt.PartiallyChecked)
        elif not checked:
            self.form.select_all_check_box.setCheckState(QtCore.Qt.Unchecked)
            self.form.select_all_check_box.setTristate(False)
        else:
            self.form.select_all_check_box.setCheckState(QtCore.Qt.Checked)
            self.form.select_all_check_box.setTristate(False)
        self.form.select_all_check_box.blockSignals(False)

    def _on_list_item_changed(
        self,
        item: CheckBoxAndTextItem,
    ) -> None:
        """Checks or unchecks the item."""
        filename = item.text()
        if item.is_checked():
            if filename in self.files_to_ignore:
                self.files_to_ignore.remove(filename)
            if (self.path / filename).exists():
                self.files_to_overwrite.add(filename)
                item.setToolTipWithoutSignal(tr('This file will be overwritten!'))
            else:
                self.files_to_write.add(filename)
                item.setToolTipWithoutSignal(tr('This file will be generated.'))
        else:
            if filename in self.files_to_write:
                self.files_to_write.remove(filename)
            if filename in self.files_to_overwrite:
                self.files_to_overwrite.remove(filename)
            self.files_to_ignore.add(filename)
            item.setToolTipWithoutSignal(tr('This file will be ignored.'))
        self._set_select_all_state()

    def _on_generate(self) -> None:
        self.form.close()

    def _on_cancel(self) -> None:
        self.files_to_ignore = self.filenames.copy()
        self.files_to_write.clear()
        self.files_to_overwrite.clear()
        self.form.close()
