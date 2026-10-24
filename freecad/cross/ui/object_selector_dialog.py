"""A dialog to choose from existing objects in a document."""

from collections.abc import Callable
from typing import TypeAlias

import FreeCAD as fc
from PySide import QtGui  # FreeCAD's PySide!

# Typing hints.
DO: TypeAlias = fc.DocumentObject


class ObjectSelector(QtGui.QDialog):
    """A dialog to choose from existing objects in a document."""

    def __init__(
        self,
        doc: fc.Document,
        filter_func: Callable | None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._doc = doc
        if filter_func is None:
            def filter_func(_):
                return True

        self.setWindowTitle("Select Object")
        layout = QtGui.QVBoxLayout(self)

        # Create a list widget and populate it with document objects
        self.list_widget = QtGui.QListWidget()
        for obj in doc.Objects:
            if filter_func(obj):
                self.list_widget.addItem(obj.Label + " (" + obj.Name + ")")

        layout.addWidget(self.list_widget)

        self.select_button = QtGui.QPushButton("Select")
        self.select_button.clicked.connect(self.accept)
        layout.addWidget(self.select_button)

    def _has_correct_type(self, obj: DO) -> bool:
        # Import here to avoid slowing down workbench start-up.
        from ..wb_utils import has_cross_type

        for cross_type in self._cross_types:
            if has_cross_type(obj, cross_type):
                return True
        return False

    def get_selected_object(self) -> DO | None:
        current_item = self.list_widget.currentItem()
        if current_item:
            # Extract the internal Name from the string "Label (Name)"
            obj_name = current_item.text().split('(')[-1].strip(')')
            return self._doc.getObject(obj_name)
        return None

