from __future__ import annotations

import FreeCAD as fc
import FreeCADGui as fcgui

try:
    from PySide import QtGui, QtCore
except:
    from PySide2 import QtGui, QtCore  # FreeCAD's PySide!

from ..freecad_utils import message
from ..gui_utils import tr
from ..wb_utils import is_link, is_robot, get_links, UI_PATH

# Stubs and type hints.
from ..link import Link as CrossLink


class _SelectionObserver:
    """Observer for FreeCAD selection changes.

    Calls the provided callback whenever the selection changes.
    Uses individual selection event hooks for broader compatibility.

    """

    def __init__(self, callback):
        self.callback = callback

    def addSelection(self, doc, obj, sub, pnt) -> None:
        self.callback()

    def removeSelection(self, doc, obj, sub) -> None:
        self.callback()

    def setSelection(self, doc) -> None:
        self.callback()

    def clearSelection(self, doc) -> None:
        self.callback()


def _get_links_from_selection(
        selection: list[fc.DocumentObject],
) -> list[CrossLink]:
    """Filter selection and return only Cross::Link objects.

    If a Cross::Robot is selected, its links are returned instead.
    Non-robot, non-link objects are ignored.

    """
    result: list[CrossLink] = []
    for obj in selection:
        if is_robot(obj):
            if hasattr(obj, 'Proxy') and hasattr(obj.Proxy, 'get_links'):
                result.extend(obj.Proxy.get_links())
        elif is_link(obj):
            result.append(obj)
        # Other objects are ignored.
    return result


class ManageLinkDisplayDialog:
    """Dialog with checkboxes to control Collision/Visual/Real display.

    Also provides a 'Set Placement Mode' button that hides Collision and Visual,
    showing only Real.
    The dialog is non-modal, allowing interaction with FreeCAD while open.
    """

    def __init__(self, links: list[CrossLink]):
        """Constructor with a list of CROSS::Link objects."""
        self.links = links
        self._is_applying = False

        self.form = fcgui.PySideUic.loadUi(
            str(UI_PATH / 'manage_link_display.ui'),
            self,
        )

        self._init_checkboxes()
        self._establish_connections()

        # Make the dialog non-modal so FreeCAD remains interactive.
        self.form.setWindowModality(QtCore.Qt.NonModal)
        self.form.setAttribute(QtCore.Qt.WA_DeleteOnClose, False)

        # Observe selection changes to re-initialize checkboxes.
        self._observer = _SelectionObserver(self._on_selection_changed)
        fcgui.Selection.addObserver(self._observer)

    def _on_selection_changed(self) -> None:
        """Re-initialize checkboxes when FreeCAD selection changes."""
        selection = fcgui.Selection.getSelection()
        new_links = _get_links_from_selection(selection)
        if not new_links:
            return
        self.links = new_links
        self._init_checkboxes()

    def _init_checkboxes(self) -> None:
        """Initialize checkboxes based on the links' current states."""
        if not self.links:
            return

        all_has_real = all(
            bool(link.Real) for link in self.links if is_link(link)
        )
        all_has_visual = all(
            bool(link.Visual) for link in self.links if is_link(link)
        )
        all_has_collision = all(
            bool(link.Collision) for link in self.links if is_link(link)
        )

        self.form.check_real.setEnabled(all_has_real)
        self.form.check_visual.setEnabled(all_has_visual)
        self.form.check_collision.setEnabled(all_has_collision)

        # Block signals during initial setup.
        for cb in (self.form.check_collision, self.form.check_visual,
                   self.form.check_real):
            cb.blockSignals(True)

        if len(self.links) == 1:
            # Single link: show its actual state directly.
            vobj = self.links[0].ViewObject
            if vobj is not None:
                self.form.check_real.setChecked(vobj.ShowReal)
                self.form.check_visual.setChecked(vobj.ShowVisual)
                self.form.check_collision.setChecked(vobj.ShowCollision)
        else:
            # Multiple links: determine mixed state for each property.
            real_values = [
                link.ViewObject.ShowReal
                for link in self.links
                if link.ViewObject is not None
            ]
            visual_values = [
                link.ViewObject.ShowVisual
                for link in self.links
                if link.ViewObject is not None
            ]
            collision_values = [
                link.ViewObject.ShowCollision
                for link in self.links
                if link.ViewObject is not None
            ]

            # Real checkbox.
            if real_values and all(v == real_values[0] for v in real_values):
                self.form.check_real.setChecked(real_values[0])
            else:
                # Mixed: enable tri-state and set partially checked.
                self.form.check_real.setTristate(True)
                self.form.check_real.setCheckState(QtCore.Qt.PartiallyChecked)

            # Visual checkbox.
            if visual_values and all(
                    v == visual_values[0] for v in visual_values):
                self.form.check_visual.setChecked(visual_values[0])
            else:
                self.form.check_visual.setTristate(True)
                self.form.check_visual.setCheckState(QtCore.Qt.PartiallyChecked)

            # Collision checkbox.
            if collision_values and all(
                    v == collision_values[0] for v in collision_values):
                self.form.check_collision.setChecked(collision_values[0])
            else:
                self.form.check_collision.setTristate(True)
                self.form.check_collision.setCheckState(
                    QtCore.Qt.PartiallyChecked,
                )

        for cb in (self.form.check_collision, self.form.check_visual,
                   self.form.check_real):
            cb.blockSignals(False)

    def _establish_connections(self) -> None:
        """Connect signals to slots.

        Uses `clicked` signal which only fires on actual user clicks,
        not on programmatic setChecked/setCheckState calls.

        """
        self.form.check_collision.clicked.connect(
            lambda checked: self._on_checkbox_clicked(
                self.form.check_collision, 'ShowCollision', checked,
            ),
        )
        self.form.check_visual.clicked.connect(
            lambda checked: self._on_checkbox_clicked(
                self.form.check_visual, 'ShowVisual', checked,
            ),
        )
        self.form.check_real.clicked.connect(
            lambda checked: self._on_checkbox_clicked(
                self.form.check_real, 'ShowReal', checked,
            ),
        )
        self.form.btn_set_placement_mode.clicked.connect(
            self._on_set_placement_mode,
        )
        self.form.button_box.accepted.connect(self._on_close)
        # Also clean up when dialog is closed via the window X button.
        self.form.finished.connect(self._on_close)

    def _on_checkbox_clicked(
            self, cb: QtGui.QCheckBox, prop: str, checked: bool,
    ) -> None:
        """Handle user click on a checkbox.

        If the checkbox was in tri-state mode (mixed state), disable tri-state
        and update to the clicked state. Then apply to all links.

        """
        if self._is_applying:
            return

        self._is_applying = True

        # If in tri-state mode (was mixed), disable tri-state and force state.
        if cb.isTristate():
            cb.blockSignals(True)
            cb.setTristate(False)
            cb.setChecked(checked)
            cb.blockSignals(False)

        # Apply to all links.
        for link in self.links:
            if not is_link(link):
                continue
            vobj = link.ViewObject
            if vobj is None:
                continue
            setattr(vobj, prop, checked)

        doc = fc.activeDocument()
        if doc:
            doc.recompute()

        self._is_applying = False

    def _on_set_placement_mode(self) -> None:
        """Hide Collision and Visual, show Real."""
        self._is_applying = True

        for cb in (self.form.check_collision, self.form.check_visual,
                   self.form.check_real):
            cb.blockSignals(True)

        for link in self.links:
            if not is_link(link):
                continue
            vobj = link.ViewObject
            if vobj is None:
                continue
            vobj.ShowCollision = False
            vobj.ShowVisual = False
            vobj.ShowReal = True

        # Reset all checkboxes to two-state mode and set values.
        for cb in (self.form.check_collision, self.form.check_visual,
                   self.form.check_real):
            cb.setTristate(False)

        self.form.check_collision.setChecked(False)
        self.form.check_visual.setChecked(False)
        self.form.check_real.setChecked(True)

        for cb in (self.form.check_collision, self.form.check_visual,
                   self.form.check_real):
            cb.blockSignals(False)

        doc = fc.activeDocument()
        if doc:
            doc.recompute()

        self._is_applying = False

        message(
            tr(
                'Placement Mode: Collision and Visual hidden,'
                ' Real shown for selected links.',
            ),
        )

    def _on_close(self) -> None:
        """Handle dialog close event."""
        if hasattr(self, '_closed') and self._closed:
            return
        self._closed = True
        fcgui.Selection.removeObserver(self._observer)
        self.form.close()

    def show(self) -> None:
        """Show the dialog non-modally."""
        self.form.show()
