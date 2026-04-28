from __future__ import annotations

import FreeCAD as fc
import FreeCADGui as fcgui

try:
    from PySide import QtGui, QtCore
except:
    from PySide2 import QtGui, QtCore  # FreeCAD's PySide!

from ..freecad_utils import message
from ..gui_utils import tr
from ..link_proxy import explode_link
from ..wb_utils import is_link, UI_PATH

# Stubs and type hints.
from ..link import Link as CrossLink


class ExplodeLinksDialog:
    """Dialog with a horizontal slider to control explode links offset."""

    def __init__(self, links: list[CrossLink]):
        """Constructor with a list of CROSS::Link objects."""
        self.links = links
        self.num_links = len(links)
        self.original_bases = []

        # Store original MountedPlacement.Base for each link
        # Base.Vector doesn't have copy(), so we create new Vector from components
        for link in links:
            base = link.MountedPlacement.Base
            self.original_bases.append(fc.Base.Vector(base.x, base.y, base.z))

        self.form = fcgui.PySideUic.loadUi(
            str(UI_PATH / 'explode_links_dialog.ui'),
            self,
        )
        self.dialog_confirmed = False

        self._setup_ui()
        self._establish_connections()

    def _setup_ui(self) -> None:
        """Setup the UI elements."""
        # Set window title
        self.form.setWindowTitle(tr('Explode Links'))

        # Set slider range: 0 to 1000
        self.form.explode_slider.setMinimum(0)
        self.form.explode_slider.setMaximum(1000)
        self.form.explode_slider.setValue(0)

    def _establish_connections(self) -> None:
        """Connect signals to slots."""
        self.form.explode_slider.valueChanged.connect(self._on_slider_changed)
        self.form.save_button.clicked.connect(self._on_save_clicked)
        self.form.button_box.accepted.connect(self._on_accept)
        self.form.button_box.rejected.connect(self._on_cancel)

    def exec_(self) -> int:
        """Execute the dialog."""
        self.form.exec_()
        return 0

    def _on_slider_changed(self, value: int) -> None:
        """Handle slider value change and apply explode to links."""
        doc = fc.activeDocument()
        if doc is None:
            return

        for i, link in enumerate(self.links):
            if not is_link(link):
                continue

            # Calculate offset for this link.
            # Slider position 0 = no explosion (links at original position)
            # Moving slider right applies offset based on slider position
            # Each link gets offset based on its index to spread them out
            
            # If we want them to spread out:
            link_offset = (i + 1) * value
            
            # Reset to original then apply offset via explode_link
            link.MountedPlacement.Base = self.original_bases[i]
            explode_link(link, link_offset)

        doc.recompute()

    def _on_save_clicked(self) -> None:
        """Save current exploded positions as new original positions."""
        for i, link in enumerate(self.links):
            if is_link(link):
                base = link.MountedPlacement.Base
                self.original_bases[i] = fc.Base.Vector(base.x, base.y, base.z)

        # Reset slider to 0 since current position is now the "zero" position
        self.form.explode_slider.blockSignals(True)
        self.form.explode_slider.setValue(0)
        self.form.explode_slider.blockSignals(False)

        message(tr('Exploded position saved.'))

    def _on_accept(self) -> None:
        """Handle dialog acceptance."""
        self.dialog_confirmed = True
        self.form.accept()

    def _on_cancel(self) -> None:
        """Handle dialog cancellation."""
        self.dialog_confirmed = False
        # Restore original placements
        for i, link in enumerate(self.links):
            if is_link(link):
                link.MountedPlacement.Base = self.original_bases[i]
                # link.ViewObject.ShowReal = False
                # link.ViewObject.ShowReal = True
        
        if fc.activeDocument():
            fc.activeDocument().recompute()
        self.form.reject()
