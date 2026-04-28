from __future__ import annotations

import json

import FreeCAD as fc
import FreeCADGui as fcgui

try:
    from PySide import QtGui, QtCore
except:
    from PySide2 import QtGui, QtCore  # FreeCAD's PySide!

from ..freecad_utils import message
from ..gui_utils import tr
from ..link_proxy import explode_link
from ..wb_utils import is_link, is_robot, UI_PATH

# Stubs and type hints.
from ..link import Link as CrossLink


def _serialize_state(links: list[CrossLink]) -> str:
    """Serialize sorted link names and their MountedPlacement.Base to JSON.

    Links are sorted by their Name for deterministic comparison.
    """
    sorted_links = sorted(links, key=lambda l: l.Name)
    state_data = []
    for link in sorted_links:
        base = link.MountedPlacement.Base
        state_data.append({
            'name': link.Name,
            'x': base.x,
            'y': base.y,
            'z': base.z,
        })
    return json.dumps(state_data)


def _deserialize_state(state_json: str) -> list[dict]:
    """Deserialize a state JSON string to a list of dicts."""
    return json.loads(state_json)


def _get_robot_from_links(links: list[CrossLink]):
    """Get the robot that owns these links, or None."""
    for link in links:
        if hasattr(link, 'Proxy') and hasattr(link.Proxy, 'get_robot'):
            robot = link.Proxy.get_robot()
            if robot:
                return robot
    return None


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
        self._cancelling = False  # Guard against recursion in _on_cancel

        # Get the robot to store/load states
        self.robot = _get_robot_from_links(links)

        self._setup_ui()
        self._establish_connections()

        # Auto-add current state if it doesn't exist yet
        self._ensure_current_state_exists()
        self._rebuild_state_buttons()

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
        # Also handle window close (X button) - restore original positions.
        # rejected is emitted by reject(), but we guard against recursion
        # by checking dialog_confirmed flag.
        self.form.rejected.connect(self._on_cancel)

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
            link_offset = (i + 1) * value

            # Reset to original then apply offset via explode_link
            link.MountedPlacement.Base = self.original_bases[i]
            explode_link(link, link_offset)

        doc.recompute()

    def _ensure_current_state_exists(self) -> None:
        """Auto-add current state if it doesn't exist in saved states."""
        if not self.robot:
            return

        current_state_json = _serialize_state(self.links)
        states = list(self.robot.ExplodeViewStates) if self.robot.ExplodeViewStates else []

        for existing_state in states:
            if existing_state == current_state_json:
                return  # Already exists

        # Add current state as the first state
        states.append(current_state_json)
        self.robot.ExplodeViewStates = states

    def _on_save_clicked(self) -> None:
        """Save current exploded positions as a new state."""
        if not self.robot:
            message(tr('Cannot save state: no robot found.'))
            return

        # Check if current state already exists (compare with saved states)
        current_state_json = _serialize_state(self.links)
        states = list(self.robot.ExplodeViewStates) if self.robot.ExplodeViewStates else []

        for existing_state in states:
            if existing_state == current_state_json:
                message(tr('Current state already saved.'))
                return

        # Add new state
        states.append(current_state_json)
        self.robot.ExplodeViewStates = states

        # Rebuild state buttons
        self._rebuild_state_buttons()

        message(tr(f'State {len(states)} saved.'))

    def _rebuild_state_buttons(self) -> None:
        """Rebuild the state buttons in the scroll area."""
        if not self.robot:
            return

        # Clear existing buttons
        layout = self.form.states_scroll_layout
        while layout.count() > 0:
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        states = list(self.robot.ExplodeViewStates) if self.robot.ExplodeViewStates else []

        for idx, state_json in enumerate(states):
            # Create a horizontal layout for each state row
            row_widget = QtGui.QWidget()
            row_layout = QtGui.QHBoxLayout(row_widget)
            row_layout.setContentsMargins(0, 0, 0, 0)

            # State button - use lambda with default argument to capture idx
            state_btn = QtGui.QPushButton(tr(f'State {idx + 1}'))
            state_btn.setToolTip(tr(f'Restore state {idx + 1}'))
            state_btn.clicked.connect(
                lambda checked, i=idx: self._on_state_clicked(i)
            )

            # Delete button
            delete_btn = QtGui.QPushButton('✕')
            delete_btn.setFixedWidth(30)
            delete_btn.setToolTip(tr(f'Delete state {idx + 1}'))
            delete_btn.clicked.connect(
                lambda checked, i=idx: self._on_delete_state_clicked(i)
            )

            row_layout.addWidget(state_btn)
            row_layout.addWidget(delete_btn)

            layout.addWidget(row_widget)

        # Add spacer at the end
        spacer = QtGui.QSpacerItem(0, 0, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        layout.addItem(spacer)

    def _on_state_clicked(self, state_index: int) -> None:
        """Restore a saved state."""
        states = list(self.robot.ExplodeViewStates) if self.robot.ExplodeViewStates else []
        if state_index < 0 or state_index >= len(states):
            return

        state_data = _deserialize_state(states[state_index])

        # Build a map of link name -> position from state data
        state_positions = {}
        for entry in state_data:
            state_positions[entry['name']] = fc.Base.Vector(entry['x'], entry['y'], entry['z'])

        # Apply state positions to links
        doc = fc.activeDocument()
        if doc is None:
            return

        for link in self.links:
            if not is_link(link):
                continue
            if link.Name in state_positions:
                link.MountedPlacement.Base = state_positions[link.Name]
                # link.ViewObject.ShowReal = False
                # link.ViewObject.ShowReal = True

        doc.recompute()

        # Update original_bases to match the restored state
        for i, link in enumerate(self.links):
            if is_link(link) and link.Name in state_positions:
                base = link.MountedPlacement.Base
                self.original_bases[i] = fc.Base.Vector(base.x, base.y, base.z)

        # Reset slider to 0
        self.form.explode_slider.blockSignals(True)
        self.form.explode_slider.setValue(0)
        self.form.explode_slider.blockSignals(False)

        # message(tr(f'State {state_index + 1} restored.'))

    def _on_delete_state_clicked(self, state_index: int) -> None:
        """Delete a saved state with confirmation."""
        states = list(self.robot.ExplodeViewStates) if self.robot.ExplodeViewStates else []
        if state_index < 0 or state_index >= len(states):
            return

        # Confirmation dialog
        reply = QtGui.QMessageBox.question(
            self.form,
            tr('Delete State'),
            tr(f'Are you sure you want to delete State {state_index + 1}?'),
            QtGui.QMessageBox.Yes | QtGui.QMessageBox.No,
            QtGui.QMessageBox.No,
        )

        if reply != QtGui.QMessageBox.Yes:
            return

        # Remove the state
        states.pop(state_index)
        self.robot.ExplodeViewStates = states

        # Rebuild buttons
        self._rebuild_state_buttons()

        message(tr(f'State {state_index + 1} deleted.'))

    def _on_accept(self) -> None:
        """Handle dialog acceptance."""
        self.dialog_confirmed = True
        self.form.accept()

    def _on_cancel(self) -> None:
        """Handle dialog cancellation."""
        if self._cancelling:
            # Guard against recursion: rejected signal is emitted by reject(),
            # so calling reject() inside a rejected handler would loop.
            return
        self._cancelling = True
        self.dialog_confirmed = False
        # Restore original placements
        for i, link in enumerate(self.links):
            # if is_link(link):
            link.MountedPlacement.Base = self.original_bases[i]
            # link.ViewObject.ShowReal = False
            # link.ViewObject.ShowReal = True

        if fc.activeDocument():
            fc.activeDocument().recompute()
        self.form.reject()

