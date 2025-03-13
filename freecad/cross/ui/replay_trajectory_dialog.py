from __future__ import annotations

import FreeCADGui as fcgui

try:
    from PySide import QtGui
except:
    from PySide2 import QtGui # FreeCAD's PySide!

from ..freecad_utils import warn
from ..wb_utils import ICON_PATH
from ..wb_utils import UI_PATH

# Stubs and type hints.
from ..trajectory import Trajectory as CrossTrajectory  # A Cross::Trajectory. # noqa: E501


class ReplayTrajectoryDialog:

    def __init__(self, trajectory: CrossTrajectory):
        """Constructor with a CROSS::Robot."""
        self.trajectory = trajectory

        self.form = fcgui.PySideUic.loadUi(
                str(UI_PATH / 'replay_trajectory_dialog.ui'),
                self,
                )
        self.dialog_confirmed = False

        self._set_icons()
        self._establish_connections()
        # Hide not implemented buttons.
        self.form.play_button.hide()
        self.form.pause_button.hide()
        self.form.loop_button.hide()

        try:
            self.point_index = self.trajectory.PointIndex
            self.max_point_index = self.trajectory.Proxy.point_count - 1
        except AttributeError as e:
            warn(f'Internal error, please report: {e}')

    def _set_icons(self) -> None:
        self.form.play_button.setIcon(
                QtGui.QIcon(str(ICON_PATH / 'media-playback-start.svg')))
        self.form.play_button.setText('')
        self.form.pause_button.setIcon(
                QtGui.QIcon(str(ICON_PATH / 'media-playback-pause.svg')))
        self.form.pause_button.setText('')
        self.form.previous_button.setIcon(
                QtGui.QIcon(str(ICON_PATH / 'media-skip-backward.svg')))
        self.form.previous_button.setText('')
        self.form.next_button.setIcon(
                QtGui.QIcon(str(ICON_PATH / 'media-skip-forward.svg')))
        self.form.next_button.setText('')

    def _establish_connections(self) -> None:
        self.form.index_spin_box.valueChanged.connect(self._on_spin_box_changed)
        self.form.index_slider.valueChanged.connect(self._on_slider_changed)
        self.form.previous_button.clicked.connect(self._on_previous_clicked)
        self.form.next_button.clicked.connect(self._on_next_clicked)
        self.form.button_box.accepted.connect(self._on_accept)
        self.form.button_box.rejected.connect(self._on_cancel)

    def exec_(self) -> int:
        self.form.exec_()
        if not self.dialog_confirmed:
            return -1
        return self.form.index_spin_box.value()

    def _on_spin_box_changed(self) -> None:
        point_index = self.form.index_spin_box.value()
        self.form.index_slider.blockSignals(True)
        self.form.index_slider.setValue(point_index)
        self.form.index_slider.blockSignals(False)
        self._update_trajectory()

    def _on_slider_changed(self) -> None:
        point_index = self.form.index_slider.value()
        self.form.index_spin_box.setValue(point_index)

    def _on_previous_clicked(self) -> None:
        self.point_index -= 1

    def _on_next_clicked(self) -> None:
        self.point_index += 1

    def _on_accept(self) -> None:
        self.dialog_confirmed = True

    def _on_cancel(self) -> None:
        self.dialog_confirmed = False

    def _update_trajectory(self) -> None:
        try:
            self.trajectory.PointIndex = self.point_index
            self.trajectory.Document.recompute()
        except AttributeError as e:
            warn(f'Internal error, please report: {e}')

    def _toggle_button_activation(self) -> None:
        if self.point_index == 0:
            self.form.previous_button.setEnabled(False)
        if self.point_index == self.max_point_index:
            self.form.next_button.setEnabled(False)

    @property
    def point_index(self) -> int:
        return self.form.index_spin_box.value()

    @point_index.setter
    def point_index(self, value: int) -> None:
        # Next line will also update self.form.index_spin_box.
        # The widgets clamp the value themselves.
        self.form.index_slider.setValue(value)

    @property
    def max_point_index(self) -> int:
        return self.form.index_spin_box.maximum()

    @max_point_index.setter
    def max_point_index(self, value: int) -> None:
        self.form.index_spin_box.setMaximum(value)
        self.form.index_slider.setMaximum(value)

    def close(self) -> None:
        self.form.close()
