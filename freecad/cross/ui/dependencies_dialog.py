"""Interactive dialog to check and install the workbench dependencies.

The dependency list is read from ``package.xml`` (see
:mod:`freecad.cross.dependencies`). The dialog shows every declared package
with its status:

* missing package: red ``not installed`` mention;
* while installing: live progress (percentage parsed from pip output) and a
  small text spinner next to the package name;
* installed package: green ``installed`` mention.

The pip installs run in a background thread so the dialog stays interactive
and can be closed at any time.
"""

from __future__ import annotations

import re
from typing import Optional

import FreeCADGui as fcgui

try:
    from PySide import QtCore, QtGui, QtWidgets
except ImportError:
    from PySide6 import QtCore, QtGui, QtWidgets

from ..dependencies import Dependency, get_dependencies, is_installed, missing_dependencies

try:
    from ..gui_utils import tr
except Exception:  # pragma: no cover
    def tr(text: str) -> str:
        return text


# Colors.
_RED = '#c0392b'
_GREEN = '#27ae60'
_BLUE = '#2980b9'
_GRAY = '#7f8c8d'

_SPINNER_FRAMES = ['|', '/', '-', '\\']

_ANSI_RE = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]')
_PERCENT_RE = re.compile(r'(\d{1,3})\s*%')
_STEP_RE = re.compile(
    r'(Collecting|Downloading|Using cached|Installing|Building wheel|'
    r'Successfully installed|Preparing metadata|Requirement already satisfied)',
    re.IGNORECASE,
)


def _clean_pip_text(text: str) -> str:
    """Strip ANSI escapes and return the last non-empty logical segment."""
    text = _ANSI_RE.sub('', text)
    parts = [p.strip() for p in re.split(r'[\r\n]+', text) if p.strip()]
    return parts[-1] if parts else ''


def _parse_percent(text: str) -> Optional[int]:
    matches = _PERCENT_RE.findall(text)
    if not matches:
        return None
    try:
        value = int(matches[-1])
    except ValueError:
        return None
    return max(0, min(100, value))


# Keep strong references to modeless dialogs so they are not garbage-collected.
_OPEN_DIALOGS: list['DependenciesDialog'] = []


class _InstallWorker(QtCore.QThread):
    """Install the missing packages on a background thread."""

    package_started = QtCore.Signal(int)
    package_output = QtCore.Signal(int, str)
    package_done = QtCore.Signal(int, bool)
    all_done = QtCore.Signal()

    def __init__(self, deps: list[tuple[int, Dependency]], parent=None):
        super().__init__(parent)
        self._deps = deps
        self._cancel = False

    def cancel(self) -> None:
        self._cancel = True

    def run(self) -> None:  # noqa: D102
        from ..packages import pip_install

        for row, dep in self._deps:
            if self._cancel:
                break
            self.package_started.emit(row)

            def _on_output(text: str, row: int = row) -> None:
                self.package_output.emit(row, text)

            ok = False
            try:
                rc = pip_install(dep.pip_name, on_output=_on_output)
                ok = rc == 0
            except Exception:
                ok = False

            # Re-check by import name (pip name may differ).
            if ok:
                import importlib
                try:
                    importlib.invalidate_caches()
                except Exception:
                    pass
                ok = is_installed(dep)

            self.package_done.emit(row, ok)

        self.all_done.emit()


class DependenciesDialog(QtWidgets.QDialog):
    """Non-modal dialog listing dependencies and installing the missing ones."""

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.setWindowTitle(tr('RobotCAD dependencies'))
        self.setModal(False)
        self.resize(620, 420)

        self._dependencies: list[Dependency] = get_dependencies()
        self._row_of: dict[int, int] = {}  # dependency index -> table row
        self._state: dict[int, str] = {}  # dependency index -> state
        self._percent: dict[int, Optional[int]] = {}
        self._detail: dict[int, str] = {}
        self._worker: Optional[_InstallWorker] = None
        self._spin_index = 0
        self._remove_on_finish = False
        # pip names installed during this dialog session (triggers the
        # re-enabling of tools that were disabled at workbench start-up).
        self._installed_this_session: set[str] = set()

        self._setup_ui()
        self._populate()

        self._spin_timer = QtCore.QTimer(self)
        self._spin_timer.setInterval(120)
        self._spin_timer.timeout.connect(self._tick_spinner)

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _setup_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(12, 12, 12, 12)

        self.header_label = QtWidgets.QLabel(
            tr('Checking the packages declared in package.xml ...'),
        )
        self.header_label.setWordWrap(True)
        layout.addWidget(self.header_label)

        self.table = QtWidgets.QTableWidget(0, 2, self)
        self.table.setHorizontalHeaderLabels([tr('Package'), tr('Status')])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.setFocusPolicy(QtCore.Qt.NoFocus)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        layout.addWidget(self.table)

        self.detail_label = QtWidgets.QLabel('')
        self.detail_label.setWordWrap(True)
        self.detail_label.setStyleSheet(f'color: {_GRAY};')
        layout.addWidget(self.detail_label)

        button_row = QtWidgets.QHBoxLayout()
        button_row.addStretch(1)

        self.install_button = QtWidgets.QPushButton(tr('Install missing'))
        self.install_button.clicked.connect(self.start_installation)
        button_row.addWidget(self.install_button)

        self.close_button = QtWidgets.QPushButton(tr('Close'))
        self.close_button.clicked.connect(self.close)
        button_row.addWidget(self.close_button)

        layout.addLayout(button_row)

    def _populate(self) -> None:
        self.table.setRowCount(len(self._dependencies))
        for index, dep in enumerate(self._dependencies):
            self._row_of[index] = index
            # Primary name is the package (pip) name; show the import name
            # when it differs.
            name_item = QtWidgets.QTableWidgetItem(dep.pip_name)
            pip_suffix = ''
            if dep.pip_name != dep.import_name:
                pip_suffix = f'  (import: {dep.import_name})'
            if dep.optional:
                pip_suffix += '  [' + tr('optional') + ']'
            name_item.setText(dep.pip_name + pip_suffix)
            self.table.setItem(index, 0, name_item)
            self.table.setItem(index, 1, QtWidgets.QTableWidgetItem(''))
            self._set_state(index, 'installed' if is_installed(dep) else 'missing')
        self._refresh_summary()

    # ------------------------------------------------------------------
    # Status helpers
    # ------------------------------------------------------------------

    def _set_item_color(self, row: int, column: int, color: str) -> None:
        item = self.table.item(row, column)
        if item is not None:
            item.setForeground(QtGui.QBrush(QtGui.QColor(color)))

    def _set_state(self, index: int, state: str) -> None:
        self._state[index] = state
        self._render(index)

    def _render(self, index: int) -> None:
        row = self._row_of.get(index)
        if row is None:
            return
        state = self._state.get(index, 'missing')
        status_item = self.table.item(row, 1)
        if status_item is None:
            return

        # Only the status text is colored; the package name keeps the default
        # (theme) color.
        if state == 'installed':
            status_item.setText(tr('installed'))
            self._set_item_color(row, 1, _GREEN)
        elif state == 'missing':
            status_item.setText(tr('not installed'))
            self._set_item_color(row, 1, _RED)
        elif state == 'installing':
            spinner = _SPINNER_FRAMES[self._spin_index % len(_SPINNER_FRAMES)]
            percent = self._percent.get(index)
            if percent is not None:
                status_item.setText(f'{spinner} {percent}%')
            else:
                status_item.setText(f'{spinner} ...')
            self._set_item_color(row, 1, _BLUE)
        elif state == 'failed':
            status_item.setText(tr('failed'))
            self._set_item_color(row, 1, _RED)
        elif state == 'skipped':
            status_item.setText(tr('skipped (optional)'))
            self._set_item_color(row, 1, _GRAY)

    def _refresh_summary(self) -> None:
        total = len(self._dependencies)
        installed = sum(1 for i in self._state if self._state[i] == 'installed')
        missing = sum(1 for i in self._state if self._state[i] in ('missing', 'failed'))
        self.header_label.setText(
            tr('Dependencies: {}/{} installed, {} missing.').format(installed, total, missing),
        )
        if not self._is_installing():
            self.install_button.setEnabled(missing > 0)

    def _tick_spinner(self) -> None:
        self._spin_index += 1
        for index, state in self._state.items():
            if state == 'installing':
                self._render(index)

    def _is_installing(self) -> bool:
        return self._worker is not None and self._worker.isRunning()

    # ------------------------------------------------------------------
    # Installation
    # ------------------------------------------------------------------

    def start_installation(self) -> None:
        """Start installing the missing dependencies (if any)."""
        if self._is_installing():
            return

        # Re-check in case the environment changed since the dialog opened.
        for index, dep in enumerate(self._dependencies):
            if is_installed(dep):
                self._state[index] = 'installed'
            else:
                self._state[index] = 'missing'
            self._render(index)

        pending: list[tuple[int, Dependency]] = [
            (index, self._dependencies[index])
            for index in range(len(self._dependencies))
            if self._state[index] == 'missing'
        ]

        if not pending:
            self.detail_label.setText(tr('Everything is already installed.'))
            self._refresh_summary()
            return

        self._percent.clear()
        self._detail.clear()
        for index, _ in pending:
            self._state[index] = 'installing'
            self._render(index)

        self.install_button.setEnabled(False)
        self._spin_timer.start()
        self.detail_label.setText(tr('Installing {} package(s) ...').format(len(pending)))

        worker = _InstallWorker(pending, self)
        worker.package_started.connect(self._on_package_started)
        worker.package_output.connect(self._on_package_output)
        worker.package_done.connect(self._on_package_done)
        worker.all_done.connect(self._on_all_done)
        self._worker = worker
        worker.start()

    @QtCore.Slot(int)
    def _on_package_started(self, row: int) -> None:
        self._state[row] = 'installing'
        self._render(row)
        self._refresh_summary()

    @QtCore.Slot(int, str)
    def _on_package_output(self, row: int, text: str) -> None:
        clean = _clean_pip_text(text)
        if not clean:
            return
        percent = _parse_percent(text)
        if percent is not None:
            self._percent[row] = percent
        # Keep a meaningful step line for the detail label.
        if _STEP_RE.search(clean):
            self._detail[row] = clean
        self._render(row)
        detail = self._detail.get(row, '')
        if detail:
            name = self._dependencies[row].pip_name
            self.detail_label.setText(f'{name}: {detail[:160]}')

    @QtCore.Slot(int, bool)
    def _on_package_done(self, row: int, ok: bool) -> None:
        self._state[row] = 'installed' if ok else 'failed'
        if ok:
            self._installed_this_session.add(self._dependencies[row].pip_name)
        self._percent.pop(row, None)
        self._render(row)
        self._refresh_summary()

    @QtCore.Slot()
    def _on_all_done(self) -> None:
        self._spin_timer.stop()
        self._worker = None
        # Final re-check for accurate statuses.
        for index, dep in enumerate(self._dependencies):
            if self._state.get(index) in ('installed', 'installing'):
                self._state[index] = 'installed' if is_installed(dep) else 'failed'
            self._render(index)
        self._refresh_summary()

        missing = [
            self._dependencies[i].pip_name
            for i in self._state
            if self._state[i] in ('missing', 'failed')
        ]
        if missing:
            self.detail_label.setText(
                tr('Still missing: {}').format(', '.join(missing)),
            )
        else:
            self.detail_label.setText(tr('All dependencies are installed.'))

        # Dependencies that just became available may enable tools that were
        # disabled at workbench start-up: re-run their modules and rebuild the
        # toolbar/menu so they appear without a FreeCAD restart.
        if self._installed_this_session:
            try:
                from ..init_gui import reactivate_commands_and_workspace
                reactivate_commands_and_workspace()
            except Exception:
                pass

        # If the dialog was closed while installing, release it now.
        if self._remove_on_finish and not self.isVisible():
            try:
                _OPEN_DIALOGS.remove(self)
            except ValueError:
                pass

    # ------------------------------------------------------------------
    # Lifetime
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:  # noqa: N802
        if self._spin_timer.isActive():
            self._spin_timer.stop()
        # Do not kill a running pip process; keep the dialog alive (and
        # referenced) until the worker finishes, then release it.
        if self._worker is not None and self._worker.isRunning():
            self._remove_on_finish = True
        else:
            try:
                _OPEN_DIALOGS.remove(self)
            except ValueError:
                pass
        super().closeEvent(event)


def open_dependencies_dialog(
    parent: Optional[QtWidgets.QWidget] = None,
    auto_install: bool = True,
) -> DependenciesDialog:
    """Create, show and return the dependencies dialog.

    When ``auto_install`` is True, the installation of the missing packages
    starts immediately (the dialog stays interactive and closable). If a
    dialog is already open, it is raised instead of creating a duplicate.
    """
    # Reuse an already-open dialog.
    for existing in list(_OPEN_DIALOGS):
        if existing.isVisible():
            existing.raise_()
            existing.activateWindow()
            if auto_install:
                existing.start_installation()
            return existing

    if parent is None:
        try:
            parent = fcgui.getMainWindow()
        except Exception:
            parent = None

    dialog = DependenciesDialog(parent)
    dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose, False)
    _OPEN_DIALOGS.append(dialog)
    dialog.show()
    dialog.raise_()
    dialog.activateWindow()

    if auto_install:
        # Defer slightly so the dialog is painted before pip work starts.
        QtCore.QTimer.singleShot(0, dialog.start_installation)

    return dialog


def show_dependencies_if_missing(parent: Optional[QtWidgets.QWidget] = None) -> bool:
    """Show the install dialog when a required dependency is missing.

    Only non-optional dependencies trigger the automatic dialog (optional ones,
    such as ``mcp``, are installed lazily and can still be installed from the
    dialog opened by the "Check and install dependencies" button).

    The dialog is only *shown* here: the installation does not start until the
    user clicks the "Install missing" button.

    Returns True when the dialog was shown.
    """
    if not missing_dependencies(include_optional=False):
        return False
    open_dependencies_dialog(parent=parent, auto_install=False)
    return True
