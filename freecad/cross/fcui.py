# -*- coding: utf-8 -*-
#
#  This library is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 2.1 of the License, or (at your option) any later version.
#
#  This library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this library; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
#
#  (c) 2024 Frank David Martínez Muñoz.
#

from __future__ import annotations

__author__ = "Frank David Martínez Muñoz"
__copyright__ = "(c) 2024 Frank David Martínez Muñoz."
__license__ = "LGPL 2.1"
__version__ = "1.0.0-beta4"
__min_python__ = "3.10"
__min_freecad__ = "0.22"

# Conventions for sections in this file:
# See: vscode extension: aaron-bond.better-comments
# ──────────────────────────────────────────────────────────────────────────────
#   Normal comments
##: Code execute at import time, create objects in global module scope
##$ Template code, meta-programming
##@ Decorators code
##% Type definitions, Widgets, Builders
##! Warning note
# ──────────────────────────────────────────────────────────────────────────────

##: [SECTION] Builtin Imports
##: ────────────────────────────────────────────────────────────────────────────

import json
import re
import sys
import threading
from contextlib import contextmanager
from functools import wraps
from pathlib import Path
from typing import Any, Callable, Dict, Generator, List, Optional, Set, Tuple, Union, Hashable

##: [SECTION] FreeCAD Imports
##: ────────────────────────────────────────────────────────────────────────────

import FreeCAD as App  # type: ignore
import FreeCADGui as Gui  # type: ignore
from FreeCAD import Base  # type: ignore

##: [SECTION] Qt/PySide Imports
##: ────────────────────────────────────────────────────────────────────────────

from PySide.QtCore import (  # type: ignore
    QMargins,
    QObject,
    QPoint,
    QRect,
    Qt,
    QTimer,
    Signal,
    Slot,
)

from PySide.QtGui import (  # type: ignore
    QAbstractItemView,
    QApplication,
    QBrush,
    QCheckBox,
    QCloseEvent,
    QColor,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFileDialog,
    QFontDatabase,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QIcon,
    QLabel,
    QLayout,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPainter,
    QPaintEvent,
    QPen,
    QPixmap,
    QPlainTextEdit,
    QPushButton,
    QAbstractButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QToolButton,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from PySide.QtSvg import QSvgRenderer  # type: ignore


##: [SECTION] Type Aliases
##: ────────────────────────────────────────────────────────────────────────────

Numeric = Union[int, float]
Vector = Base.Vector

##: [SECTION] Core Widgets, Contexts and Decorators
##: ────────────────────────────────────────────────────────────────────────────


# ──────────────────────────────────────────────────────────────────────────────
def set_qt_attrs(qobject: QObject, **kwargs):
    """
    Call setters on Qt objects by argument names.

    :param QObject qobject: Object instance.
    :param dict[str, Any] kwargs: dict of property_name to value.
    """
    for name, value in kwargs.items():
        if value is not None:
            if name == "properties":
                for prop_name, prop_value in value.items():
                    qobject.setProperty(prop_name, prop_value)
                continue
            setter = getattr(qobject, f"set{name[0].upper()}{name[1:]}", None)
            if setter:
                if isinstance(value, tuple):
                    setter(*value)
                else:
                    setter(value)
            else:
                raise NameError(f"Invalid property {name}")


# ──────────────────────────────────────────────────────────────────────────────
def setup_layout(layout: QLayout, add: bool = True, **kwargs):
    """
    Setup layouts adding wrapper widget if required.
    """
    set_qt_attrs(layout, **kwargs)
    parent = build_context().current()
    if parent.layout() is not None or add is False:
        w = QWidget()
        w.setLayout(layout)
        if add:
            parent.layout().addWidget(w)
        with build_context().stack(w):
            yield w
    else:
        parent.setLayout(layout)
        yield parent


# ──────────────────────────────────────────────────────────────────────────────
def place_widget(
    widget: QWidget,
    label: Union[QWidget, str] = None,
    stretch: int = 0,
    alignment=Qt.Alignment(),
) -> None:
    """
    Place widget in layout.
    """
    current = build_context().current()
    if isinstance(current, QScrollArea):
        if current.widget():
            raise ValueError("Scroll can contains only one widget")
        current.setWidget(widget)
        return

    if isinstance(current, QSplitter):
        current.addWidget(widget)
        return

    if isinstance(current, QMainWindow):
        current.setCentralWidget(widget)
        return

    layout = current.layout()
    if layout is None:
        layout = build_context().default_layout_provider()
        current.setLayout(layout)

    if label is None:
        layout.addWidget(widget, stretch, alignment)
    else:
        layout.addWidget(widget_with_label_row(widget, label, stretch, alignment))


##% [Widget] QWidget with label and widget in Vertical or Horizontal layout
##% ────────────────────────────────────────────────────────────────────────────
def widget_with_label_row(
    widget: QWidget,
    label: Union[QWidget, str],
    stretch: int = 0,
    alignment=Qt.Alignment(),
    orientation: Qt.Orientation = Qt.Orientation.Horizontal,
) -> QWidget:
    """
    Create a widget with a label and widget.
    """
    row = QWidget()
    if orientation == Qt.Orientation.Vertical:
        layout = QVBoxLayout()
    else:
        layout = QHBoxLayout()
    row.setLayout(layout)
    layout.setContentsMargins(0, 0, 0, 0)
    if isinstance(label, QWidget):
        layout.addWidget(label)
    elif label:
        layout.addWidget(QLabel(str(label)))
    layout.addWidget(widget, stretch, alignment)
    return row


##% Color
##% ────────────────────────────────────────────────────────────────────────────
class Color(QColor):
    """
    QColor with additional constructor for hex rgba color code.
    Use like Color(code='#ff0000'), Color(code='#ff0000ff')
    """

    def __init__(self, *args, code: str = None, alpha: float = None, **kwargs):
        if code is not None:
            if code.startswith("#"):
                code = code[1:]
            if len(code) < 8:
                code += "FFFFFFFF"
            r, g, b, a = (
                int(code[:2], 16),
                int(code[2:4], 16),
                int(code[4:6], 16),
                int(code[6:8], 16),
            )
            super().__init__(r, g, b, a)
        elif len(args) == 1 and isinstance(args[0], QColor):
            super().__init__()
            self.setRgba(args[0].rgba())
        else:
            super().__init__(*args, **kwargs)
        if isinstance(alpha, float):
            self.setAlphaF(alpha)

    def __str__(self) -> str:
        return f"rgba({self.red()},{self.green()},{self.blue()},{self.alpha()})"


##% [Widget] ColorIcon
##% ────────────────────────────────────────────────────────────────────────────
class ColorIcon(QIcon):
    """
    Monochromatic Icon with transparent background.
    """

    def __init__(self, path, color):
        pixmap = QPixmap(path)
        mask = pixmap.createMaskFromColor(QColor("transparent"), Qt.MaskInColor)
        pixmap.fill(color)
        pixmap.setMask(mask)
        super().__init__(pixmap)
        self.setIsMask(True)


##% PySignal
##% ───────────────────────────────────────────────────────────────────────────
class PySignal:
    """
    Imitate Qt Signals for non QObject objects
    """

    _listeners: Set[Callable]

    def __init__(self):
        self._listeners = set()

    def connect(self, listener: Callable):
        self._listeners.add(listener)

    def disconnect(self, listener: Callable):
        try:
            self._listeners.remove(listener)
        except KeyError:
            pass  # Not found, Ok

    def emit(self, *args, **kwargs):
        for listener in self._listeners:
            listener(*args, **kwargs)


##@ [Decorator] on_event
##@ ────────────────────────────────────────────────────────────────────────────
def on_event(target, event=None):
    """
    Event binder decorator. Connects the decorated function to `event` signal
    on all targets.

    :param QObject | Signal | list[QObject|Signal] target: target object or objects.
    :param str event: name of the signal.
    """
    if not target:
        raise ValueError("Invalid empty target")

    if not isinstance(target, (list, tuple, set)):
        target = [target]

    if event is None:

        def deco(fn):
            for t in target:
                t.connect(fn)
            return fn

    else:

        def deco(fn):
            for t in target:
                getattr(t, event).connect(fn)
            return fn

    return deco


##% SelectedObject
##% ────────────────────────────────────────────────────────────────────────────
class SelectedObject:
    """
    Store Selection information of a single object+sub
    """

    def __init__(self, doc: str, obj: str, sub: str = None, pnt: Vector = None):
        self.doc = doc
        self.obj = obj
        self.sub = sub
        self.pnt = pnt

    def __iter__(self):
        yield App.getDocument(self.doc).getObject(self.obj)
        yield self.sub
        yield self.pnt

    def __repr__(self) -> str:
        return f"{self.doc}#{self.obj}.{self.sub}"

    def __hash__(self) -> int:
        return hash((self.doc, self.obj, self.sub))

    def __eq__(self, __o: object) -> bool:
        return hash(self) == hash(__o)

    def __ne__(self, __o: object) -> bool:
        return not self.__eq__(__o)

    def resolve_object(self):
        return App.getDocument(self.doc).getObject(self.obj)

    def resolve_sub(self):
        return getattr(self.resolve_object(), self.sub)


# ──────────────────────────────────────────────────────────────────────────────
def register_select_observer(owner: QWidget, observer):
    """Add observer with auto remove on owner destroyed"""
    Gui.Selection.addObserver(observer)

    def destroyed(*_):
        Gui.Selection.removeObserver(observer)

    owner.destroyed.connect(destroyed)


# [Context] selection
# ──────────────────────────────────────────────────────────────────────────────
@contextmanager
def selection(*names, clean: bool = True, doc: App.Document = None):
    """
    Add objects identified by names into current selection.

    :param bool clean: remove selection at the end, defaults to True
    :param App.Document doc: Document, defaults to App.ActiveDocument
    :yield list[DocumentObject]: list of selected objects.
    """
    sel = Gui.Selection
    try:
        doc_name = (doc or App.ActiveDocument).Name
        if len(names) == 0:
            yield sel.getSelection(doc_name)
        else:
            sel.clearSelection()
            for name in names:
                if isinstance(name, (tuple, list)):
                    sel.addSelection(doc_name, *name)
                elif isinstance(name, SelectedObject):
                    sel.addSelection(name.doc, name.obj, name.sub)
                else:
                    sel.addSelection(doc_name, name)
            yield sel.getSelection(doc_name)
    finally:
        if clean:
            sel.clearSelection()


##% BuildContext class
##% ────────────────────────────────────────────────────────────────────────────
class _BuildContext:
    """
    Qt Widget tree build context and stack
    """

    def __init__(self):
        self._stack = []
        self.default_layout_provider = QVBoxLayout

    def push(self, widget):
        self._stack.append(widget)

    def pop(self):
        self._stack.pop()

    def reset(self):
        last = self._stack[-1] if self._stack else None
        self._stack = []
        return last

    @contextmanager
    def stack(self, widget):
        self.push(widget)
        try:
            yield widget
        finally:
            self.pop()

    @contextmanager
    def parent(self):
        if len(self._stack) > 1:
            current = self._stack[-1]
            self._stack.pop()
            parent = self._stack[-1]
            try:
                yield parent
            finally:
                self._stack.append(current)

    def current(self):
        return self._stack[-1]

    def dump(self):
        print(f"BuildContext: {self._stack}")


# ──────────────────────────────────────────────────────────────────────────────
def build_context() -> _BuildContext:
    """
    Build context for the current thread.
    """
    bc = getattr(_thread_local_gui_vars, "BuildContext", None)
    if bc is None:
        _thread_local_gui_vars.BuildContext = _BuildContext()
        return _thread_local_gui_vars.BuildContext
    else:
        return bc


##% [Context] Parent
##% ────────────────────────────────────────────────────────────────────────────
@contextmanager
def Parent():
    """Put parent in context"""
    with build_context().parent() as p:
        yield p


##% Dialogs
##% ────────────────────────────────────────────────────────────────────────────
class Dialogs:
    """
    Keeps a list of Active dialogs
    """

    _list = []

    @classmethod
    def dump(cls):
        print(f"Dialogs: {cls._list}")

    @classmethod
    def register(cls, dialog):
        cls._list.append(dialog)
        dialog.closeEvent = lambda e: cls.destroy_dialog(dialog)

    @classmethod
    def destroy_dialog(cls, dlg):
        cls._list.remove(dlg)
        dlg.deleteLater()

    @classmethod
    def open(cls, w, modal: bool = True):
        Dialogs.register(w)
        if modal:
            w.open()
        else:
            w.show()
            try:
                w.raise_()  # Mac ?? Wayland ??
            except Exception as ex:
                print_err(str(ex))
            if hasattr(w, "requestActivate"):
                w.requestActivate()


##% [Widget Impl] DialogWidget
##% ────────────────────────────────────────────────────────────────────────────
class DialogWidget(QDialog):
    """
    Simple Dialog with onClose as signal.
    """

    onClose = Signal(QCloseEvent)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def closeEvent(self, event: QCloseEvent):
        self.onClose.emit(event)
        super().closeEvent(event)


##% [Widget] Dialog
##% ────────────────────────────────────────────────────────────────────────────
@contextmanager
def Dialog(
    title: str = None,
    *,
    size: Tuple[int, int] = None,
    show: bool = True,
    modal: bool = True,
    parent: QWidget = None,
    **kwargs,
) -> Generator[QDialog, Any, Any]:
    """
    Dialog context manager/widget.

    Example:
    ~:code:../examples/ui/widgets.py[Dialog]:~

    :param str title: window title, defaults to None
    :param Tuple[int, int] size: dialog size, defaults to None
    :param bool show: show automatically, defaults to True
    :param bool modal: window modality, defaults to True
    :param QWidget parent: parent widget, defaults to None
    :param Dict[str, Any] **kwargs: Qt properties
    :return QDialog: The Dialog
    """
    if parent is None:
        parent = find_active_window()

    w = DialogWidget(parent=parent)

    if title is not None:
        w.setWindowTitle(title)
    set_qt_attrs(w, **kwargs)

    build_context().reset()
    with build_context().stack(w):
        yield w
        if isinstance(size, (tuple, list)):
            w.resize(size[0], size[1])
        else:
            w.adjustSize()
        if show:
            Dialogs.open(w, modal)


##% [Widget] Scroll
##% ────────────────────────────────────────────────────────────────────────────
@contextmanager
def Scroll(*, add: bool = True, **kwargs) -> Generator[QScrollArea, Any, Any]:
    """
    Scrollable area context manager/widget.

    Example:
    ~:code:../examples/ui/widgets.py[Scroll]:~

    :param bool add: add to context, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties
    :return QScrollArea: Scroll widget
    """
    w = QScrollArea()
    set_qt_attrs(w, **kwargs)
    if add:
        place_widget(w)
    with build_context().stack(w):
        yield w


##% [Widget] GroupBox
##% ────────────────────────────────────────────────────────────────────────────
@contextmanager
def GroupBox(
    title: str = None,
    *,
    add: bool = True,
    **kwargs,
) -> Generator[QGroupBox, Any, Any]:
    """
    GroupBox context manager/widget.

    Example:
    ~:code:../examples/ui/widgets.py[GroupBox]:~

    :param str title: Group title, defaults to None
    :param bool add: add to context, defaults to True
    :param bool add: add to context, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties
    :return QGroupBox: The group widget
    """
    w = QGroupBox()
    if title:
        w.setTitle(title)
    set_qt_attrs(w, **kwargs)
    if add:
        place_widget(w)
    with build_context().stack(w):
        yield w


##% [Widget] Container
##% ────────────────────────────────────────────────────────────────────────────
@contextmanager
def Container(*, add: bool = True, **kwargs) -> Generator[QFrame, Any, Any]:
    """
    Simple container context/widget.

    Example
    ~:code:../examples/ui/widgets.py[Scroll]:~

    :param bool add: add to context, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties
    :return QFrame: The container widget
    """
    w = QWidget()
    set_qt_attrs(w, **kwargs)
    if add:
        place_widget(w)
    with build_context().stack(w):
        yield w


##% [Layout tool] Stretch
##% ────────────────────────────────────────────────────────────────────────────
def Stretch(stretch: int = 0) -> None:
    """
    Adds stretch factor to the current layout

    :param int stretch: 0-100 stretch factor, defaults to 0
    """
    layout = build_context().current().layout()
    if layout:
        layout.addStretch(stretch)


##% [Layout tool] Spacing
##% ────────────────────────────────────────────────────────────────────────────
def Spacing(size: int) -> None:
    """
    Adds spacing ro the current layout.

    :param int size: spacing
    """
    layout = build_context().current().layout()
    if layout:
        layout.addSpacing(size)


##% [Widget] TabContainer
##% ────────────────────────────────────────────────────────────────────────────
@contextmanager
def TabContainer(
    *,
    stretch: int = 0,
    add: bool = True,
    **kwargs,
) -> Generator[QTabWidget, Any, Any]:
    """
    Tab Container context/widget

    Example:
    ~:code:../examples/ui/widgets.py[TabContainer]:~

    :param int stretch: stretch, defaults to 0
    :param bool add: add to the context, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties
    :return QTabWidget: The widget
    """
    w = QTabWidget()
    set_qt_attrs(w, **kwargs)
    if add:
        place_widget(w, stretch=stretch)
    with build_context().stack(w):
        yield w


##% [Widget] Tab
##% ────────────────────────────────────────────────────────────────────────────
@contextmanager
def Tab(
    title: str,
    *,
    icon: QIcon = None,
    add: bool = True,
    **kwargs,
) -> Generator[QWidget, Any, Any]:
    """
    Tab widget/context in a tab container

    Example:
    ~:code:../examples/ui/widgets.py[TabContainer]:~

    :param str title: Tab's title
    :param QIcon icon: Icon, defaults to None
    :param bool add: add to the context, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties
    :return QWidget: the widget
    """
    w = QWidget()
    set_qt_attrs(w, **kwargs)
    with build_context().stack(w):
        yield w
    if add:
        if icon:
            build_context().current().addTab(w, icon, title)
        else:
            build_context().current().addTab(w, title)


##% [Widget] Splitter
##% ────────────────────────────────────────────────────────────────────────────
@contextmanager
def Splitter(*, add=True, **kwargs) -> Generator[QSplitter, Any, Any]:
    """
    Split context/container

    Example:
    ~:code:../examples/ui/widgets.py[Splitter]:~

    :param bool add: add to current context, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties
    :return QSplitter: The splitter widget
    """
    w = QSplitter()
    set_qt_attrs(w, **kwargs)
    if add:
        place_widget(w)
    with build_context().stack(w):
        yield w


##% [Layout] Col (Vertical Box)
##% ────────────────────────────────────────────────────────────────────────────
@contextmanager
def Col(*, add: bool = True, **kwargs) -> Generator[QWidget, Any, Any]:
    """
    Vertical context/layout

    Example:
    ~:code:../examples/ui/widgets.py[Col]:~

    :param bool add: add to current context, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties
    :return QWidget: A container widget with Vertical layout
    """
    yield from setup_layout(QVBoxLayout(), add=add, **kwargs)


##% [Layout] Row (Horizontal Box)
##% ────────────────────────────────────────────────────────────────────────────
@contextmanager
def Row(*, add: bool = True, **kwargs):
    """
    Horizontal context/layout

    Example:
    ~:code:../examples/ui/widgets.py[Row]:~

    :param bool add: add to current context, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties
    :return QWidget: A container widget with Horizontal layout
    """
    yield from setup_layout(QHBoxLayout(), add=add, **kwargs)


##% [Widget Impl] HtmlWidget
##% ────────────────────────────────────────────────────────────────────────────
class HtmlWidget(QLabel):
    """
    Html template widget.
    """

    VAR_RE = re.compile(r"\{\{(.*?)\}\}")  # template var: {{name}}
    base_path: Path
    css: str

    def __init__(self, base_path: Path, css: str):
        super().__init__()
        self.css = css or ""
        self.base_path = base_path

    def interpolator(self, variables: Dict[str, Any]):
        if variables:

            def replacer(match: re.Match) -> str:
                var_name = match.group(1)
                if var_name == "__base__":
                    return str(self.base_path)
                return str(variables.get(var_name, ""))

        else:

            def replacer(match: re.Match) -> str:
                if match.group(1) == "__base__":
                    return str(self.base_path)
                else:
                    return ""

        return replacer

    def setValue(self, html: str, variables: Dict[str, Any] = None):
        content = HtmlWidget.VAR_RE.sub(self.interpolator(variables), html)
        self.setText(f"<style>{self.css}</style>{content}")


##% [Widget] Html
##% ────────────────────────────────────────────────────────────────────────────
def Html(
    *,
    html: str = None,
    file: str = None,
    css: str = None,
    css_file: str = None,
    base_path: str = None,
    background: str = None,
    stretch: int = 0,
    alignment: Qt.Alignment = Qt.Alignment(),
    variables: Dict[str, Any] = None,
    add: bool = True,
    **kwargs,
) -> HtmlWidget:
    """
    Basic HTML Render widget

    Example:
    ~:code:../examples/ui/widgets.py[Html]:~

    :param str html: raw html content, defaults to None
    :param str file: path to html file, defaults to None
    :param str css: raw css code, defaults to None
    :param str css_file: path to css file, defaults to None
    :param str base_path: base dir for loading resources, defaults to None
    :param str background: background color code, defaults to None
    :param int stretch: layout stretch, defaults to 0
    :param Qt.Alignment alignment: layout alignment, defaults to Qt.Alignment()
    :param Dict[str, Any] variables: interpolation variables, defaults to None
    :param bool add: add to current context, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties of QLabel
    :return HtmlWidget: Html Widget
    """
    if html and file:
        raise ValueError("html and file arguments are mutually exclusive")

    if base_path:
        base_path = Path(base_path)
        if file:
            file = Path(base_path, file)
        if css_file:
            css_file = Path(base_path, css_file)
    elif file:
        base_path = Path(file).parent
        if css_file:
            css_file = Path(base_path, css_file)

    if html is None:
        with open(file, "r") as f:
            html = f.read()

    base_css = ""
    if css_file:
        with open(css_file, "r") as f:
            base_css = f.read()

    if css is not None:
        base_css += css

    label = HtmlWidget(base_path, base_css)
    label.setValue(html, variables)

    if background is not None:
        label.setStyleSheet(f"background-color: {background};")

    set_qt_attrs(label, **kwargs)
    if add:
        place_widget(label, stretch=stretch, alignment=alignment)
    return label


##% [Widget] TextLabel
##% ────────────────────────────────────────────────────────────────────────────
def TextLabel(
    text: str = "",
    *,
    stretch: int = 0,
    alignment: Qt.Alignment = Qt.Alignment(),
    add: bool = True,
    **kwargs,
) -> QLabel:
    """
    Simple text label widget

    Example:
    ~:code:../examples/ui/widgets.py[TextLabel]:~

    :param str text: text, defaults to ""
    :param int stretch: layout stretch, defaults to 0
    :param Qt.Alignment alignment: layout alignment, defaults to Qt.Alignment()
    :param bool add: add to current context, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties of QLabel
    :return QLabel: The widget
    """
    label = QLabel(text)
    set_qt_attrs(label, **kwargs)
    if add:
        place_widget(label, stretch=stretch, alignment=alignment)
    return label


##% [Widget] InputFloat
##% ────────────────────────────────────────────────────────────────────────────
def InputFloat(
    value: float = 0.0,
    *,
    name: str = None,
    min: float = 0.0,
    max: float = sys.float_info.max,
    decimals: int = 6,
    step: float = 0.01,
    label: Union[QWidget, str] = None,
    stretch: int = 0,
    alignment: Qt.Alignment = Qt.Alignment(),
    add: bool = True,
    **kwargs,
) -> QDoubleSpinBox:
    """
    Input float widget

    Example:
    ~:code:../examples/ui/widgets.py[InputFloat]:~

    :param float value: initial value, defaults to 0.0
    :param str name: objectName, defaults to None
    :param float min: minimum accepted value, defaults to 0.0
    :param float max: maximum accepted value, defaults to sys.float_info.max
    :param int decimals: decimal digits, defaults to 6
    :param float step: spin steps, defaults to 0.01
    :param Union[QWidget, str] label: ui label, defaults to None
    :param int stretch: layout stretch, defaults to 0
    :param Qt.Alignment alignment: layout alignment, defaults to Qt.Alignment()
    :param bool add: add to current context, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties
    :return QDoubleSpinBox: The input widget
    """
    widget = QDoubleSpinBox()
    widget.setMinimum(min)
    widget.setMaximum(max)
    widget.setSingleStep(step)
    widget.setDecimals(decimals)
    widget.setValue(value)
    set_qt_attrs(widget, **kwargs)
    if name:
        widget.setObjectName(name)
    if add:
        place_widget(widget, label=label, stretch=stretch, alignment=alignment)
    return widget


##% [Widget Impl] InputTextWidget
##% ────────────────────────────────────────────────────────────────────────────
class InputTextWidget(QLineEdit):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def value(self):
        return self.text()

    def setValue(self, value):
        self.setText(str(value))


##% [Widget] InputText
##% ────────────────────────────────────────────────────────────────────────────
def InputText(
    value: str = "",
    *,
    name: str = None,
    label: Union[QWidget, str] = None,
    stretch: int = 0,
    alignment: Qt.Alignment = Qt.Alignment(),
    add: bool = True,
    **kwargs,
) -> InputTextWidget:
    """
    Input text widget

    Example:
    ~:code:../examples/ui/widgets.py[InputText]:~

    :param str value: initial value, defaults to ""
    :param str name: objectName, defaults to None
    :param Union[QWidget, str] label: gui label, defaults to None
    :param int stretch: layout stretch, defaults to 0
    :param Qt.Alignment alignment: layout alignment, defaults to Qt.Alignment()
    :param bool add: add to current context, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties
    :return InputTextWidget: The widget
    """
    widget = InputTextWidget()
    widget.setText(value)
    set_qt_attrs(widget, **kwargs)
    if name:
        widget.setObjectName(name)
    if add:
        place_widget(widget, label=label, stretch=stretch, alignment=alignment)
    return widget


##% [Widget Impl] InputQuantityWidget
##% ────────────────────────────────────────────────────────────────────────────
class InputQuantityWidget:
    def __init__(self, editor) -> None:
        self.editor = editor

    def value(self) -> Any:
        return self.editor.property("rawValue")

    def rawValue(self) -> Any:
        return self.editor.property("rawValue")

    def setValue(self, value: Any):
        return self.editor.setProperty("rawValue", value)

    def setMinimum(self, value: float):
        return self.editor.setProperty("minimum", value)

    def setMaximum(self, value: float):
        return self.editor.setProperty("maximum", value)

    def setSingleStep(self, value: float):
        return self.editor.setProperty("singleStep", value)

    def setUnit(self, value: str):
        return self.editor.setProperty("unit", value)


##% [Widget] InputQuantity
##% ────────────────────────────────────────────────────────────────────────────
def InputQuantity(
    value: Numeric = None,
    *,
    name: str = None,
    min: Numeric = None,
    max: Numeric = None,
    step: Numeric = 1.0,
    label: Union[QWidget, str] = None,
    stretch: int = 0,
    alignment: Qt.Alignment = Qt.Alignment(),
    unit: str = None,
    obj: object = None,
    property: str = None,
    add: bool = True,
    **kwargs,
) -> InputQuantityWidget:
    """
    Input Quantity Widget with unit and expressions support.

    Example:
    ~:code:../examples/ui/widgets.py[InputQuantity]:~

    :param Numeric value: Initial value, defaults to None
    :param str name: objectName (Qt), defaults to None
    :param Numeric min: Minimum accepted value, defaults to None
    :param Numeric max: Maximum accepted value, defaults to None
    :param Numeric step: Spin step, defaults to 1.0
    :param Union[QWidget, str] label: gui label, defaults to None
    :param int stretch: Layout stretch, defaults to 0
    :param Qt.Alignment alignment: Layout alignment, defaults to Qt.Alignment()
    :param str unit: quantity unit (i.e. mm, in, ...), defaults to None
    :param DocumentObject obj: Object to bind the expression engine, defaults to None
    :param str property: Property name of the bounded DocumentObject if any, defaults to None
    :param bool add: Add to current context, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties
    :return InputQuantityWidget: The widget
    """
    if obj and property:
        if property not in obj.PropertiesList:
            raise ValueError(f"Invalid property name: {property}")

    editor = _fc_ui_loader.createWidget("Gui::InputField")
    widget = InputQuantityWidget(editor)
    if min is not None:
        widget.setMinimum(min)
    if max is not None:
        widget.setMaximum(max)
    if step is not None:
        widget.setSingleStep(step)
    if unit is not None:
        widget.setUnit(unit)
    set_qt_attrs(editor, **kwargs)

    if name:
        editor.setObjectName(name)

    if obj and property:
        ee = Gui.ExpressionBinding(editor)
        ee.bind(obj, property)
        ee.setAutoApply(True)
        editor.valueChanged.connect(lambda v: setattr(obj, property, v))
    elif value is not None:
        if isinstance(value, str):
            editor.setText(value)
        elif isinstance(value, (float, int)):
            if isinstance(unit, str):
                editor.setText(f"{float(value)} {unit}")
            elif isinstance(unit, App.Units.Unit):
                editor.setText(App.Units.Quantity(value, unit).UserString)
        elif isinstance(value, App.Units.Quantity):
            editor.setText(value.UserString)
        else:
            widget.setValue(value)

    if add:
        place_widget(editor, label=label, stretch=stretch, alignment=alignment)

    return widget


##% [Widget] InputInt
##% ────────────────────────────────────────────────────────────────────────────
def InputInt(
    value: int = 0,
    *,
    name: str = None,
    min: int = 0,
    max: int = None,
    step: int = 1,
    label: Union[QWidget, str] = None,
    stretch: int = 0,
    alignment: Qt.Alignment = Qt.Alignment(),
    add: bool = True,
    **kwargs,
) -> QSpinBox:
    """
    Input int widget

    Example:
    ~:code:../examples/ui/widgets.py[InputInt]:~

    :param int value: initial value, defaults to 0
    :param str name: objectName, defaults to None
    :param int min: minimum accepted value, defaults to 0
    :param int max: maximum accepted value, defaults to None
    :param int step: spin steps, defaults to 1
    :param Union[QWidget, str] label: ui label, defaults to None
    :param int stretch: layout stretch, defaults to 0
    :param Qt.Alignment alignment: layout alignment, defaults to Qt.Alignment()
    :param bool add: add to current context, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties
    :return QSpinBox: The input widget
    """
    widget = QSpinBox()
    widget.setMinimum(min)
    if max is not None:
        widget.setMaximum(max)
    widget.setSingleStep(step)
    widget.setValue(value)
    set_qt_attrs(widget, **kwargs)
    if name:
        widget.setObjectName(name)
    if add:
        place_widget(widget, label=label, stretch=stretch, alignment=alignment)
    return widget


##% [Widget Impl] QCheckBoxExt
##% ────────────────────────────────────────────────────────────────────────────
class QCheckBoxExt(QCheckBox):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def value(self) -> bool:
        return self.checkState() == Qt.Checked

    def setValue(self, value: bool):
        self.setCheckState(Qt.Checked if value else Qt.Unchecked)


##% [Widget] InputBoolean
##% ────────────────────────────────────────────────────────────────────────────
def InputBoolean(
    value: bool = False,
    *,
    name: str = None,
    label: Union[QWidget, str] = None,
    stretch: int = 0,
    alignment: Qt.Alignment = Qt.Alignment(),
    add: bool = True,
    **kwargs,
) -> QCheckBoxExt:
    """
    Input boolean widget as QCheckBox

    Example:
    ~:code:../examples/ui/widgets.py[InputBoolean]:~

    :param bool value: initial value, defaults to False
    :param str name: objectName, defaults to None
    :param Union[QWidget, str] label: ui label, defaults to None
    :param int stretch: layout stretch, defaults to 0
    :param Qt.Alignment alignment: layout alignment, defaults to Qt.Alignment()
    :param bool add: add to the gui, defaults to True
    :param Dict[str, Any] **kwargs: settable QCheckBox properties
    :return QCheckBoxExt: The widget
    """
    widget = QCheckBoxExt()
    widget.setValue(value)
    set_qt_attrs(widget, **kwargs)
    if name:
        widget.setObjectName(name)
    if add:
        place_widget(widget, label=label, stretch=stretch, alignment=alignment)
    return widget


##% [Layout] LayoutWidget
##% ────────────────────────────────────────────────────────────────────────────
class LayoutWidget(QWidget):
    """
    Layout widget builder
    """

    def __init__(self, layout_builder: Callable[[], QLayout], **kwargs):
        super().__init__()
        layout = layout_builder()
        set_qt_attrs(layout, **kwargs)
        self.setLayout(layout)

    def addWidget(
        self,
        widget: QWidget,
        stretch: int = 0,
        alignment: Qt.Alignment = Qt.Alignment(),
    ):
        self.layout().addWidget(widget, stretch, alignment)

    def addStretch(self, stretch: int = 0):
        self.layout().addStretch(stretch)

    def addSpacing(self, size: int):
        self.layout().addSpacing(size)


##% [Widget Impl] InputFloatListWidget
##% ────────────────────────────────────────────────────────────────────────────
class InputFloatListWidget(QWidget):
    valueChanged = Signal()

    def __init__(
        self,
        count: int = 0,
        values: List[float] = None,
        label_fn: Callable[[int], str] = None,
        resizable: bool = False,
        min_count: int = 0,
        **kwargs,
    ):
        super().__init__()
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)
        self.options = kwargs
        self.min_count = min_count

        if not values:
            values = [0.0] * count
        else:
            count = len(values)

        if count < min_count:
            raise ValueError(f"Minimum rows required are {min_count}")

        if not label_fn:
            label_fn = str

        self._label_fn = label_fn

        self.inputs = []
        labels = tuple(QLabel(label_fn(i)) for i in range(count))
        for i in range(count):
            ctrl = InputFloat(value=values[i], add=False, **kwargs)
            ctrl.valueChanged.connect(self.valueChanged)
            self.inputs.append(ctrl)

        panel = LayoutWidget(QVBoxLayout, contentsMargins=(0, 0, 0, 0))
        layout.addWidget(panel)
        for i in range(count):
            panel.addWidget(widget_with_label_row(self.inputs[i], labels[i]))
        self.panel = panel

        if resizable:
            self.resize_controls()

    def resize_controls(self):
        buttons = LayoutWidget(QHBoxLayout, contentsMargins=(0, 0, 0, 0))
        self.layout().addWidget(buttons, 0, alignment=Qt.AlignRight)

        @button(label="+", add=False, tool=True)
        def add():
            self.addValue(**self.options)

        @button(label="−", add=False, tool=True)
        def remove():
            self.removeLast()

        buttons.addWidget(add, alignment=Qt.AlignRight)
        buttons.addWidget(remove, alignment=Qt.AlignRight)

    def value(self) -> Tuple[float, ...]:
        return tuple(i.value() for i in self.inputs)

    def setValue(self, value):
        while len(value) > len(self.inputs):
            self.addValue()
        while len(value) < len(self.inputs):
            self.removeLast()
        for i, input in enumerate(self.inputs):
            input.setValue(value[i])

    def addValue(self, **kwargs):
        input = InputFloat(add=False, **kwargs)
        input.valueChanged.connect(self.valueChanged)
        i = len(self.inputs)
        self.inputs.append(input)
        self.panel.addWidget(widget_with_label_row(input, self._label_fn(i)))

    def removeLast(self):
        if len(self.inputs) > self.min_count:
            item = self.inputs.pop()
            item.parent().setParent(None)


##% [Widget] InputFloatList
##% ────────────────────────────────────────────────────────────────────────────
def InputFloatList(
    values: List[float] = None,
    label: Union[QWidget, str] = None,
    name: str = None,
    label_fn: Callable[[int], str] = None,
    count: int = 0,
    resizable: bool = False,
    min_count: int = 0,
    add: bool = True,
    **kwargs,
):
    """
    Widget to accept lists of float numbers.

    Example:
    ~:code:../examples/ui/widgets.py[InputFloatList]:~

    :param List[float] values: initial values, defaults to None
    :param Union[QWidget, str] label: ui label, defaults to None
    :param str name: objectName, defaults to None
    :param Callable[[int], str] label_fn: label provider (for custom row labels), defaults to None
    :param int count: number of autogenerated rows, defaults to 0
    :param bool resizable: allow resizing the list, defaults to False
    :param int min_count: minimum number of rows, defaults to 0
    :param bool add: add to the gui, defaults to True
    :param Dict[str, Any] **kwargs: Qt properties
    :return InputFloatListWidget: The widget
    """
    widget = InputFloatListWidget(
        count=count,
        label_fn=label_fn,
        values=values,
        resizable=resizable,
        min_count=min_count,
        **kwargs,
    )
    if name:
        widget.setName(name)
    if add:
        place_widget(widget, label=label)
    return widget


##% [Widget Impl] InputVectorWrapper
##% ────────────────────────────────────────────────────────────────────────────
class InputVectorWrapper:
    def __init__(self, g, x, y, z):
        self.group = g
        self.x = x
        self.y = y
        self.z = z

    def value(self) -> Vector:
        return Vector(self.x.value(), self.y.value(), self.z.value())

    def setValue(self, value):
        v = to_vec(value)
        self.x.setValue(v.x)
        self.y.setValue(v.y)
        self.z.setValue(v.z)


##% [Widget] InputVector
##% ────────────────────────────────────────────────────────────────────────────
def InputVector(
    label: Union[QWidget, str] = None,
    value: Union[Tuple, Vector] = (0.0, 0.0, 0.0),
) -> InputVectorWrapper:
    """
    Widget to accept a vector.

    Example:
    ~:code:../examples/ui/widgets.py[InputVector]:~

    :param Union[QWidget, str] label: ui label, defaults to None
    :param Union[Tuple, Vector] value: vector value, defaults to (0.0, 0.0, 0.0)
    :return InputVectorWrapper: The widget
    """
    with GroupBox(title=label) as g:
        with Col():
            x = InputFloat(label="X:")
            y = InputFloat(label="Y:")
            z = InputFloat(label="Z:")
            widget = InputVectorWrapper(g, x, y, z)
            widget.setValue(value)
            return widget


##% [Widget Impl] InputOptionsWidget
##% ────────────────────────────────────────────────────────────────────────────
class InputOptionsWidget(QComboBox):
    def __init__(self, data: Dict[str, Hashable]):
        super().__init__()
        self.index = dict()
        self.lookup = dict()
        for i, (label, value) in enumerate(data.items()):
            self.index[i] = value
            self.lookup[value] = i
            self.addItem(label)

    def value(self):
        return self.index.get(self.currentIndex(), None)

    def setValue(self, value):
        index = self.lookup.get(value, None)
        if index is not None:
            self.setCurrentIndex(index)


##% [Widget] InputOptions
##% ────────────────────────────────────────────────────────────────────────────
def InputOptions(
    options: Dict[str, Hashable],
    value: Hashable = None,
    label: str = None,
    name: str = None,
    stretch: int = 0,
    alignment: Qt.Alignment = Qt.Alignment(),
    **kwargs,
) -> InputOptionsWidget:
    """
    ComboBox widget

    Example:
    ~:code:../examples/ui/widgets.py[InputOptions]:~

    :param Dict[str, Hashable] options: label to value mapping
    :param Hashable value: initial value, defaults to None
    :param str label: gui label, defaults to None
    :param str name: objectName, defaults to None
    :param int stretch: layout stretch, defaults to 0
    :param Qt.Alignment alignment: layout alignment, defaults to Qt.Alignment()
    :param Dict[str, Any] **kwargs: Qt properties
    :return InputOptionsWidget: The widget
    """
    widget = InputOptionsWidget(options)
    set_qt_attrs(widget, **kwargs)
    if value is not None:
        widget.setValue(value)
    if name:
        widget.setObjectName(name)
    place_widget(widget, label=label, stretch=stretch, alignment=alignment)
    return widget


##% [Widget] InputSelectOne
##% ────────────────────────────────────────────────────────────────────────────
class InputSelectOne:
    def __init__(
        self,
        label: str = None,
        name: str = None,
        active: bool = False,
        auto_deactivate: bool = True,
    ):
        """
        3D Selection widget. Allows to pick an object

        Example:
        ~:code:../examples/ui/widgets.py[InputSelectOne]:~

        :param str label: gui label, defaults to None
        :param str name: objectName, defaults to None
        :param bool active: activated by default, defaults to False
        :param bool auto_deactivate: _description_, defaults to True
        """
        self._value = None
        self._pre = None
        self._auto_deactivate = auto_deactivate
        self.selected = PySignal()
        with Row(
            add=False,
            spacing=0,
            margin=0,
            contentsMargins=(0, 0, 0, 0),
        ) as ctl:

            @button(
                text=_tr_select,
                tool=True,
                checkable=True,
                styleSheet="QToolButton:checked{background-color: #FF0000; color:#FFFFFF;}",
                focusPolicy=Qt.FocusPolicy.NoFocus,
                objectName=name,
                checked=active,
            )
            def select():
                pass

            @button(
                tool=True,
                focusPolicy=Qt.FocusPolicy.NoFocus,
                icon=QIcon(":icons/edit-cleartext.svg"),
            )
            def clear():
                self.setValue(None)

            display = QLineEdit()
            display.setReadOnly(True)
            place_widget(display)

            self.display = display
            self.button = select
            register_select_observer(select, self)

            with Parent():
                place_widget(ctl, label=label)

    @property
    def active(self) -> bool:
        return self.button.isChecked()

    def value(self) -> Optional[SelectedObject]:
        return self._value

    def pre(self) -> Optional[SelectedObject]:
        return self._pre

    def setValue(self, value: Optional[SelectedObject]) -> None:
        self._value = value
        if value:
            self.display.setText(f"{value.doc}#{value.obj}.{value.sub}")
            if self._auto_deactivate:
                self.button.setChecked(False)
            self.selected.emit(self._value)
        else:
            self.display.setText("")

    def setPreselection(self, doc, obj, sub):
        if self.button.isChecked():
            self._pre = SelectedObject(doc, obj, sub)

    def addSelection(self, doc, obj, sub, pnt):
        if self.button.isChecked():
            self.setValue(SelectedObject(doc, obj, sub, pnt))

    def removeSelection(self, doc, obj, sub):
        if self.button.isChecked():
            if self._value:
                v = self._value
                if (v.doc, v.obj) == (doc, obj):
                    self.setValue(None)

    def setSelection(self, doc):
        if self.button.isChecked():
            self.setValue(SelectedObject(doc, Gui.Selection.getSelection()[-1].Name))

    def clearSelection(self, doc):
        pass


##% [Widget] InputSelectMany
##% ────────────────────────────────────────────────────────────────────────────
class InputSelectMany:
    ValueDataRole = Qt.UserRole

    def __init__(self, label: str = None, name: str = None, active: bool = False):
        """
        3D Multi-Selection Widget.

        Example:
        ~:code:../examples/ui/widgets.py[InputSelectMany]:~

        :param str label: gui label, defaults to None
        :param str name: objectName, defaults to None
        :param bool active: active by default or not, defaults to False
        """
        self._value = set()
        self.selected = PySignal()
        with Col(add=False, spacing=0, margin=0, contentsMargins=(0, 0, 0, 0)) as ctl:
            with Row(spacing=0, margin=0, contentsMargins=(0, 0, 0, 0)):

                @button(
                    text=_tr_add,
                    alignment=Qt.AlignLeft,
                    tool=True,
                    checkable=True,
                    styleSheet="QToolButton:checked{background-color: #FF0000; color:#FFFFFF;}",
                    focusPolicy=Qt.FocusPolicy.NoFocus,
                    objectName=name,
                    checked=active,
                )
                def select():
                    pass

                @button(
                    text=_tr_remove,
                    tool=True,
                    alignment=Qt.AlignLeft,
                    focusPolicy=Qt.FocusPolicy.NoFocus,
                )
                def remove():
                    selected = self.display.selectedItems()
                    for item in selected:
                        value = item.data(0, InputSelectMany.ValueDataRole)
                        self._value.remove(value)
                        self.display.takeTopLevelItem(self.display.indexOfTopLevelItem(item))

                @button(
                    text=_tr_clean,
                    tool=True,
                    alignment=Qt.AlignLeft,
                    focusPolicy=Qt.FocusPolicy.NoFocus,
                    icon=QIcon(":icons/edit-cleartext.svg"),
                )
                def clear():
                    self._value.clear()
                    self.display.clear()

                Stretch()

            display = QTreeWidget()
            display.setColumnCount(2)
            display.setHeaderLabels([_tr_object, _tr_sub_object])
            place_widget(display)

            self.display = display
            self.button = select
            register_select_observer(select, self)

            with Parent():
                with GroupBox(title=label):
                    place_widget(ctl)

    @property
    def active(self) -> bool:
        return self.button.isChecked()

    def value(self) -> List[SelectedObject]:
        return self._value

    def addValue(self, value: SelectedObject) -> None:
        if value not in self._value:
            item = QTreeWidgetItem([value.obj, value.sub])
            item.setData(0, InputSelectMany.ValueDataRole, value)
            self.display.addTopLevelItem(item)
            self._value.add(value)
            self.selected.emit(value)

    def setPreselection(self, doc, obj, sub):
        pass

    def addSelection(self, doc, obj, sub, pnt):
        if self.button.isChecked():
            self.addValue(SelectedObject(doc, obj, sub, pnt))

    def removeSelection(self, doc, obj, sub):
        pass

    def setSelection(self, doc):
        if self.button.isChecked():
            self.addValue(SelectedObject(doc, Gui.Selection.getSelection()[-1].Name))

    def clearSelection(self, doc):
        pass


##% [Widget] button
##@ [Decorator] button
##% ────────────────────────────────────────────────────────────────────────────
def button(
    label: str = None,
    *,
    tool: bool = False,
    icon: Union[QIcon, str] = None,
    stretch: int = 0,
    alignment: Qt.Alignment = Qt.Alignment(),
    add: bool = True,
    **kwargs,
) -> QAbstractButton:
    """
    Button Widget.

    Example:
    ~:code:../examples/ui/widgets.py[buttons]:~

    :param str label: text of the button, defaults to None
    :param bool tool: use tool style button, defaults to False
    :param Union[QIcon, str] icon: icon, defaults to None
    :param int stretch: layout stretch, defaults to 0
    :param Qt.Alignment alignment: layout alignment, defaults to Qt.Alignment()
    :param bool add: add to current context, defaults to True
    :return QAbstractButton: The widget
    """
    btn = QToolButton() if tool else QPushButton()
    set_qt_attrs(btn, **kwargs)

    if label:
        btn.setText(label)
    elif "text" not in kwargs:
        btn.setText("Button")

    if isinstance(icon, QIcon):
        btn.setIcon(icon)
    elif isinstance(icon, str):
        btn.setIcon(QIcon(icon))

    if add:
        place_widget(btn, stretch=stretch, alignment=alignment)

    def wrapper(handler) -> QAbstractButton:
        btn.clicked.connect(handler)
        return btn

    return wrapper


##% [Gui] ProgressIndicator
##% ────────────────────────────────────────────────────────────────────────────
class ProgressIndicator:
    """
    This wrapper is required because there is a bug with
    Base.ProgressIndicator on MacOS
    """

    def __init__(self, *args, **kwargs) -> None:
        try:
            self.control = Base.ProgressIndicator(*args, **kwargs)
        except Exception:
            self.control = None

    def start(self, *args, **kwargs):
        if self.control:
            self.control.start(*args, **kwargs)

    def next(self, *args, **kwargs):
        if self.control:
            self.control.next(*args, **kwargs)

    def stop(self, *args, **kwargs):
        if self.control:
            self.control.stop(*args, **kwargs)


##% [Context] progress_indicator
##% ────────────────────────────────────────────────────────────────────────────
@contextmanager
def progress_indicator(message: str = None, steps: int = 0):
    """
    Indicator of progress context manager

    Example:
    ~:code:../examples/ui/widgets.py[progress_indicator]:~

    :param str message: Message, defaults to 'Working...'
    :param int steps: max number of steps, defaults to 0
    :yield ProgressIndicator: The progress indicator controller
    """
    bar = ProgressIndicator()
    bar.start(message or _tr_working, steps)
    try:
        yield bar
    finally:
        bar.stop()
        del bar


##% [Widget Impl] SvgImageViewWidget
##% ────────────────────────────────────────────────────────────────────────────
class SvgImageViewWidget(QWidget):
    """
    High resolution Svg Widget
    """

    def __init__(self, uri: str):
        super().__init__()
        self.uri = uri
        self._size = (0, 0)
        self._renderer = None
        self.update_img(QRect(QPoint(0, 0), self.size()))

    def update_img(self, rect: QRect):
        if not self.uri or not Path(self.uri).exists():
            return

        renderer = QSvgRenderer(self.uri, self)
        view_box = renderer.viewBoxF()

        width, height = 0, 0
        r_width, r_height = rect.width(), rect.height()
        svg_width, svg_height = view_box.width(), view_box.height()

        if svg_width <= 0 or svg_height <= 0:
            svg_width = r_width
            svg_height = r_height

        scale_w, scale_h = 1.0, 1.0
        if svg_width > 0 and r_width > 0:
            scale_w = r_width / svg_width
        if svg_height > 0 and r_height > 0:
            scale_h = r_height / svg_height

        scale = min(scale_h, scale_w)
        width = svg_width * scale
        height = svg_height * scale

        self._renderer = renderer
        self._size = (width, height)
        self.setMinimumSize(width, height)

    def paintEvent(self, e):
        qp = QPainter(self)
        try:
            rect = e.rect()
            if (
                self._size == (0, 0)
                or self._size != (rect.width(), rect.height())
                or self._renderer is None
            ):
                self.update_img(rect)
            if self._renderer:
                qp.save()
                qp.setViewport(0, 0, *self._size)
                self._renderer.render(qp)
                qp.restore()
        finally:
            qp.end()

    def setValue(self, uri):
        self.uri = uri
        self._renderer = None
        self.update()

    def value(self):
        return self.uri


##% [Widget Impl] ImageViewWidget
##% ────────────────────────────────────────────────────────────────────────────
class ImageViewWidget(QLabel):
    def __init__(self, uri, background=None) -> None:
        super().__init__()
        self.uri = uri
        self._pixmap = QPixmap(uri)
        self._bg = None
        if isinstance(background, QColor):
            self._bg = background
        elif isinstance(background, str):
            self._bg = Color(code=background)

    def setValue(self, uri):
        self._pixmap = QPixmap(uri)
        self.update()

    def value(self):
        return self.uri

    def paintEvent(self, e):
        qp = QPainter(self)
        try:
            qp.setRenderHint(QPainter.SmoothPixmapTransform)
            winSize = e.rect()
            if self._bg:
                qp.fillRect(winSize, self._bg)
            pixmapRatio = (
                float(self._pixmap.width()) / self._pixmap.height()
                if self._pixmap.height()
                else 1.0
            )
            windowRatio = float(winSize.width()) / winSize.height() if winSize.height() else 1.0
            if pixmapRatio < windowRatio:
                newWidth = int(winSize.height() * pixmapRatio)
                qp.drawPixmap(0, 0, newWidth, winSize.height(), self._pixmap)
            else:
                newHeight = int(winSize.width() / pixmapRatio)
                qp.drawPixmap(0, 0, winSize.width(), newHeight, self._pixmap)
        finally:
            qp.end()


##% [Widget] ImageView
##% ────────────────────────────────────────────────────────────────────────────
def ImageView(
    uri: str,
    *,
    label: str = None,
    name: str = None,
    background: Union[str, QColor] = None,
    add: bool = True,
    **kwargs,
) -> ImageViewWidget:
    """
    Image render widget

    Example:
    ~:code:../examples/ui/widgets.py[ImageView]:~

    :param str uri: path to load the image from
    :param str label: gui label, defaults to None
    :param str name: objectName, defaults to None
    :param Union[str, QColor] background: background color, defaults to None
    :param bool add: _description_, defaults to True
    :return ImageViewWidget: _description_
    """
    widget = ImageViewWidget(uri, background)
    if name:
        widget.setObjectName(name)
    set_qt_attrs(widget, **kwargs)
    if add:
        place_widget(widget, label=label)
    return widget


##% [Widget] SvgImageView
##% ────────────────────────────────────────────────────────────────────────────
def SvgImageView(
    uri: str,
    name: str = None,
    label: str = None,
    stretch=0,
    alignment=Qt.Alignment(),
    **kwargs,
) -> SvgImageViewWidget:
    """
    High resolution Svg Image box

    Example:
    ~:code:../examples/ui/widgets.py[SvgImageView]:~

    :param str uri: file path if the svg
    :param str name: Widget name, defaults to None
    :return SvgImageViewWidget: The widget
    """
    widget = SvgImageViewWidget(uri)
    if name:
        widget.setObjectName(name)
    set_qt_attrs(widget, **kwargs)
    place_widget(widget, label, stretch=stretch, alignment=alignment)
    return widget


##% [Widget Impl] TableWidget
##% ────────────────────────────────────────────────────────────────────────────
class TableWidget(QTableWidget):
    def __init__(self, headers: List[str], rows: List[List[Any]]):
        num_rows = len(rows)
        num_cols = len(headers)
        super().__init__(num_rows, num_cols)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)

        def column(col_spec: str):
            if col_spec.startswith("<"):
                return (col_spec[1:], Qt.AlignLeft)
            if col_spec.startswith(">"):
                return (col_spec[1:], Qt.AlignRight)
            if col_spec.startswith("^"):
                return (col_spec[1:], Qt.AlignCenter)
            return (col_spec, Qt.AlignLeft)

        aligns = []
        for col in range(num_cols):
            text, align = column(str(headers[col]))
            aligns.append(align | Qt.AlignVCenter)
            item = QTableWidgetItem(text)
            item.setTextAlignment(align | Qt.AlignVCenter)
            self.setHorizontalHeaderItem(col, item)

        self.num_cols = num_cols
        self.aligns = aligns
        self.setRowsData(rows)

    def setRowsData(self, rows: List[List[Any]]):
        num_rows = len(rows)
        aligns = self.aligns
        self.setRowCount(0)  # Force Clean
        self.setRowCount(num_rows)
        for row in range(num_rows):
            row_data = rows[row]
            for col in range(min(self.num_cols, len(row_data))):
                text, align = row_data[col], aligns[col]
                item = QTableWidgetItem("" if text is None else str(text))
                item.setTextAlignment(align)
                self.setItem(row, col, item)


##% [Widget Impl] Table
##% ────────────────────────────────────────────────────────────────────────────
def Table(
    headers: List[str],
    rows: List[List[Any]],
    *,
    name: str = None,
    stretch: int = 0,
    alignment: Qt.Alignment = Qt.Alignment(),
    add: bool = True,
    **kwargs,
) -> TableWidget:
    """
    Simple Table output widget

    Example:
    ~:code:../examples/ui/widgets.py[Table]:~

    :param List[str] headers: column headers
    :param List[List[Any]] rows: data
    :param str name: objectName, defaults to None
    :param int stretch: layout stretch, defaults to 0
    :param Qt.Alignment alignment: layout alignment, defaults to Qt.Alignment()
    :param bool add: add to current context, defaults to True
    :return TableWidget: The table widget
    """
    table = TableWidget(headers, rows)
    set_qt_attrs(table, **kwargs)
    if name:
        table.setObjectName(name)
    if add:
        place_widget(table, stretch=stretch, alignment=alignment)
    return table


##% [Concurrency] Main Thread Hook
##% ────────────────────────────────────────────────────────────────────────────
class _ui_thread_hook_class(QObject):
    queued = Signal(object)

    def __init__(self):
        super().__init__(QApplication.instance())
        self.moveToThread(QApplication.instance().thread())
        self.queued.connect(self.run, Qt.QueuedConnection)

    def send(self, func):
        """
        Callable from any thread. Queue the callable object `func`
        to execute in main thread as soon as possible.
        """
        self.queued.emit(func)

    @Slot(object)
    def run(self, func):
        """
        Runs the callable `func` in main thread.
        """
        func()


##@ [Decorator] ui_thread
##@ ────────────────────────────────────────────────────────────────────────────
def ui_thread(delay: int = 0):
    """
    Decorator for running callables in Qt's UI Thread.

    :param int delay: delay in milliseconds
    """

    def decorator(f):
        @wraps(f)
        def wrapper(*args, **kwargs):
            if delay > 0:
                _ui_thread_hook_obj.send(
                    lambda: QTimer.singleShot(
                        delay,
                        lambda: f(*args, **kwargs),
                    ),
                )
            else:
                _ui_thread_hook_obj.send(lambda: f(*args, **kwargs))

        return wrapper

    return decorator


##% [Helper] CanvasHelper
##% ────────────────────────────────────────────────────────────────────────────
class CanvasHelper:
    """
    Provides contextual styles for canvas related tasks.
    """

    def __init__(self, widget: QWidget, painter: QPainter, event: QPaintEvent) -> None:
        self.widget = widget
        self.painter = painter
        self.event = event

    def setBackgroundColor(self, color: QColor):
        self.painter.fillRect(self.event.rect(), color)

    @contextmanager
    def pen(self, **kwargs):
        self._old_pen = self.painter.pen()
        pen = QPen()
        set_qt_attrs(pen, **kwargs)
        self.painter.setPen(pen)
        yield pen
        self.painter.setPen(self._old_pen)

    @contextmanager
    def brush(self, **kwargs):
        self._old_brush = self.painter.brush()
        brush = QBrush()
        set_qt_attrs(brush, **kwargs)
        self.painter.setBrush(brush)
        yield brush
        self.painter.setBrush(self._old_brush)


##% [Widget Impl] CanvasWidget
##% ────────────────────────────────────────────────────────────────────────────
class CanvasWidget(QWidget):
    def __init__(
        self,
        *,
        paint: Callable[[QWidget, QPainter, QPaintEvent], None] = None,
        setup: Callable[[QWidget, QPainter, QPaintEvent], None] = None,
        parent: QWidget = None,
    ):
        super().__init__(parent)
        self._paint = paint
        self._setup = setup

    def paintEvent(self, e):
        qp = QPainter(self)
        qp.save()
        helper = CanvasHelper(self, qp, e)
        try:
            if self._setup:
                self._setup(self, qp, helper)
            if self._paint:
                self._paint(self, qp, helper)
        finally:
            qp.restore()
            qp.end()


##% [Widget] Canvas
##% ────────────────────────────────────────────────────────────────────────────
def Canvas(
    paint: Callable[[QWidget, QPainter, QPaintEvent], None],
    *,
    setup: Callable[[QWidget, QPainter, QPaintEvent], None] = None,
    name: str = None,
    stretch: int = 0,
    width: int = 0,
    height: int = 0,
    **kwargs,
) -> CanvasWidget:
    """
    Canvas Widget to allow custom painting

    Example:
    ~:code:../examples/ui/widgets.py[Canvas]:~

    :param Callable[[QWidget, QPainter, QPaintEvent], None] paint: function to paint
    :param Callable[[QWidget, QPainter, QPaintEvent], None] setup: function to setup canvas, defaults to None
    :param str name: objectName, defaults to None
    :param int stretch: layout stretch, defaults to 0
    :param int width: minimum width, defaults to 0
    :param int height: minimum height, defaults to 0
    :return CanvasWidget: The widget
    """
    widget = CanvasWidget(paint=paint, setup=setup)
    widget.setMinimumSize(width, height)
    set_qt_attrs(widget, **kwargs)
    if name:
        widget.setObjectName(name)
    place_widget(widget, stretch=stretch, alignment=Qt.AlignVCenter | Qt.AlignCenter)
    return widget


##% [Widget Impl] HeaderWidget
##% ────────────────────────────────────────────────────────────────────────────
class HeaderWidget(QWidget):
    def __init__(self, text: str, line=True, **kwargs):
        super().__init__()
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self.setLayout(layout)

        label = QLabel(text)
        label.setStyleSheet("font-weight: bold")
        label.setProperty("styleClass", "header-label")
        set_qt_attrs(label, **kwargs)
        layout.addWidget(label)

        if line:
            sep = QFrame()
            label.setProperty("styleClass", "header-line")
            sep.setFrameShape(QFrame.HLine)
            sep.setFrameShadow(QFrame.Sunken)
            layout.addWidget(sep)


##% [Widget] Header
##% ────────────────────────────────────────────────────────────────────────────
def Header(text: str, add: bool = True, **kwargs) -> HeaderWidget:
    """
    Simple header label

    Example:
    ~:code:../examples/ui/widgets.py[Header]:~

    :param str text: the text
    :param bool add: add to current context, defaults to True
    :return HeaderWidget: the widget
    """
    widget = HeaderWidget(text, **kwargs)
    if add:
        place_widget(widget)
    return widget


##% [Widget Impl] LogViewWidget
##% ────────────────────────────────────────────────────────────────────────────
class LogViewWidget(QPlainTextEdit):
    def __init__(
        self,
        style: str = None,
        err_style: str = None,
        warn_style: str = None,
    ) -> None:
        super().__init__()
        self.style = style
        self.err_style = err_style or ""
        self.warn_style = warn_style or ""
        if style:
            self.setStyleSheet(style)
        self.setReadOnly(True)
        self.setCenterOnScroll(True)

    def info(self, message: str):
        self.appendHtml(f"<p>{message}</p>")
        self.ensureCursorVisible()

    def error(self, message: str):
        self.appendHtml(f"<p style='{self.err_style}'>{message}</p>")
        self.ensureCursorVisible()

    def warn(self, message: str):
        self.appendHtml(f"<p style='{self.warn_style}'>{message}</p>")
        self.ensureCursorVisible()


##% [Widget] LogView
##% ────────────────────────────────────────────────────────────────────────────
def LogView(
    style: str = None,
    err_style: str = None,
    warn_style: str = None,
    add=True,
    **kwargs,
):
    widget = LogViewWidget(style, err_style, warn_style)
    set_qt_attrs(widget, **kwargs)
    if add:
        place_widget(widget)
    return widget


##% [Widget] Section
##% ────────────────────────────────────────────────────────────────────────────
@contextmanager
def Section(header: Union[QWidget, str], indent: int = 0, **kwargs):
    with Container():
        with Col(contentsMargins=(indent, 0, 0, 0), spacing=0):
            if isinstance(header, QWidget):
                place_widget(header)
            else:
                Header(header)
            with Container(contentsMargins=(indent, 0, 0, 0)):
                yield Container(**kwargs)


##: [SECTION] Utility functions
##: ────────────────────────────────────────────────────────────────────────────


# ──────────────────────────────────────────────────────────────────────────────
def margins(
    left: int = 0,
    top: int = 0,
    right: int = 0,
    bottom: int = 0,
) -> QMargins:
    return QMargins(left, top, right, bottom)


# ──────────────────────────────────────────────────────────────────────────────
def update_style(widget: QWidget) -> None:
    """
    Force widget style refresh

    :param QWidget widget: the widget
    """
    style = widget.style()
    style.unpolish(widget)
    style.polish(widget)
    widget.update()


# ──────────────────────────────────────────────────────────────────────────────
def get_open_file(caption: str = None, filter: str = None) -> Union[str, None]:
    """
    Open File Dialog

    :param str caption: dialog title, defaults to "Open"
    :param _type_ filter: filter pattern, defaults to None
    :return _type_: selected file name
    """
    names = QFileDialog.getOpenFileName(
        QApplication.activeWindow(),
        caption=caption or _tr_open,
        filter=filter,
    )
    if names and names[0]:
        return names[0]


# ──────────────────────────────────────────────────────────────────────────────
def get_save_file(
    caption: str = None,
    filter: str = None,
    file: str = None,
) -> Union[str, None]:
    """
    Save File Dialog

    :param str caption: dialog title, defaults to Save
    :param str filter: filter pattern, defaults to None
    :param str file: default filename, defaults to None
    :return str: target file name
    """
    names = QFileDialog.getSaveFileName(
        QApplication.activeWindow(),
        caption=caption or _tr_save,
        dir=file,
        filter=filter,
    )
    if names and names[0]:
        return names[0]


# ──────────────────────────────────────────────────────────────────────────────
def load_font(path: str):
    """
    Load font into Qt

    :param str path: filesystem path to the font
    """
    QFontDatabase.addApplicationFont(path)


# ──────────────────────────────────────────────────────────────────────────────
@ui_thread(delay=10)
def show_info(
    message,
    title=None,
    std_icon=QMessageBox.Information,
    std_buttons=QMessageBox.Ok,
    parent=None,
):
    diag = QMessageBox(parent or find_active_window())
    diag.setIcon(std_icon)
    diag.setWindowTitle(str(title) if title else _tr_information)
    diag.setText(str(message))
    diag.setStandardButtons(std_buttons)
    diag.open()


# ──────────────────────────────────────────────────────────────────────────────
def show_warning(
    message,
    title=None,
    std_icon=QMessageBox.Warning,
    std_buttons=QMessageBox.Ok,
    parent=None,
):
    show_info(message, title or _tr_warning, std_icon, std_buttons, parent)


# ──────────────────────────────────────────────────────────────────────────────
def show_error(
    message,
    title=None,
    std_icon=QMessageBox.Critical,
    std_buttons=QMessageBox.Ok,
    parent=None,
):
    show_info(message, title or _tr_error, std_icon, std_buttons, parent)


# ──────────────────────────────────────────────────────────────────────────────
def qt_get_widget_path(widget: QWidget, index: int) -> str:
    """
    Returns a string representation if widget hierarchy position as a path.
    i.e. root/mainWindow/QFrame/QLabel_1
    """
    name = widget.objectName()
    if not name:
        name = f"{widget.__class__.__name__}_{index}"
    path = [name]
    parent = widget.parent()
    while parent:
        path.append(parent.objectName() or parent.__class__.__name__)
        parent = parent.parent()
    return "/".join(reversed(path))


# ──────────────────────────────────────────────────────────────────────────────
def save_widget_state(widget: QWidget, file: Union[Path, str]):
    """
    Dump all values of child nodes into file in json format.
    """
    data = {}
    index = 0
    for child in widget.findChildren(QWidget):
        if hasattr(child, "value"):
            path = qt_get_widget_path(child, index)
            index += 1
            try:
                data[path] = child.value()
            except Exception:
                print_log(f"Ignoring value of {path}")
    with open(file, "w") as f:
        f.write(json.dumps(data))


# ──────────────────────────────────────────────────────────────────────────────
def load_widget_state(widget, file):
    """
    Load values from json file into widget child nodes.
    """
    file = Path(file)
    if file.exists():
        with open(file, "r") as f:
            data = json.load(f)
            index = 0
            for child in widget.findChildren(QWidget):
                if hasattr(child, "setValue"):
                    path = qt_get_widget_path(child, index)
                    index += 1
                    if path in data:
                        try:
                            child.setValue(data[path])
                        except Exception:
                            print_log(f"Ignoring value of {path} because it was not found")


# ──────────────────────────────────────────────────────────────────────────────
def find_active_window():
    """
    Return the current active window.
    """
    focus = QApplication.focusWidget()
    if focus:
        parent = focus.parent()
        if isinstance(parent, (QDialog, QMainWindow)) and parent.isVisible():
            return parent
    return Gui.getMainWindow()


# ──────────────────────────────────────────────────────────────────────────────
def get_tr(context: str) -> Callable[[str], str]:
    """
    Returns a translation function for the given context

    :param str context: translation context
    :return Callable[[str], str]: translation function
    """

    def tr(text: str) -> str:
        return QApplication.translate(context, text)

    return tr


# ──────────────────────────────────────────────────────────────────────────────
def print_log(*args):
    App.Console.PrintLog(f"[{_tr_info}] {' '.join(str(a) for a in args)}\n")


# ──────────────────────────────────────────────────────────────────────────────
def print_err(*args):
    App.Console.PrintError(f"[{_tr_error}] {' '.join(str(a) for a in args)}\n")


# ──────────────────────────────────────────────────────────────────────────────
def to_vec(input: Any) -> Vector:
    """
    Converts tuple/list/vector/object to Vector.
    """
    if isinstance(input, Vector):
        return input
    if isinstance(input, (tuple, list)):
        if len(input) == 3:
            return Vector(*input)
        if len(input) == 2:
            return Vector(*input, 0)
        if len(input) == 1:
            return Vector(*input, 0, 0)
    if isinstance(input, (float, int)):
        return Vector(input, 0, 0)
    if hasattr(input, "X"):
        if hasattr(input, "Y"):
            if hasattr(input, "Z"):
                return Vector(input.X, input.Y, input.Z)
            else:
                return Vector(input.X, input.Y, 0)
        else:
            return Vector(input.X, 0, 0)
    raise TypeError(f"Invalid input, {type(input)} is not convertible to Vector")


##: [SECTION] Module translations (i18n)
##: ────────────────────────────────────────────────────────────────────────────

_tr = get_tr("fcui")
_tr_info = _tr("Info")
_tr_error = _tr("Error")
_tr_add = _tr("Add")
_tr_remove = _tr("Remove")
_tr_clean = _tr("Clean")
_tr_object = _tr("Object")
_tr_sub_object = _tr("SubObject")
_tr_working = _tr("Working...")
_tr_open = _tr("Open")
_tr_save = _tr("Save")
_tr_information = _tr("Information")
_tr_warning = _tr("Warning")
_tr_select = _tr("Select...")

##: [SECTION] Globals
##: ────────────────────────────────────────────────────────────────────────────

# Global Main Thread hook
_ui_thread_hook_obj = _ui_thread_hook_class()

# UI Builders are not thread safe, so each thread has its own state.
_thread_local_gui_vars = threading.local()

# FreeCAD UI loader to access custom widgets
_fc_ui_loader = Gui.UiLoader()
