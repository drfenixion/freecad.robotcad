# SPDX-License: LGPL-3.0-or-later
# (c) 2024 Frank David Martínez Muñoz. <mnesarco at gmail.com>

# ruff: noqa: A002
# ruff: noqa: D102, D106, D401
# ruff: noqa: N801, N802, N807
# ruff: noqa: ANN401, A002
# ruff: noqa: PLR0913

"""fpo: FreeCAD Scripted Object API."""

from __future__ import annotations

__author__ = "Frank David Martínez Muñoz"
__copyright__ = "(c) 2024 Frank David Martínez Muñoz."
__license__ = "LGPL 2.1"
__version__ = "1.0.0-beta6"
__min_python__ = "3.10"
__min_freecad__ = "0.22"
__contributors__ = "Gaël Écorchard"


# Conventions for sections in this file:
# vscode: aaron-bond.better-comments adds comment colored highlight
# ──────────────────────────────────────────────────────────────────────────────
# #  Normal comments
# #: Code execute at import time, create objects in global module scope
# #$ Template code, creates injectable methods
# #@ Decorators code
# #% Type definitions
# #; Simple section label
# #! Warning note
# ──────────────────────────────────────────────────────────────────────────────

# General imports
# ──────────────────────────────────────────────────────────────────────────────
import contextlib
import inspect
import re
import sys
import textwrap
import traceback
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum, IntEnum
from pathlib import Path
from typing import (
    TYPE_CHECKING,
    Any,
    ClassVar,
    Generic,
    Protocol,
    TypeAlias,
    TypeVar,
)

import FreeCAD as App  # type: ignore[all]
from FreeCAD import (  # type: ignore[all]
    Document,  # type: ignore[all]
    DocumentObject,  # type: ignore[all]
)

if TYPE_CHECKING:
    from collections.abc import Generator, Hashable, Iterable

    from Part import Shape  # type: ignore[all]

##: Conditional imports
##: ────────────────────────────────────────────────────────────────────────────

if App.GuiUp:
    import FreeCADGui as Gui  # type: ignore[all]
    from FreeCADGui import ViewProviderDocumentObject  # type: ignore[all]
    from PySide import QtCore, QtGui  # type: ignore[all]

##: Logging
##: ────────────────────────────────────────────────────────────────────────────


def print_log(*args: tuple) -> None:
    """Print into the FreeCAD console with info level."""
    App.Console.PrintLog(f"[info] {' '.join(str(a) for a in args)}\n")


def print_err(*args: tuple) -> None:
    """Print into the FreeCAD console with error level."""
    App.Console.PrintError(f"[error] {' '.join(str(a) for a in args)}\n")


##: FreeCAD defined c++ types (Just for the sake of documentation)
##: ────────────────────────────────────────────────────────────────────────────
ObjectRef: TypeAlias = DocumentObject | ViewProviderDocumentObject


##: Events
##: ────────────────────────────────────────────────────────────────────────────
class events:
    """Namespace: All event classes."""

    @dataclass
    class ExtensionEvent:
        source: DocumentObject
        name: str

    @dataclass
    class AttachEvent:
        source: DocumentObject
        view_provider: ViewProviderDocumentObject | None = None

    @dataclass
    class CreateEvent:
        source: DocumentObject

    @dataclass
    class StartEvent:
        source: DocumentObject
        view_provider: ViewProviderDocumentObject | None = None

    @dataclass
    class DocumentRestoredEvent:
        source: DocumentObject

    @dataclass
    class ExecuteEvent:
        source: DocumentObject

    PT = TypeVar("PT")  # Property Type

    @dataclass
    class PropertyChangedEvent(Generic[PT]):
        source: DocumentObject
        property_name: str
        old_value: events.PT | None
        new_value: events.PT | None
        view_provider: ViewProviderDocumentObject | None = None

    @dataclass
    class PropertyWillChangeEvent(Generic[PT]):
        source: DocumentObject
        property_name: str
        value: events.PT | None
        view_provider: ViewProviderDocumentObject | None = None

    @dataclass
    class SerializeEvent:
        source: DocumentObject
        state: dict[str, Any]
        view_provider: ViewProviderDocumentObject | None = None

    @dataclass
    class DeserializeEvent:
        source: DocumentObject
        state: dict[str, Any]
        view_provider: ViewProviderDocumentObject | None = None

    @dataclass
    class RemoveEvent:
        source: DocumentObject

    @dataclass
    class ContextMenuEvent:
        source: DocumentObject
        view_provider: ViewProviderDocumentObject
        menu: QtGui.QMenu

    @dataclass
    class ClaimChildrenEvent:
        source: DocumentObject
        view_provider: ViewProviderDocumentObject

    @dataclass
    class EditStartEvent:
        source: DocumentObject
        view_provider: ViewProviderDocumentObject
        mode: int

    @dataclass
    class EditEndEvent:
        source: DocumentObject
        view_provider: ViewProviderDocumentObject
        mode: int

    @dataclass
    class DeleteEvent:
        source: DocumentObject
        view_provider: ViewProviderDocumentObject
        sub_elements: Any

    @dataclass
    class DoubleClickEvent:
        source: DocumentObject
        view_provider: ViewProviderDocumentObject

    @dataclass
    class DataChangedEvent:
        source: DocumentObject
        view_provider: ViewProviderDocumentObject
        property_name: str

    @dataclass
    class MigrationEvent:
        source: DocumentObject
        from_version: int
        to_version: int

    @dataclass
    class DragAndDropEvent:
        source: DocumentObject
        view_provider: ViewProviderDocumentObject
        dragged_object: DocumentObject


##: Defined symbols
##: ────────────────────────────────────────────────────────────────────────────
_SO_VERSION = "SOInternalVersion"
_SO_META = "__so_meta__"
_SO_REF = "__so_ref__"
_ON_EXTENSION = "on_extension"
_ON_ATTACH = "on_attach"
_ON_CREATE = "on_create"
_ON_START = "on_start"
_ON_RESTORE = "on_restore"
_ON_EXECUTE = "on_execute"
_ON_CHANGE = "on_change"
_ON_BEFORE_CHANGE = "on_before_change"
_ON_SERIALIZE = "on_serialize"
_ON_DESERIALIZE = "on_deserialize"
_ON_REMOVE = "on_remove"
_IS_DIRTY = "is_dirty"
_ON_CONTEXT_MENU = "on_context_menu"
_ON_CLAIM_CHILDREN = "on_claim_children"
_ON_EDIT_START = "on_edit_start"
_ON_EDIT_END = "on_edit_end"
_ON_DELETE = "on_delete"
_ON_DBL_CLICK = "on_dbl_click"
_GET_ICON = "icon"
_SET_DISPLAY_MODE = "set_display_mode"
_DISPLAY_MODES = "display_modes"
_DEFAULT_DISPLAY_MODE = "default_display_mode"
_ON_OBJECT_CHANGE = "on_object_change"
_CAN_DRAG_OBJECTS = "can_drag_objects"
_CAN_DROP_OBJECTS = "can_drop_objects"
_CAN_DRAG_OBJECT = "can_drag_object"
_CAN_DROP_OBJECT = "can_drop_object"
_ON_DRAG_OBJECT = "on_drag_object"
_ON_DROP_OBJECT = "on_drop_object"
_ON_MIGRATE_CLASS = "on_migrate_class"
_ON_MIGRATE_UP = "on_migrate_upgrade"
_ON_MIGRATE_DOWN = "on_migrate_downgrade"
_ON_MIGRATE_COMPLETE = "on_migrate_complete"
_ON_MIGRATE_ERROR = "on_migrate_error"
_SET_VERSION = "set_version"
_OBJECT = "Object"
_VIEW_OBJECT = "ViewObject"


##: Global private static registries
##: ────────────────────────────────────────────────────────────────────────────

# Mapping of Known Extensions Support by extension name
_extensions: dict[str, ExtensionSupport] = {}


# ┌────────────────────────────────────────────────────────────────────────────┐
# │ private Utility functions                                                  │
# └────────────────────────────────────────────────────────────────────────────┘


# ──────────────────────────────────────────────────────────────────────────────
def _f_arity(func: Callable) -> int:
    """Return the number of arguments of a function."""
    sig = inspect.signature(func)
    return len(sig.parameters)


# ──────────────────────────────────────────────────────────────────────────────
def _m_arity(method: Callable) -> int:
    """Return the number of arguments of a method (arity - 1)."""
    n = _f_arity(method) - 1
    if n < 0:
        msg = "Invalid method signature"
        raise ValueError(msg)
    return n


# ──────────────────────────────────────────────────────────────────────────────
def _call(obj: Any, name: str, *args: tuple, **kwargs: dict) -> Any:
    """Call a method on obj if exists."""
    if func := getattr(obj, name, None):
        return func(*args, **kwargs)
    return None


# ──────────────────────────────────────────────────────────────────────────────
def _snake_to_camel(text: str | None) -> str | None:
    """Transform text from snake naming to camel case."""
    if text:
        return "".join(token.capitalize() for token in text.split("_"))
    return None


# ──────────────────────────────────────────────────────────────────────────────
def _resolve_uri(path: str, base_dir: Path | None = None) -> str:
    """Resolve relative paths if prefixed with 'self:'."""
    if str(path).startswith("self:") and base_dir:
        rel_path_elements = path[5:].lstrip(" /").split("/")
        return str(Path(base_dir, *rel_path_elements))
    return path


# ──────────────────────────────────────────────────────────────────────────────
_Pct = TypeVar("_Pct")


def _property_and_meta(prop: Property) -> tuple[Property, property]:
    """Create an additional property to retrieve the property meta."""

    @property
    def _get_meta(target: Proxy) -> PropertyMeta:
        """Return updatable metadata of the property."""
        return PropertyMeta(prop, target.__so_ref__)

    return prop, _get_meta


class _prop_constructor(Generic[_Pct]):  # Allow return inferred return type
    """Create a constructor for a specific Property Type."""

    def __init__(self, prop_type: str, py_type: type[_Pct] = str) -> None:
        self.prop_type = prop_type
        self.py_type = py_type

    def __call__(
        self,
        *,
        name: str | None = None,
        section: str = "Data",
        default: _Pct | None = None,
        description: str = "",
        mode: PropertyMode | None = None,
        observer_func: Callable | None = None,
        link_property: str | bool = False,
        meta: bool = False,
    ) -> Property[_Pct] | tuple[Property[_Pct], PropertyMeta]:
        """
        Real Type is: Property[_Pct] | tuple[Property[_Pct], property].

        Function type is faked to allow property code completion.
        """
        prop = Property(
            type=self.prop_type,
            section=section,
            observer_func=observer_func,
            name=name,
            link_property=link_property,
            default=default,
            description=description,
            mode=PropertyMode.Default if mode is None else mode,
        )

        if meta:
            return _property_and_meta(prop)
        return prop


# ──────────────────────────────────────────────────────────────────────────────
def _is(types: type | tuple[type, ...]) -> Callable[[Any], bool]:
    """Return a predicate to check the type of the passed object."""

    def predicate(obj: Any) -> bool:
        return isinstance(obj, types)

    return predicate


# ──────────────────────────────────────────────────────────────────────────────
def _get_properties(cls: type) -> Iterable[tuple[str, Property]]:
    """Return the list of Properties defined in a proxy class."""
    return inspect.getmembers(cls, _is(Property))


# ──────────────────────────────────────────────────────────────────────────────
def _get_display_modes(cls: type) -> Iterable[tuple[str, DisplayMode]]:
    """Return the list of Display Modes defined in a proxy class."""
    return inspect.getmembers(cls, _is(DisplayMode))


# ──────────────────────────────────────────────────────────────────────────────
def _t_forward(cls: type, forward_from: str, forward_to: str) -> None:
    """
    Create a function that forwards the call to another function.

    :param str forward_from: name of the original function
    :param str forward_to: name of the target function
    """
    overridden = getattr(cls, forward_from, None)
    if overridden:
        msg = f"{forward_from} is already reserved. use {forward_to} instead"
        raise NameError(msg)

    forward = getattr(cls, forward_to, None)
    if forward:

        def handler(self: Any, *args: tuple, **kwargs: dict) -> Any:
            return forward(self, *args, **kwargs)

        handler.__signature__ = inspect.signature(forward)
        handler.__doc__ = forward.__doc__
        handler.__name__ = forward_from
        setattr(cls, forward_from, handler)


##% ┌───────────────────────────────────────────────────────────────────────────┐
##% │ Core types                                                                │
##% └───────────────────────────────────────────────────────────────────────────┘


##% ────────────────────────────────────────────────────────────────────────────
class _DocIntEnum(IntEnum):
    """
    IntEnum with member docs.

    See: https://stackoverflow.com/a/50473952/1524027
    """

    def __new__(cls: type, value: int, doc: str = "") -> IntEnum:
        self = int.__new__(cls, value)
        self._value_ = value
        self.__doc__ = doc
        return self


##% ────────────────────────────────────────────────────────────────────────────
class FeatureState(_DocIntEnum):
    """ScriptedObject state. See lifecycle."""

    Attaching = -1, "FreeCAD is creating and binding the objects"
    Creating = 0, "Setting up all stuff like properties, extensions, etc..."
    Created = 1, "Already created"
    Active = 2, "Ready for execution"
    Restoring = 3, "Restoring from FCStd document"
    Restored = 4, "Fully restored from FCStd document"


##% ────────────────────────────────────────────────────────────────────────────
class PropertyMode(_DocIntEnum):
    """Property mode flags."""

    # fmt: off
    Default = 0,       "No special property type"
    ReadOnly = 1,      "Property is read-only in the editor"
    Transient = 2,     "Property won't be saved to file"
    Hidden = 4,        "Property won't appear in the editor"
    Output = 8,        "Modified property doesn't touch its parent container"
    NoRecompute = 16,  "Modified property doesn't touch its container for recompute"
    NoPersist = 32,    "Property won't be saved to file at all"
    # fmt: on


##% ────────────────────────────────────────────────────────────────────────────
class PropertyEditorMode(_DocIntEnum):
    """Editor Modes."""

    # fmt: off
    Default = 0,   "No special mode"
    ReadOnly = 1,  "Property is read only in the editor"
    Hidden = 2,    "Property is hidden in the editor"
    # fmt: on


class PropertyStatus(_DocIntEnum):
    """Property status managed by setPropertyStatus/getPropertyStatus."""

    # fmt: off
    Touched = 0,             "touched property"
    Immutable = 1,           "can't modify property"
    ReadOnly = 2,            "for property editor"
    Hidden = 3,              "for property editor"
    Transient = 4,           "for property container save"
    MaterialEdit = 5,        "to turn ON PropertyMaterial edit"
    NoMaterialListEdit = 6,  "to turn OFF PropertyMaterialList edit"
    Output = 7,              "same effect as Prop_Output"
    LockDynamic = 8,         "prevent being removed from dynamic property"
    NoModify = 9,            "prevent causing Gui::Document::setModified()"
    PartialTrigger = 10,     "allow change in partial doc"
    NoRecompute = 11,        "don't touch owner for recompute on property change"
    Single = 12,             "for save/load of floating point numbers"
    Ordered = 13,            "for PropertyLists whether the order of the elements is relevant for the container using it"  # noqa: E501
    EvalOnRestore = 14,      "In case of expression binding, evaluate the expression on restore and touch the object on value change."  # noqa: E501
    Busy = 15,               "internal use to avoid recursive signaling"
    CopyOnChange = 16,       "for Link to copy the linked object on change of the property with this flag"  # noqa: E501
    UserEdit = 17,           "cause property editor to create button for user defined editing"

    # The following bits are corresponding to PropertyType set when the
    # property added. These types are meant to be static, and cannot be
    # changed in runtime. It is mirrored here to save the linear search
    # required in PropertyContainer::getPropertyType()

    PropDynamic = 21,      "indicating the property is dynamically added"
    PropNoPersist = 22,    "corresponding to Prop_NoPersist"
    PropNoRecompute = 23,  "corresponding to Prop_NoRecompute"
    PropReadOnly = 24,     "corresponding to Prop_ReadOnly"
    PropTransient = 25,    "corresponding to Prop_Transient"
    PropHidden = 26,       "corresponding to Prop_Hidden"
    PropOutput = 27,       "corresponding to Prop_Output"

    User1 = 28,  "user-defined status"
    User2 = 29,  "user-defined status"
    User3 = 30,  "user-defined status"
    User4 = 31,  "user-defined status"
    # fmt: on


##% ────────────────────────────────────────────────────────────────────────────
class EditMode(_DocIntEnum):
    """ViewProvider Edit Mode."""

    Default = (
        0,
        "The object will be edited using the mode defined \
         internally to be the most appropriate for the object type",
    )
    Transform = (
        1,
        "The object will have its placement editable with the \
         `Std TransformManip` command",
    )
    Cutting = (
        2,
        "This edit mode is implemented as available but currently \
         does not seem to be used by any object",
    )
    Color = (
        3,
        "The object will have the color of its individual faces \
         editable with the Part FaceColors command",
    )


##% ────────────────────────────────────────────────────────────────────────────
_PT = TypeVar("_PT")


class Property(Generic[_PT]):
    """
    Proxy object to create, access and manipulate remote freecad properties.

    Ref: https://wiki.freecad.org/FeaturePython_Custom_Properties
    """

    type: str  # Type of the supported property
    binding: str  # Name of the property on the proxy class
    observer_func: Callable  # Change listener
    observer_arity: int  # Change listener arity
    name: str  # Actual name of the property
    link_property: str | bool  # Name of the Link Property to configure
    section: str  # Capitalized single word due to FC limitations
    default: _PT  # Initial value
    description: str  # GUI description
    mode: PropertyMode  # PropertyEditor mode for the property
    enum: type[Enum]  # Type of enum used by "App::PropertyEnumeration"
    options: Callable  # Callable that provides a list of options, either
                       # as options() or options(obj)

    # ──────────
    def __init__(  # noqa: D107
        self,
        *,
        type: str,
        binding: str | None = None,
        section: str = "Data",
        observer_func: Callable | None = None,
        name: str | None = None,
        link_property: str | bool = False,
        default: _Pct | None = None,
        description: str = "",
        mode: PropertyMode = PropertyMode.Default,
        enum: type[Enum] | None = None,
        options: Callable | None = None,
    ) -> None:
        self.type = type
        self.binding = binding
        self.section = section
        self.observer_func = observer_func
        if observer_func:
            self.observer(observer_func)
        self.name = name or _snake_to_camel(binding)
        self.link_property = link_property
        self.default = default
        self.description = description
        self.mode = PropertyMode.Default if mode is None else mode
        self.enum = enum
        self.options = options

    ##@ ─────────
    FN = TypeVar("FN", bound=Callable)

    def observer(self, func: FN) -> FN:
        """Register the listener for property change event."""
        self.observer_func = func
        sig = inspect.signature(func)
        self.observer_arity = len(sig.parameters)
        return func

    # ──────────
    def create(self, fp: ObjectRef) -> None:
        """Add the property to the object and initialize it."""
        if self.name not in fp.PropertiesList:
            fp.addProperty(self.type, self.name, self.section, self.description, self.mode)
            if self.enum:
                setattr(fp, self.name, [str(e.value) for e in list(self.enum)])
            elif self.options:
                if _f_arity(self.options) == 0:
                    setattr(fp, self.name, self.options())
                else:
                    setattr(fp, self.name, self.options(fp))
            self.reset(fp)

    # ──────────
    def reset(self, fp: ObjectRef) -> None:
        """Set the value to its default."""
        if self.default is not None:
            self.update(fp, self.default)

    # ──────────
    def update(self, obj: ObjectRef, value: _PT) -> None:
        """Set the value of the property in the remote object."""
        if not hasattr(obj, self.name):
            return
        if self.enum:
            setattr(obj, self.name, str(value.value))
            return
        attr = getattr(obj, self.name)
        if hasattr(attr, "Value"):
            if isinstance(value, str):
                attr.Value = App.Units.Quantity(value)
            else:
                attr.Value = value
            return
        setattr(obj, self.name, value)

    # ──────────
    def read(self, obj: ObjectRef) -> _PT:
        """Get the value of the property from the remote object."""
        if hasattr(obj, self.name):
            v = getattr(obj, self.name)
            if self.enum:
                if v is None:
                    return next(iter(self.enum))
                return self.enum(v)

            if hasattr(v, "Value"):
                return v.Value

            return v
        return None

    # ──────────
    def set_mode(self, obj: ObjectRef, mode: PropertyEditorMode | int | str | list[str]) -> None:
        """Change editor mode for the property."""
        if hasattr(obj, self.name):
            obj.setEditorMode(self.name, mode)

    # ──────────
    def set_status(
        self,
        obj: ObjectRef,
        status: str | int | PropertyStatus | list[str | int | PropertyStatus],
    ) -> None:
        """Change editor status for the property."""
        if hasattr(obj, self.name):
            obj.setPropertyStatus(self.name, status)

    if TYPE_CHECKING:
        # This fake typing here is to allow code completion to infer the property type
        def __get__(self, obj, obj_type=None) -> _PT: ...  # noqa: ANN001, D105
        def __set__(self, obj, value: _PT) -> None: ...  # noqa: ANN001, D105


##% ────────────────────────────────────────────────────────────────────────────
class PropertyMeta:
    """Property metadata proxy."""

    prop: Property
    target: ObjectRef

    def __init__(self, property: Property, target: ObjectRef) -> None:
        """Bounded property proxy to ObjectRef."""
        self.prop = property
        self.target = target

    @property
    def mode(self) -> list[str]:
        return self.target.getEditorMode(self.prop.name)

    @mode.setter
    def mode(self, value: PropertyEditorMode | int | str | list[str]) -> None:
        self.prop.set_mode(self.target, value)

    @property
    def status(self) -> list[PropertyStatus]:
        return list(map(PropertyStatus, self.target.getPropertyStatus(self.prop.name)))

    @status.setter
    def status(self, value: str | int | PropertyStatus | list[str | int | PropertyStatus]) -> None:
        self.prop.set_status(self.target, value)

    @property
    def enum(self) -> type[Enum] | None:
        return self.prop.enum

    @enum.setter
    def enum(self, value: type[Enum]) -> None:
        self.prop.enum = value
        setattr(self.target, self.prop.name, [str(e.value) for e in list(value)])

    @property
    def options(self) -> list[str | int] | None:
        return self.target.getEnumerationsOfProperty(self.prop.name)

    @options.setter
    def options(self, value: list[str | int]) -> None:
        setattr(self.target, self.prop.name, list(map(str, value)))

    @property
    def description(self) -> str:
        return self.target.getDocumentationOfProperty(self.prop.name)

    @description.setter
    def description(self, value: str) -> None:
        self.prop.description = value
        self.target.setDocumentationOfProperty(self.prop.name, value)


##% ────────────────────────────────────────────────────────────────────────────
class DisplayMode:
    """Display Mode Manager."""

    name: str
    _builder_func: Callable
    is_default: bool

    # ──────────
    def __init__(
        self,
        name: str | None = None,
        *,
        is_default: bool = False,
        builder: Callable | None = None,
    ) -> None:
        """
        Declare a display mode.

        Declarator of Display Modes, allows to configure a mode and optionally
        a builder method to create and register the coin object.

        :param str name: Name of the display mode
        :param bool is_default: Configure the DM as default, defaults to False
        :param Callable[[ViewObject], coin.SoGroup] builder: Method to build the coin object
        """
        self.name = name
        self.is_default = is_default
        self._builder_func = builder

    ##@ ─────────
    FN = TypeVar("FN", bound=Callable)

    def builder(self, func: FN) -> FN:
        """Bind the builder for the coin object."""
        self._builder_func = func
        return func


##% ────────────────────────────────────────────────────────────────────────────
@dataclass
class _Template:
    """
    Code template.

    Internal class used to inject tailored methods into the proxy objects to
    interface between FreeCAD internal objects and python proxy instances with
    the new API.
    """

    name: str  # Name of the method to be created
    aliases: Iterable[str]  # Aliases to be injected also
    builder: Callable  # The actual method builder
    allow_override: bool  # allow/deny user defined methods that collide
    override_error_msg: str  # Message to suggest alternative names

    # ──────────
    def __call__(self, meta: TypeMeta) -> None:
        """Apply the template."""
        self._add(meta, self.name, allow_override=self.allow_override)
        if self.aliases:
            for name in self.aliases:
                self._add(meta, name, allow_override=True)

    # ──────────
    def _add(self, meta: TypeMeta, name: str, *, allow_override: bool = False) -> None:
        """Build the method and inject with the name."""
        overridden = getattr(meta.cls, name, None)
        if overridden and not allow_override:
            msg = f"Attribute {name} is already defined. {self.override_error_msg or ''}"
            raise NameError(msg)

        if attr := self.builder(overridden, meta):
            if hasattr(attr, "__name__"):
                attr.__name__ = name
            setattr(meta.cls, name, attr)

    # ──────────
    @property
    def __doc__(self) -> str | None:
        return self.builder.__doc__


##% ────────────────────────────────────────────────────────────────────────────
class ExtensionSupport:
    """Base class of extension managers."""

    name: str  # Name of the supported extension

    # ──────────
    def __init__(self, name: str) -> None:
        """Create and register the extension Manager."""
        self.name = name
        _extensions[name] = self

    # ──────────
    def on_create(self, proxy: Proxy, obj: ObjectRef, meta: TypeMeta) -> None:
        """Extension listener for on_create event."""

    # ──────────
    def on_restore(self, proxy: Proxy, obj: ObjectRef, meta: TypeMeta) -> None:
        """Extension listener for on_restore event."""

    # ──────────
    def on_attach(self, proxy: Proxy, obj: ObjectRef, _meta: TypeMeta) -> None:
        """Extension listener for on_attach event."""
        self.add_extension(proxy, obj)

    # ──────────
    def on_start(self, proxy: Proxy, obj: ObjectRef, _meta: TypeMeta) -> None:
        """Extension listener for on_start event."""
        self.add_extension(proxy, obj)

    # ──────────
    def on_execute(self, proxy: Proxy, obj: ObjectRef, meta: TypeMeta) -> None:
        """Extension listener for on_execute event."""

    # ──────────
    def on_add(self, proxy: Proxy, obj: ObjectRef, meta: TypeMeta) -> None:
        """Extension listener for on_add event."""

    # ──────────
    def on_change(self, proxy: Proxy, obj: ObjectRef, meta: TypeMeta, prop: str) -> None:
        """Extension listener for on_change event."""

    # ──────────
    def on_class(self, meta: TypeMeta) -> None:
        """Extension listener for on_class event."""

    # ──────────
    def add_extension(self, proxy: Proxy, obj: ObjectRef, name: str | None = None) -> None:
        """Add the extension to the object."""
        _name = name or self.name
        if _name not in _extensions:
            msg = f"Extension {_name} not found."
            raise NameError(msg)

        if not obj.hasExtension(_name):
            obj.addExtension(_name)
            meta = self.find_meta(proxy)
            self.on_add(proxy, obj, meta)
            if handler := _event_handler(meta.cls, _ON_EXTENSION):
                handler(proxy, events.ExtensionEvent(obj, _name))

    # ──────────
    def find_meta(self, proxy: Proxy) -> TypeMeta:
        """Return the meta info associated with the proxy."""
        meta = getattr(proxy.__class__, _SO_META, None)
        if meta is None:
            msg = f"Invalid proxy type: {proxy.__class__.__name__}"
            raise TypeError(msg)
        return meta


##% ────────────────────────────────────────────────────────────────────────────
class Part_AttachExtensionPython(ExtensionSupport):
    """Extension manager of: Part::AttachExtensionPython."""

    # ──────────
    def on_execute(self, _proxy: DataProxy, obj: DocumentObject, _meta: TypeMeta) -> None:
        obj.positionBySupport()


##% ────────────────────────────────────────────────────────────────────────────
class App_LinkBaseExtensionPython(ExtensionSupport):
    """Extension manager of App::LinkBaseExtensionPython."""

    # ──────────
    def on_class(self, meta: TypeMeta) -> None:
        meta.view_provider_name_override = "Gui::ViewProviderLinkPython"

    # ──────────
    def resolve_link_prop(self, link_property: str | bool, source: str) -> str:
        """Return the name of the linked property."""
        if isinstance(link_property, bool) and link_property:
            return source
        return link_property

    # ──────────
    def on_start(self, proxy: DataProxy, obj: DocumentObject, meta: TypeMeta) -> None:
        super().on_start(proxy, obj, meta)
        mapping = {
            self.resolve_link_prop(prop.link_property, prop.name): prop.name
            for prop in meta.properties.values()
            if prop.link_property
        }
        if len(mapping) > 0:
            obj.configLinkProperty(**mapping)


##% ────────────────────────────────────────────────────────────────────────────
class App_LinkExtensionPython(App_LinkBaseExtensionPython):
    """Extension manager of App::LinkBaseExtensionPython."""


##% ────────────────────────────────────────────────────────────────────────────
class TypeMeta:
    """
    Metadata of the proxy classes.

    Inspects the original class and provides mechanisms to enhance them by adding
    methods, properties, display modes, extensions, etc...
    """

    cls: type  # proxy enhanced class
    properties: dict[str, Property]  # map binding to property
    property_lookup: dict[str, Property]  # map name to property
    display_modes: dict[str, DisplayMode]  # display mode builders
    view_proxy: type[ViewProxy]  # associated ViewProxy
    extensions: set[ExtensionSupport]  # Added extensions
    version: int  # Proxy versions for migrations support
    object_type: str  # Type of DocumentObject
    subtype: str  # Subtype of Proxy
    base_dir: Path  # Directory where the class is declared
    view_provider_name_override: str  # Forced type of the view provider
    icon: str  # Icon path

    # ──────────
    def __init__(  # noqa: D107
        self,
        cls: type,
        object_type: str | None = None,
        base_dir: Path | None = None,
        subtype: str | None = None,
        view_proxy: Any = None,
        extensions: Iterable[str] | None = None,
        version: int = 1,
        view_provider_name_override: str | None = None,
        icon: str | None = None,
    ) -> None:
        self.cls = cls
        self.version = version
        self.object_type = object_type
        self.subtype = subtype
        self.base_dir = base_dir
        self.properties = {}
        self.property_lookup = {}
        self.display_modes = {}
        self.icon = icon

        for name, prop in _get_properties(self.cls):
            if not bool(prop.binding):
                prop.binding = name
            if not bool(prop.name):
                prop.name = _snake_to_camel(name)
            self.properties[prop.binding] = prop
            self.property_lookup[prop.name] = prop

        for name, dm in _get_display_modes(self.cls):
            if not bool(dm.name):
                dm.name = name
            self.display_modes[name] = dm

        self.view_proxy = view_proxy
        if extensions:
            self.extensions = {_extensions[name] for name in extensions}
        else:
            self.extensions = set()
        self.view_provider_name_override = view_provider_name_override
        cls.__so_meta__ = self
        self.bind_properties()

    # ──────────
    def bind_properties(self) -> None:
        """Bind all available properties."""
        for prop in self.properties.values():
            self.bind_property(prop)

    # ──────────
    def bind_property(self, prop: Property) -> None:
        """Bind a property to a local proxy attribute."""

        def getter(self: Any) -> Any:
            return prop.read(self.__so_ref__)

        def setter(self: Any, value: Any) -> None:
            prop.update(self.__so_ref__, value)

        setattr(
            self.cls,
            prop.binding,
            property(getter, setter, doc=prop.description),
        )

    # ──────────
    def add_property(self, fp: ObjectRef, prop: Property) -> None:
        """Create and register a property."""
        if bool(prop.binding) and prop.binding in self.properties:
            msg = f'Binding "{self.cls.__name__}.{prop.binding}" already exists'
            raise NameError(msg)

        if prop.name in self.property_lookup:
            msg = f'Property "{self.cls.__name__}.Object.{prop.name}" already exists'
            raise NameError(msg)

        if prop.name in fp.PropertiesList:
            msg = f'Property "{self.cls.__name__}.Object.{prop.name}" already exists'
            raise NameError(msg)

        prop.create(fp)

    # ──────────
    def apply_extensions(
        self,
        proxy: Proxy,
        obj: ObjectRef,
        method_name: str,
        *args: tuple,
        **kwargs: dict,
    ) -> None:
        """Call extensions runtime lifecycle."""
        for ext in self.extensions:
            method = getattr(ext, method_name)
            method(proxy, obj, self, *args, **kwargs)

    # ──────────
    def apply_extensions_on_class(self) -> None:
        """Call extensions static lifecycle."""
        for ext in self.extensions:
            ext.on_class(self)

    # ──────────
    def add_version_prop(self, obj: ObjectRef) -> None:
        """Inject the fpo version into the proxy."""
        if _SO_VERSION not in obj.PropertiesList:
            obj.addProperty(
                "App::PropertyInteger",
                _SO_VERSION,
                "SO",
                "Internal Scripted Object Version",
                PropertyMode.Hidden.value,
            )
        setattr(obj, _SO_VERSION, self.version)

    # ──────────
    def ensure_properties(self, _proxy: Proxy, obj: ObjectRef) -> None:
        """
        Add missing properties to the ObjectRef.

        Add new properties on start if the class have declared new
        properties since the file was saved.
        """
        for prop in self.properties.values():
            prop.create(obj)

    # ──────────
    def init_properties(self, proxy: Proxy, obj: ObjectRef) -> None:
        for prop in self.properties.values():
            prop.create(obj)
            if prop.default is not None:
                proxy.__so_old__[prop.name] = prop.default

    # ──────────
    def init_display_modes(self, proxy: ViewProxy, obj: ViewProviderDocumentObject) -> None:
        from pivy import coin  # type: ignore !! Lazy Load !!

        if len(self.display_modes) > 0:
            for dm in self.display_modes.values():
                dm_obj = None
                if builder := dm._builder_func:  # noqa: SLF001
                    dm_obj = builder(proxy, obj)
                if not dm_obj:
                    dm_obj = coin.SoGroup()
                obj.addDisplayMode(dm_obj, dm.name)


##% ────────────────────────────────────────────────────────────────────────────
class UIValidator(Protocol):
    """Validator for preferences."""

    def setup(self, ui: Any) -> None: ...
    def validate(self, value: Any) -> str | None: ...


@dataclass
class PreferencePreset:
    """Preference variant under a preset name."""

    preference: Preference
    preset: str

    def __post_init__(self) -> None:
        """Sanitize preset name."""
        if "/" in self.preset:
            self.preset = self.preset.replace("/", "-")

    def __call__(self, *, update: Any | None = None, default: Any = None):  # noqa: ANN204
        return self.preference(update=update, default=default, preset=self.preset)

    def read(self, default: Any = None) -> Any:
        return self.preference.read(preset=self.preset, default=default)

    def write(self, value: bool | float | str | None) -> None:
        self.preference.write(value, preset=self.preset)


@dataclass
class Preference:
    """FreeCAD Preference."""

    group: str
    name: str
    default: Any = None
    value_type: type = None
    root: str = "BaseApp"
    many: bool = False
    label: str | None = None
    description: str | None = None
    unit: str | None = None
    options: dict[str, Hashable] | None = None
    parser: Callable[[str], Any] | None = None
    ui: str | None = None
    ui_group: str | None = None
    ui_page: str | None = None
    ui_section: str | None = None
    ui_exclude: bool = False
    ui_validators: list[UIValidator] | None = None

    # ─────────
    def __post_init__(self) -> None:  # noqa: D105
        if self.value_type is None:
            self.value_type = type(self.default) if self.default is not None else str
        if not self.label:
            self.label = self.name

    # ─────────
    @property
    def group_key(self) -> str:
        return f"User parameter:{self.root}/{self.group}"

    # ─────────
    def read(self, *, preset: str | None = None, default: Any = None) -> Any:
        if preset:
            group = App.ParamGet(f"{self.group_key}/presets/{preset}")
        else:
            group = App.ParamGet(self.group_key)

        try:
            if self.value_type is bool:
                v = group.GetBool(self.name)
                return (default or self.default) if v is None else v
            if self.value_type is int:
                return group.GetInt(self.name) or default or self.default
            if self.value_type is float:
                return group.GetFloat(self.name) or default or self.default
            if self.value_type is str:
                return group.GetString(self.name) or default or self.default
        except Exception:  # noqa: BLE001
            print_err(f"Error reading preference: {self}")
        return default or self.default

    # ─────────
    # Read/Write shortcut
    # Changed in 1.0.0-beta5: for write, update name only arg is required
    def __call__(
        self,
        *,
        update: Any | None = None,
        preset: str | None = None,
        default: Any = None,
    ) -> Any:
        if update is None:
            value = self.read(preset=preset)
            if value is None and default:
                return default
            return value
        self.write(update, preset=preset)
        return None

    # ─────────
    def write(  # noqa: C901, PLR0912
        self,
        value: bool | float | str | None,
        *,
        preset: str | None = None,
    ) -> None:
        if preset:
            group = App.ParamGet(f"{self.group_key}/presets/{preset}")
        else:
            group = App.ParamGet(self.group_key)

        try:
            if self.value_type is bool:
                if value is None:
                    group.RemBool(self.name)
                else:
                    group.SetBool(self.name, self.value_type(value))
            elif self.value_type is int:
                if value is None:
                    group.RemInt(self.name)
                else:
                    group.SetInt(self.name, self.value_type(value))
            elif self.value_type is float:
                if value is None:
                    group.RemFloat(self.name)
                else:
                    group.SetFloat(self.name, self.value_type(value))
            elif self.value_type is str:
                if value is None:
                    group.RemString(self.name)
                else:
                    group.SetString(self.name, self.value_type(value))
        except Exception:  # noqa: BLE001
            print_err(
                f"Error writing preference: {group}/{self.name} {self.value_type} = {value} {type(value)}",  # noqa: E501
            )

    # ─────────
    def preset(self, name: str) -> PreferencePreset:
        """Returns a new Preference based on self but stored in a different preset."""
        return PreferencePreset(self, name)

    # ─────────
    def preset_names(self) -> list[str]:
        group = App.ParamGet(f"{self.group_key}/presets")
        return list(group.GetGroups())

    ##% ─────────
    class ParamObserver:
        listeners: ClassVar[dict] = {}
        ParamGroup: TypeAlias = Any  # Missing type from FreeCAD stubs

        # ─────────
        def __init__(self, group: ParamGroup, callback: Callable) -> None:  # noqa: D107
            self.callback = callback
            group.AttachManager(self)
            Preference.ParamObserver.listeners[hash(self)] = group

        # ─────────
        def slotParamChanged(
            self,
            group: ParamGroup,
            value_type: str,
            name: str,
            value: Any,
        ) -> None:
            self.callback(group, value_type, name, value)

        # ─────────
        def unsubscribe(self) -> None:
            try:
                del Preference.ParamObserver.listeners[hash(self)]
            except Exception:  # noqa: BLE001
                print_err(f"Invalid subscription or already removed: {self.callback.__name__}")

    # ─────────
    @staticmethod
    def subscribe(group: str, root: str = "BaseApp") -> Callable:
        group = f"User parameter:{root}/{group}"

        def wrapper(func: Callable) -> Preference.ParamObserver:
            param_group = App.ParamGet(group)
            return Preference.ParamObserver(param_group, func)

        return wrapper


class Preferences:
    """A set of preferences under a preset name."""

    def __init__(
        self,
        preset: str = "Default",
        copy_from: Preferences | None = None,
    ) -> None:
        """
        Create a preferences preset proxy.

        :param str preset: name of the preset, defaults to "Default"
        :param Preferences | None copy_from: optional name of an existing preset, defaults to None
        """
        self.preset = preset
        for name, pref in self.declared_preferences():
            setattr(self, name, pref.preset(preset))

        if isinstance(copy_from, str):
            copy_from = self.__class__(copy_from)

        if isinstance(copy_from, Preferences):
            copy_from.copy_to(self)

    def copy_to(self, to: Preferences | str) -> Preferences:
        to = self.__class__(to) if isinstance(to, str) else to
        target = to.preset
        for _name, pref in self.declared_preferences():
            pref.write(pref.read(preset=self.preset), preset=target)
        return to

    @classmethod
    def declared_preferences(cls) -> tuple[tuple[str, Preference], ...]:
        if (value := getattr(cls, "_cached_preferences", None)) is not None:
            return value

        value = tuple(inspect.getmembers(cls, _is(Preference)))
        cls._cached_preferences = value
        return value

    def preset_names(self) -> list[str]:
        presets: set[str] = {"Default"}
        update = presets.update
        for _name, pref in self.declared_preferences():
            update(pref.preset_names())
        return sorted(presets)


##@ ┌───────────────────────────────────────────────────────────────────────────┐
##@ │ Decorators                                                                │
##@ └───────────────────────────────────────────────────────────────────────────┘


##@ ────────────────────────────────────────────────────────────────────────────
def proxy(  # noqa: ANN201
    *,
    object_type: str = "App::FeaturePython",
    subtype: str | None = None,
    view_proxy: ViewProxy | None = None,
    extensions: Iterable[str] | None = None,
    view_provider_name_override: str | None = None,
    version: int = 1,
):
    """
    Main decorator for DataProxy creation.

    Decorating a class with @proxy(...) adds support for the fcapi API
    """
    # base dir is useful for relative resource lookup
    base_dir = Path(inspect.stack()[1].filename).parent

    # Ensure object_type suffix
    if not object_type.endswith("Python"):
        object_type = f"{object_type}Python"

    # Actual decorator that applies all the required transformations to the class
    def transformer(cls: type):  # noqa: ANN202
        meta = TypeMeta(
            cls,
            object_type,
            base_dir,
            subtype or cls.__name__,
            view_proxy,
            extensions,
            version,
            view_provider_name_override,
        )
        meta.apply_extensions_on_class()
        t_proxy_constructor(meta)
        t_proxy_set_version(meta)
        t_proxy_object(meta)
        t_proxy_vobject(meta)
        t_proxy_attach(meta)
        t_proxy_create(meta)
        t_proxy_rebind(meta)
        t_proxy_remove(meta)
        t_proxy_restore(meta)
        t_proxy_execute(meta)
        t_proxy_before_change(meta)
        t_proxy_change(meta)
        t_proxy_add_property(meta)
        t_proxy_set_prop_mode(meta)
        t_proxy_set_prop_status(meta)
        t_loads(meta)
        t_dumps(meta)
        t_proxy_view_provider_name_override(meta)
        t_proxy_dirty(meta)
        t_proxy_is_active(meta)
        return cls

    return transformer


##@ ────────────────────────────────────────────────────────────────────────────
def view_proxy(
    *,
    view_provider_name_override: str | None = None,
    extensions: Iterable[str] | None = None,
    icon: str | None = None,
) -> Callable[[type], ViewProxy]:
    """
    Decorator for ViewProxy creation.

    Decorating a class with @view_proxy(...) adds support for the fcapi API.
    """
    # base dir is useful for relative resource lookup
    base_dir = Path(inspect.stack()[1].filename).parent

    # Actual decorator that applies all the required transformations to the class
    def transformer(cls: type) -> ViewProxy:
        meta = TypeMeta(
            cls,
            base_dir=base_dir,
            extensions=extensions,
            view_provider_name_override=view_provider_name_override,
            icon=icon,
        )
        meta.apply_extensions_on_class()
        t_proxy_add_property(meta)
        t_proxy_set_prop_mode(meta)
        t_proxy_set_prop_status(meta)
        t_loads(meta)
        t_dumps(meta)
        t_view_proxy_constructor(meta)
        t_view_proxy_object(meta)
        t_view_proxy_vobject(meta)
        t_view_proxy_attach(meta)
        t_view_proxy_ctx_menu(meta)
        t_view_proxy_claim_children(meta)
        t_view_proxy_change(meta)
        t_view_proxy_delete(meta)
        t_view_proxy_dbl_click(meta)
        t_view_proxy_edit_start(meta)
        t_view_proxy_edit_end(meta)
        t_view_proxy_icon(meta)
        t_view_proxy_set_dm(meta)
        t_view_proxy_get_dms(meta)
        t_view_proxy_get_def_dm(meta)
        t_view_proxy_object_change(meta)

        # Drag and Drop support:
        t_view_proxy_can_drag(meta)
        t_view_proxy_can_drop(meta)

        # https://github.com/FreeCAD/FreeCAD/blob/d5b90e50af6a758af748179f289bb8f09e357266/src/Mod/Part/BOPTools/SplitFeatures.py#L129

        _t_forward(meta.cls, "canDragObjects", _CAN_DRAG_OBJECTS)
        _t_forward(meta.cls, "canDropObjects", _CAN_DROP_OBJECTS)
        _t_forward(meta.cls, "dragObject", _ON_DRAG_OBJECT)
        _t_forward(meta.cls, "dropObject", _ON_DROP_OBJECT)

        return cls

    return transformer


##@ Internal decorator to create class attribute templates
##@ ────────────────────────────────────────────────────────────────────────────
def template(
    *,
    name: str,
    aliases: Iterable[str] | None = None,
    allow_override: bool = False,
    override_error_msg: str | None = None,
) -> Callable[[Callable], _Template]:
    """
    Decorator to crearte a method builder.

    :param str name: Name of the method to be created
    :param Iterable aliases: Aliases of the created method, default=None
    :param bool allow_override: if False, user pre-defined methods will be rejected, default=False
    :param str override_error_msg: Message raised if there are name collisions, default=None
    """

    def wrapper(func: Callable) -> _Template:
        return _Template(
            name,
            aliases,
            func,
            allow_override,
            override_error_msg,
        )

    return wrapper


# Method builders must return a Callable or None
GeneratedMethod: TypeAlias = property | Callable | None

##$ ┌───────────────────────────────────────────────────────────────────────────┐
##$ │ DataProxy Templates                                                       │
##$ └───────────────────────────────────────────────────────────────────────────┘


# ──────────────────────────────────────────────────────────────────────────────
def _set_view_proxy(obj: DocumentObject, view_proxy: ViewProxy) -> None:
    if (vo := getattr(obj, "ViewObject", None)) and hasattr(vo, "Proxy"):
        vo.Proxy = view_proxy if view_proxy is not None else 0


# ──────────────────────────────────────────────────────────────────────────────
def _event_handler(cls: type, name: str) -> Callable[[Proxy, Any], Any] | None:
    """
    Return a valid callable to invoke the user defined handler.

    Removes the event argument depending on user signature.

    :param type cls: Proxy class
    :param str name: method name
    :return Callable[[Proxy, Any], Any] | None: event handler method
    """
    if handler := getattr(cls, name, None):
        if (arity := _m_arity(handler)) == 1:
            return handler

        if arity != 0:
            msg = f"Unsupported signature for method {name}"
            raise TypeError(msg)

        def wrapper(self: Any, _) -> Any:
            return handler(self)

        return wrapper
    return None


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="__init__", allow_override=True)
def t_proxy_constructor(overridden: Any, meta: TypeMeta) -> GeneratedMethod:
    """
    Create a constructor for the DataProxy class.

    Calls the user defined constructor if exists
    """

    def __init__(self: Any, *args: tuple, **kwargs: dict) -> None:
        self.__so_ref__ = None
        self.__so_old__ = {}
        self.Type = meta.subtype
        self.__so_state__ = FeatureState.Attaching
        if overridden:
            overridden(self, *args, **kwargs)

    if overridden:
        __init__.__doc__ = overridden.__doc__
        __init__.__signature__ = inspect.signature(overridden)
    return __init__


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="unsetupObject", override_error_msg=f"Use {_ON_REMOVE} instead.")
def t_proxy_remove(_, meta: TypeMeta) -> GeneratedMethod:
    if handler := _event_handler(meta.cls, _ON_REMOVE):

        def unsetupObject(self: Any, obj: DocumentObject) -> None:
            return handler(self, events.RemoveEvent(obj))

        return unsetupObject
    return None


##$ ────────────────────────────────────────────────────────────────────────────
@template(name=_SET_VERSION)
def t_proxy_set_version(*_args) -> GeneratedMethod:
    def set_version(self: Any, version: int) -> None:
        """Update the Object version."""
        if not isinstance(version, int):
            msg = f"{type(self).__qualname__}.version must be an integer"
            raise TypeError(msg)
        if hasattr(self, _SO_REF):
            setattr(self.__so_ref__, _SO_VERSION, version)
        else:
            msg = f"Current class {type(self).__qualname__} is not a proxy"
            raise TypeError(msg)

    return set_version


##$ ────────────────────────────────────────────────────────────────────────────
@template(name=_OBJECT)
def t_proxy_object(*_args) -> GeneratedMethod:
    def Object(self: Any) -> DocumentObject:
        """Return FreeCAD internal Feature Object."""
        return self.__so_ref__

    return property(Object)


##$ ────────────────────────────────────────────────────────────────────────────
@template(name=_VIEW_OBJECT)
def t_proxy_vobject(*_args) -> GeneratedMethod:
    def ViewObject(self: Any) -> ViewProviderDocumentObject:
        """Return FreeCAD internal View Provider Object."""
        return self.Object.ViewObject

    return property(ViewObject)


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="attach", override_error_msg=f"Use {_ON_ATTACH} instead")
def t_proxy_attach(_, meta: TypeMeta) -> GeneratedMethod:
    def attach(self: Any, obj: DocumentObject) -> None:
        self.__so_ref__ = obj
        self.__so_old__ = {}
        self.Type = meta.subtype

        meta.apply_extensions(self, obj, _ON_ATTACH)
        if handler := _event_handler(meta.cls, _ON_ATTACH):
            handler(self, events.AttachEvent(obj, getattr(obj, "ViewObject", None)))

        self.__so_state__ = FeatureState.Creating

        meta.add_version_prop(obj)
        meta.init_properties(self, obj)

        meta.apply_extensions(self, obj, _ON_CREATE)
        if handler := _event_handler(meta.cls, _ON_CREATE):
            handler(self, events.CreateEvent(obj))

        self.__so_state__ = FeatureState.Created

        meta.apply_extensions(self, obj, _ON_START)
        if handler := _event_handler(meta.cls, _ON_START):
            handler(self, events.StartEvent(obj, getattr(obj, "ViewObject", None)))

        self.__so_state__ = FeatureState.Active

    return attach


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="create")
def t_proxy_create(_, meta: TypeMeta) -> GeneratedMethod:
    def create(
        name: str | None = None,
        label: str | None = None,
        doc: Document | None = None,
    ) -> DocumentObject:
        """Create the FreeCAD Objects, the Python Proxies and bind them."""
        _doc = doc or App.activeDocument() or App.newDocument()
        _name = name or meta.subtype
        proxy = meta.cls()

        view_proxy = None
        if meta.view_proxy and App.GuiUp:
            view_proxy = meta.view_proxy(None)

        obj = _doc.addObject(meta.object_type, _name, proxy, view_proxy, attach=True)
        _set_view_proxy(obj, view_proxy)
        obj.Label = label or _name

        return obj

    return staticmethod(create)


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="rebind")
def t_proxy_rebind(_, meta: TypeMeta) -> GeneratedMethod:
    def rebind(obj: DocumentObject) -> DataProxy:
        """Recreate the proxy objects and rebind them to FreeCAD objects."""
        proxy = meta.cls()
        proxy.__so_ref__ = obj
        obj.Proxy = proxy
        _call(proxy, "attach", obj)
        view_proxy = None
        if meta.view_proxy and App.GuiUp and hasattr(obj, "ViewObject"):
            view_proxy = meta.view_proxy()
            view_proxy.__so_ref__ = obj.ViewObject
        _set_view_proxy(obj, view_proxy)
        return proxy

    return staticmethod(rebind)


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="onDocumentRestored", override_error_msg=f"Use {_ON_RESTORE} instead")
def t_proxy_restore(_, meta: TypeMeta) -> GeneratedMethod:
    def onDocumentRestored(self: Any, obj: DocumentObject) -> None:
        self.__so_state__ = FeatureState.Restoring
        self.__so_ref__ = obj
        self.__so_old__ = {}
        obj.Proxy = self

        # Restore last values
        for prop_name, prop in meta.property_lookup.items():
            self.__so_old__[prop_name] = prop.read(obj)

        meta.apply_extensions(self, obj, _ON_RESTORE)
        if handler := _event_handler(meta.cls, _ON_RESTORE):
            handler(self, events.DocumentRestoredEvent(obj))

        self.__so_state__ = FeatureState.Restored

        meta.apply_extensions(self, obj, _ON_START)
        meta.ensure_properties(self, obj)
        if handler := _event_handler(meta.cls, _ON_START):
            handler(self, events.StartEvent(obj, getattr(obj, "ViewObject", None)))

        self.__so_state__ = FeatureState.Active

    return onDocumentRestored


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="execute", override_error_msg=f"Use {_ON_EXECUTE} instead")
def t_proxy_execute(_, meta: TypeMeta) -> GeneratedMethod:
    handler = _event_handler(meta.cls, _ON_EXECUTE)

    def execute(self: Any, obj: DocumentObject) -> None:
        meta.apply_extensions(self, obj, _ON_EXECUTE)
        if handler:
            handler(self, events.ExecuteEvent(obj))

    return execute


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="onBeforeChange", override_error_msg=f"Use {_ON_BEFORE_CHANGE} instead")
def t_proxy_before_change(_, meta: TypeMeta) -> GeneratedMethod:
    handler = _event_handler(meta.cls, _ON_BEFORE_CHANGE)

    def onBeforeChange(self: Any, obj: DocumentObject, prop_name: str) -> None:
        if getattr(self, "__so_state__", None) == FeatureState.Active:
            old_value = getattr(obj, prop_name, None)
            self.__so_old__[prop_name] = old_value
            if handler:
                handler(
                    self,
                    events.PropertyWillChangeEvent(
                        obj,
                        prop_name,
                        old_value,
                        getattr(obj, "ViewObject", None),
                    ),
                )

    return onBeforeChange


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="onChanged", override_error_msg=f"Use {_ON_CHANGE} instead")
def t_proxy_change(_, meta: TypeMeta) -> GeneratedMethod:
    def onChanged(self: Any, obj: DocumentObject, prop_name: str) -> None:
        if getattr(self, "__so_state__", None) == FeatureState.Active:
            new_value = getattr(obj, prop_name)
            old_value = self.__so_old__.get(prop_name, None)
            if new_value != old_value:
                meta.apply_extensions(self, obj, _ON_CHANGE, prop_name)
                prop = meta.property_lookup.get(prop_name, None)
                event = events.PropertyChangedEvent(
                    obj,
                    prop_name,
                    old_value,
                    new_value,
                    getattr(obj, "ViewObject", None),
                )
                if prop and prop.observer_func:
                    args = (self, event)[0 : prop.observer_arity]
                    prop.observer_func(*args)
                if handler := _event_handler(meta.cls, _ON_CHANGE):
                    handler(self, event)

    return onChanged


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="add_property")
def t_proxy_add_property(_, meta: TypeMeta) -> GeneratedMethod:
    def add_property(self: Any, prop: Property) -> None:
        meta.add_property(self.__so_ref__, prop)

    return add_property


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="set_property_mode")
def t_proxy_set_prop_mode(_, meta: TypeMeta) -> GeneratedMethod:
    def set_property_mode(
        self: Any,
        *,
        mode: PropertyEditorMode,
        name: str | None = None,
        binding: str | None = None,
    ) -> None:
        if not (name or binding):
            msg = "name or binding argument must be provided"
            raise NameError(msg)
        if name and binding:
            msg = "name and binding arguments are mutually exclusive"
            raise NameError(msg)
        if binding:
            prop = meta.properties.get(binding, None)
            if prop is None:
                msg = f"There is not property bound to name {binding}"
                raise NameError(msg)
            self.__so_ref__.setPropertyMode(prop.name, mode)
        else:
            self.__so_ref__.setPropertyMode(name, mode)

    return set_property_mode


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="set_property_status")
def t_proxy_set_prop_status(_, meta: TypeMeta) -> GeneratedMethod:
    def handler(
        self: Any,
        *,
        status: str,
        name: str | None = None,
        binding: str | None = None,
    ) -> None:
        if not (name or binding):
            msg = "name or binding argument must be provided"
            raise NameError(msg)
        if name and binding:
            msg = "name and binding arguments are mutually exclusive"
            raise NameError(msg)
        if binding:
            prop = meta.properties.get(binding, None)
            if prop is None:
                msg = f"There is not property bound to name {binding}"
                raise NameError(msg)
            self.__so_ref__.setPropertyStatus(prop.name, status)
        else:
            self.__so_ref__.setPropertyStatus(name, status)

    return handler


##$ ────────────────────────────────────────────────────────────────────────────
@template(
    name="dumps",
    override_error_msg=f"Use {_ON_SERIALIZE} instead",
)
def t_dumps(_, meta: TypeMeta) -> GeneratedMethod:
    def dumps(self: Any) -> dict[str, Any]:
        state = {}
        if hasattr(self, "Type"):
            state["_Type"] = self.Type
        if handler := _event_handler(meta.cls, _ON_SERIALIZE):
            handler(self, events.SerializeEvent(self.Object, state, self.ViewObject))
        return state

    return dumps


##$ ────────────────────────────────────────────────────────────────────────────
@template(
    name="loads",
    override_error_msg=f"Use {_ON_DESERIALIZE} instead",
)
def t_loads(_, meta: TypeMeta) -> GeneratedMethod:
    def loads(self: Any, state: dict[str, Any]) -> None:
        if isinstance(state, dict) and (_type := state.get("_Type")):
            self.Type = _type
        if handler := _event_handler(meta.cls, _ON_DESERIALIZE):
            handler(self, events.DeserializeEvent(self.Object, state, self.ViewObject))

    return loads


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="mustExecute", override_error_msg=f"Use {_IS_DIRTY} instead")
def t_proxy_dirty(_, meta: TypeMeta) -> GeneratedMethod:
    if handler := _event_handler(meta.cls, _IS_DIRTY):

        def mustExecute(self: Any, _obj: DocumentObject) -> bool:
            return handler(self, None)

        return mustExecute
    return None


##$ ────────────────────────────────────────────────────────────────────────────
@template(
    name="getViewProviderName",
    override_error_msg="Set the `view_provider_name_override` argument in the decorator instead",
)
def t_proxy_view_provider_name_override(_, meta: TypeMeta) -> GeneratedMethod:
    if meta.view_provider_name_override:

        def getViewProviderName(_self: Any, _obj: DocumentObject) -> str | None:
            return meta.view_provider_name_override

        return getViewProviderName

    view_meta = None
    if meta.view_proxy and hasattr(meta.view_proxy, _SO_META):
        view_meta = meta.view_proxy.__so_meta__

    if view_meta and view_meta.view_provider_name_override:

        def getViewProviderName(_self: Any, _obj: DocumentObject) -> str | None:
            return view_meta.view_provider_name_override

        return getViewProviderName
    return None


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="is_active", allow_override=True)
def t_proxy_is_active(overridden: Any, _: TypeMeta) -> GeneratedMethod:
    if overridden:
        return overridden

    def is_active(self: Any) -> bool:
        return getattr(self, "__so_state__", None) == FeatureState.Active
    return is_active


##$ ┌───────────────────────────────────────────────────────────────────────────┐
##$ │ ViewProxy Templates                                                       │
##$ └───────────────────────────────────────────────────────────────────────────┘


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="__init__", allow_override=True)
def t_view_proxy_constructor(overridden: Any, _: TypeMeta) -> GeneratedMethod:
    """
    Create the constructor for the ViewProxy.

    Calls the user defined constructor if any.
    If there is a user defined __init__, it must
    accept the ViewObject as it first argument_

    ```python
        def __init__(self, vp, ...)
    ```
    """

    def __init__(
        self: Any,
        vp: ViewProviderDocumentObject,
        /,
        *args: tuple,
        **kwargs: dict,
    ) -> None:
        self.__so_ref__ = vp
        if vp:
            vp.Proxy = self
        if overridden:
            if object.__init__ is overridden:
                overridden(self)
            else:
                overridden(self, vp, *args, **kwargs)

    if overridden:
        __init__.__doc__ = overridden.__doc__
        __init__.__signature__ = inspect.signature(overridden)
    return __init__


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="attach", override_error_msg=f"Use {_ON_ATTACH} instead")
def t_view_proxy_attach(_: Any, meta: TypeMeta) -> GeneratedMethod:
    """Create attach method."""

    def attach(self: Any, vp: ViewProviderDocumentObject) -> None:
        self.__so_ref__ = vp
        self.__so_old__ = {}
        if handler := _event_handler(meta.cls, _ON_ATTACH):
            handler(self, events.AttachEvent(vp.Object, vp))
        meta.init_properties(self, vp)
        meta.init_display_modes(self, vp)
        meta.apply_extensions(self, vp, _ON_ATTACH)
        if handler := _event_handler(meta.cls, _ON_START):
            handler(self, events.StartEvent(vp.Object, vp))

    return attach


##$ ────────────────────────────────────────────────────────────────────────────
@template(
    name="setupContextMenu",
    override_error_msg=f"Use {_ON_CONTEXT_MENU} instead",
)
def t_view_proxy_ctx_menu(_: Any, meta: TypeMeta) -> GeneratedMethod:
    """Create setupContextMenu method."""
    if handler := _event_handler(meta.cls, _ON_CONTEXT_MENU):

        def setupContextMenu(self: Any, vp: ViewProviderDocumentObject, menu: QtGui.QMenu) -> None:
            handler(self, events.ContextMenuEvent(vp.Object, vp, menu))

        return setupContextMenu
    return None


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="claimChildren", override_error_msg=f"Use {_ON_CLAIM_CHILDREN} instead")
def t_view_proxy_claim_children(_: Any, meta: TypeMeta) -> GeneratedMethod:
    """Create claimChildren method."""
    if handler := _event_handler(meta.cls, _ON_CLAIM_CHILDREN):

        def claimChildren(self: Any) -> list[DocumentObject]:
            return handler(self, events.ClaimChildrenEvent(self.Object, self.ViewObject)) or []

        return claimChildren
    return None


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="setEdit", override_error_msg=f"Use {_ON_EDIT_START} instead")
def t_view_proxy_edit_start(_: Any, meta: TypeMeta) -> GeneratedMethod:
    """Create setEdit method."""
    if handler := _event_handler(meta.cls, _ON_EDIT_START):

        def setEdit(self: Any, vp: ViewProviderDocumentObject, mode: int = 0) -> bool | None:
            return handler(self, events.EditStartEvent(vp.Object, vp, mode))

        return setEdit
    return None


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="unsetEdit", override_error_msg=f"Use {_ON_EDIT_END} instead")
def t_view_proxy_edit_end(_: Any, meta: TypeMeta) -> GeneratedMethod:
    """Create unsetEdit method."""
    if handler := _event_handler(meta.cls, _ON_EDIT_END):

        def unsetEdit(self: Any, vp: ViewProviderDocumentObject, mode: int = 0) -> bool | None:
            return handler(self, events.EditEndEvent(vp.Object, vp, mode))

        return unsetEdit
    return None


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="canDragObject", override_error_msg=f"Use {_CAN_DRAG_OBJECT} instead")
def t_view_proxy_can_drag(_: Any, meta: TypeMeta) -> GeneratedMethod:
    """Create canDragObject method."""
    if handler := _event_handler(meta.cls, _CAN_DRAG_OBJECT):

        def canDragObject(self: Any, obj: DocumentObject) -> bool:
            return bool(handler(self, events.DragAndDropEvent(self.Object, self.ViewObject, obj)))

        return canDragObject
    return None


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="canDropObject", override_error_msg=f"Use {_CAN_DROP_OBJECT} instead")
def t_view_proxy_can_drop(_: Any, meta: TypeMeta) -> GeneratedMethod:
    """Create canDropObject method."""
    if handler := _event_handler(meta.cls, _CAN_DROP_OBJECT):

        def canDropObject(self: Any, obj: DocumentObject) -> bool:
            return bool(handler(self, events.DragAndDropEvent(self.Object, self.ViewObject, obj)))

        return canDropObject
    return None


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="onDelete", override_error_msg=f"Use {_ON_DELETE} instead")
def t_view_proxy_delete(_, meta: TypeMeta) -> GeneratedMethod:
    if handler := _event_handler(meta.cls, _ON_DELETE):

        def onDelete(self: Any, vp: ViewProviderDocumentObject, sub_elements: list[str]) -> bool:
            return bool(handler(self, events.DeleteEvent(vp.Object, vp, sub_elements)))

        return onDelete
    return None


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="doubleClicked", override_error_msg=f"Use {_ON_DBL_CLICK} instead")
def t_view_proxy_dbl_click(_, meta: TypeMeta) -> GeneratedMethod:
    if handler := _event_handler(meta.cls, _ON_DBL_CLICK):

        def doubleClicked(self: Any, vp: ViewProviderDocumentObject) -> bool:
            return bool(handler(self, events.DoubleClickEvent(vp.Object, vp)))

        return doubleClicked
    return None


##$ ────────────────────────────────────────────────────────────────────────────
# TODO(mnesarco): Add support for onBeforeChange old+new
# https://github.com/mnesarco/fcapi/issues/15
@template(name="onChanged", override_error_msg=f"Use {_ON_CHANGE} instead")
def t_view_proxy_change(_, meta: TypeMeta) -> GeneratedMethod:
    def onChanged(self: Any, vp: ViewProviderDocumentObject, prop_name: str) -> None:
        data_proxy = getattr(vp.Object, "Proxy", None)
        if getattr(data_proxy, "__so_state__", None) != FeatureState.Active:
            return

        new_value = getattr(vp, prop_name, None)
        meta.apply_extensions(self, vp, _ON_CHANGE, prop_name)
        prop = meta.property_lookup.get(prop_name, None)
        event = events.PropertyChangedEvent(vp.Object, prop_name, None, new_value, vp)

        if prop and prop.observer_func:
            args = (self, event)[0 : prop.observer_arity]
            prop.observer_func(*args)

        if handler := _event_handler(meta.cls, _ON_CHANGE):
            handler(self, event)

    return onChanged


##$ ────────────────────────────────────────────────────────────────────────────
@template(
    name="getIcon",
    override_error_msg=f"Use {_GET_ICON} instead or pass the icon argument in the decorator",
)
def t_view_proxy_icon(_, meta: TypeMeta) -> GeneratedMethod:
    get_icon = getattr(meta.cls, _GET_ICON, None)
    if get_icon or meta.icon:

        def getIcon(self: Any) -> str | None:
            if get_icon:
                return _resolve_uri(get_icon(self), meta.base_dir)
            if meta.icon:
                return _resolve_uri(meta.icon, meta.base_dir)
            return None

        return getIcon
    return None


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="setDisplayMode", override_error_msg=f"Use {_SET_DISPLAY_MODE} instead")
def t_view_proxy_set_dm(_, meta: TypeMeta) -> GeneratedMethod:
    get = getattr(meta.cls, _SET_DISPLAY_MODE, None)

    def setDisplayMode(self: Any, mode: str) -> str:
        if get:
            return get(self, mode)
        return mode

    return setDisplayMode


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="getDisplayModes", override_error_msg=f"Use {_DISPLAY_MODES} instead")
def t_view_proxy_get_dms(_, meta: TypeMeta) -> GeneratedMethod:
    if meta_dm := getattr(meta.cls, _DISPLAY_MODES, None):

        def getDisplayModes(self: Any, _vp: ViewProviderDocumentObject) -> list[str]:
            return meta_dm(self)

    else:

        def getDisplayModes(_self: Any, _vp: ViewProviderDocumentObject) -> list[str]:
            return [dm.name for dm in meta.display_modes.values()]

    return getDisplayModes


##$ ────────────────────────────────────────────────────────────────────────────
@template(
    name="getDefaultDisplayMode",
    override_error_msg=f"Use {_DEFAULT_DISPLAY_MODE} instead",
)
def t_view_proxy_get_def_dm(_, meta: TypeMeta) -> GeneratedMethod:
    if handler := getattr(meta.cls, _DEFAULT_DISPLAY_MODE, None):

        def getDefaultDisplayMode(self: Any) -> str:
            return handler(self)

        return getDefaultDisplayMode

    if not meta.display_modes:
        return None

    defaults = [dm.name for dm in meta.display_modes.values() if dm.is_default]

    if len(defaults) == 0:
        if len(meta.display_modes) == 1:
            defaults = [dm.name for dm in meta.display_modes.values()]
        else:
            modes = [dm.name for dm in meta.display_modes.values()]
            msg = f"One display mode must be declared as default: {modes}"
            raise RuntimeError(msg)

    if len(defaults) > 1:
        msg = f"Only one display mode can be declared as default: {defaults}"
        raise RuntimeError(msg)

    def getDefaultDisplayMode(_self: Any) -> str:
        return defaults[0]

    return getDefaultDisplayMode


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="updateData", override_error_msg=f"Use {_ON_OBJECT_CHANGE} instead")
def t_view_proxy_object_change(_, meta: TypeMeta) -> GeneratedMethod:
    if handler := _event_handler(meta.cls, _ON_OBJECT_CHANGE):

        def updateData(self: Any, obj: DocumentObject, prop_name: str) -> None:
            handler(self, events.DataChangedEvent(obj, obj.ViewObject, prop_name))

        return updateData
    return None


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="Object")
def t_view_proxy_object(*_args) -> GeneratedMethod:
    def Object(self: Any) -> DocumentObject:
        """Return FreeCAD internal Feature Object."""
        return self.__so_ref__.Object

    return property(Object)


##$ ────────────────────────────────────────────────────────────────────────────
@template(name="ViewObject")
def t_view_proxy_vobject(*_) -> GeneratedMethod:
    def ViewObject(self: Any) -> ViewProviderDocumentObject:
        """Return FreeCAD internal View Provider Object."""
        return self.__so_ref__

    return property(ViewObject)


# ; ┌───────────────────────────────────────────────────────────────────────────┐
# ; │ Migrations                                                                │
# ; └───────────────────────────────────────────────────────────────────────────┘


##% ────────────────────────────────────────────────────────────────────────────
class MigrationMeta:
    """Migrations Meta data."""

    def __init__(self, old: type, current: type) -> None:
        """
        Migration metadata.

        :param type old: proxy class to migrate from
        :param type current: proxy class to migrate to, can be the same as old
        """
        self.old = old
        self.current = current
        self.inplace = old is current

        overridden = getattr(old, "onDocumentRestored", None)
        old.onDocumentRestored = t_migrations_onDocumentRestored(overridden, self)


##@ ────────────────────────────────────────────────────────────────────────────
def migrations(current: type | None = None) -> Callable[[type], type]:
    """
    Install migrations management into the class.

    :param ProxyClass current: most recent class, if omitted, the same class is used.
    """
    T = TypeVar("T")

    def wrapper(cls: T) -> T:
        cls._fp_migrations = MigrationMeta(cls, current or cls)
        return cls

    return wrapper


##$ ────────────────────────────────────────────────────────────────────────────
def t_migrations_onDocumentRestored(
    overridden: Callable | None,
    meta: MigrationMeta,
) -> GeneratedMethod:
    """Proxy.onDocumentRestored override used for migrations."""
    current_version = 1
    proxy_meta: TypeMeta = getattr(meta.current, _SO_META, None)
    if proxy_meta:
        current_version = proxy_meta.version

    def onDocumentRestored(self: Any, obj: DocumentObject) -> None:
        self.__so_state__ = FeatureState.Restoring
        self.__so_ref__ = obj
        version = getattr(obj, _SO_VERSION, 1)
        event = events.MigrationEvent(obj, version, current_version)

        if not meta.inplace:
            _migrate_class(self, meta, event)
        elif version < current_version:
            _migrate_upgrade(self, meta, event)
        elif version > current_version:
            _migrate_downgrade(self, meta, event)

        if overridden:
            overridden(self, obj)

    return onDocumentRestored


# Case: migration to a different class
# ──────────────────────────────────────────────────────────────────────────────
def _migrate_class(
    self: Any,
    meta: MigrationMeta,
    event: events.MigrationEvent,
) -> None:
    old_v_id = f"{meta.old.__name__}:{event.from_version}"
    new_v_id = f"{meta.current.__name__}:{event.to_version}"
    print_log(
        f"Document contains a different version of {old_v_id}. ",
        f"Current version is {new_v_id}",
    )
    if hasattr(self, _ON_MIGRATE_CLASS):
        _try_migration(
            self,
            meta,
            event,
            self.on_migrate_class,
        )


# Case: migration to a higher version
# ──────────────────────────────────────────────────────────────────────────────
def _migrate_upgrade(
    self: Any,
    meta: MigrationMeta,
    event: events.MigrationEvent,
) -> None:
    old_v_id = f"{meta.current.__name__}:{event.from_version}"
    print_log(
        f"Document contains an older version of {old_v_id}. ",
        f"Current version is {event.to_version}",
    )
    if hasattr(self, _ON_MIGRATE_UP):
        _try_migration(
            self,
            meta,
            event,
            self.on_migrate_upgrade,
        )


# Case: migration to a lower version
# ──────────────────────────────────────────────────────────────────────────────
def _migrate_downgrade(
    self: Any,
    meta: MigrationMeta,
    event: events.MigrationEvent,
) -> None:
    old_v_id = f"{meta.current.__name__}:{event.from_version}"
    print_log(
        f"Document contains a newer version of {old_v_id}. ",
        f"Current version is {event.to_version}",
    )
    if hasattr(self, _ON_MIGRATE_DOWN):
        _try_migration(
            self,
            meta,
            event,
            self.on_migrate_downgrade,
        )


# Apply migration logic and report error/status
# ──────────────────────────────────────────────────────────────────────────────
def _try_migration(
    self: Any,
    meta: MigrationMeta,
    event: events.MigrationEvent,
    action: Callable[[events.MigrationEvent], None],
) -> None:
    old_v_id = f"{meta.old.__name__}:{event.from_version}"
    new_v_id = f"{meta.current.__name__}:{event.to_version}"
    message = f"from {old_v_id} to {new_v_id}"
    print_log(f"A migration handler was found, attempting to migrate {message}")
    try:
        action(event)
        new_version = getattr(event.source, _SO_VERSION, event.from_version)
        if new_version != event.to_version or not isinstance(event.source.Proxy, meta.current):
            print_err(f"!!! Failed to migrate {message}")
            if hasattr(self, _ON_MIGRATE_ERROR):
                self.on_migrate_error(event)
        else:
            print_log(f"Migration completed {message}")
            if hasattr(self, _ON_MIGRATE_COMPLETE):
                self.on_migrate_complete(event)
    except Exception:  # noqa: BLE001
        print_err(f"!!! Failed to migrate {message}")
        traceback.print_exc(file=sys.stderr)
        if hasattr(self, _ON_MIGRATE_ERROR):
            self.on_migrate_error(event)


##: ┌───────────────────────────────────────────────────────────────────────────┐
##: │ Dynamic module setup                                                      │
##: └───────────────────────────────────────────────────────────────────────────┘

##: Install Extension Managers
##: ────────────────────────────────────────────────────────────────────────────
Part_AttachExtensionPython("Part::AttachExtensionPython")
App_LinkBaseExtensionPython("App::LinkBaseExtensionPython")
App_LinkExtensionPython("App::LinkExtensionPython")
ExtensionSupport("App::GeoFeatureGroupExtensionPython")
ExtensionSupport("App::GroupExtensionPython")
ExtensionSupport("App::OriginGroupExtensionPython")
ExtensionSupport("App::SuppressibleExtensionPython")
ExtensionSupport("TechDraw::CosmeticExtensionPython")
ExtensionSupport("Gui::ViewProviderExtensionPython")
ExtensionSupport("Gui::ViewProviderGeoFeatureGroupExtensionPython")
ExtensionSupport("Gui::ViewProviderGroupExtensionPython")
ExtensionSupport("Gui::ViewProviderOriginGroupExtensionPython")
ExtensionSupport("PartGui::ViewProviderAttachExtensionPython")
ExtensionSupport("PartGui::ViewProviderSplineExtensionPython")

##: Define constructors for each property type except for Enumeration
##: ────────────────────────────────────────────────────────────────────────────
##: Generated from <FreeCAD_sources>/src/App/Application.cpp
##: Supported Property types
##: ────────────────────────────────────────────────────────────────────────────
PropertyAcceleration = _prop_constructor("App::PropertyAcceleration", float)
PropertyAmountOfSubstance = _prop_constructor("App::PropertyAmountOfSubstance", float)
PropertyAngle = _prop_constructor("App::PropertyAngle", float)
PropertyArea = _prop_constructor("App::PropertyArea", float)
PropertyBool = _prop_constructor("App::PropertyBool", bool)
PropertyBoolList = _prop_constructor("App::PropertyBoolList", list[bool])
PropertyColor = _prop_constructor("App::PropertyColor", tuple[float, float, float])
PropertyColorList = _prop_constructor("App::PropertyColorList", list[tuple[float, float, float]])
PropertyComplexGeoData = _prop_constructor("App::PropertyComplexGeoData", object)
PropertyCompressiveStrength = _prop_constructor("App::PropertyCompressiveStrength", float)
PropertyCurrentDensity = _prop_constructor("App::PropertyCurrentDensity", float)
PropertyDensity = _prop_constructor("App::PropertyDensity", float)
PropertyDirection = _prop_constructor("App::PropertyDirection", App.Vector)
PropertyDissipationRate = _prop_constructor("App::PropertyDissipationRate", float)
PropertyDistance = _prop_constructor("App::PropertyDistance", float)
PropertyDynamicViscosity = _prop_constructor("App::PropertyDynamicViscosity", float)
PropertyElectricCharge = _prop_constructor("App::PropertyElectricCharge", float)
PropertyElectricCurrent = _prop_constructor("App::PropertyElectricCurrent", float)
PropertyElectricPotential = _prop_constructor("App::PropertyElectricPotential", float)
PropertyElectricalCapacitance = _prop_constructor("App::PropertyElectricalCapacitance", float)
PropertyElectricalConductance = _prop_constructor("App::PropertyElectricalConductance", float)
PropertyElectricalConductivity = _prop_constructor("App::PropertyElectricalConductivity", float)
PropertyElectricalInductance = _prop_constructor("App::PropertyElectricalInductance", float)
PropertyElectricalResistance = _prop_constructor("App::PropertyElectricalResistance", float)
PropertyExpressionEngine = _prop_constructor("App::PropertyExpressionEngine", list[tuple[str, str]])
PropertyFile = _prop_constructor("App::PropertyFile", str)
PropertyFileIncluded = _prop_constructor("App::PropertyFileIncluded", str)
PropertyFloat = _prop_constructor("App::PropertyFloat", float)
PropertyFloatConstraint = _prop_constructor("App::PropertyFloatConstraint", float)
PropertyFloatList = _prop_constructor("App::PropertyFloatList", list[float])
PropertyFont = _prop_constructor("App::PropertyFont", object)
PropertyForce = _prop_constructor("App::PropertyForce", float)
PropertyFrequency = _prop_constructor("App::PropertyFrequency", float)
PropertyGeometry = _prop_constructor("App::PropertyGeometry", object)
PropertyHeatFlux = _prop_constructor("App::PropertyHeatFlux", float)
PropertyInteger = _prop_constructor("App::PropertyInteger", int)
PropertyIntegerConstraint = _prop_constructor("App::PropertyIntegerConstraint", int)
PropertyIntegerList = _prop_constructor("App::PropertyIntegerList", list[int])
PropertyIntegerSet = _prop_constructor("App::PropertyIntegerSet", set[int])
PropertyInverseArea = _prop_constructor("App::PropertyInverseArea", float)
PropertyInverseLength = _prop_constructor("App::PropertyInverseLength", float)
PropertyInverseVolume = _prop_constructor("App::PropertyInverseVolume", float)
PropertyKinematicViscosity = _prop_constructor("App::PropertyKinematicViscosity", float)
PropertyLength = _prop_constructor("App::PropertyLength", float)
PropertyLink = _prop_constructor("App::PropertyLink", DocumentObject)
PropertyLinkChild = _prop_constructor("App::PropertyLinkChild", DocumentObject)
PropertyLinkGlobal = _prop_constructor("App::PropertyLinkGlobal", DocumentObject)
PropertyLinkHidden = _prop_constructor("App::PropertyLinkHidden", DocumentObject)
PropertyLinkList = _prop_constructor("App::PropertyLinkList", list[DocumentObject])
PropertyLinkListChild = _prop_constructor("App::PropertyLinkListChild", list[DocumentObject])
PropertyLinkListGlobal = _prop_constructor("App::PropertyLinkListGlobal", list[DocumentObject])
PropertyLinkListHidden = _prop_constructor("App::PropertyLinkListHidden", list[DocumentObject])
PropertyLinkSub = _prop_constructor("App::PropertyLinkSub", tuple[DocumentObject, list[str]])
PropertyLinkSubChild = _prop_constructor(
    "App::PropertyLinkSubChild",
    tuple[DocumentObject, list[str]],
)
PropertyLinkSubGlobal = _prop_constructor(
    "App::PropertyLinkSubGlobal",
    tuple[DocumentObject, list[str]],
)
PropertyLinkSubHidden = _prop_constructor(
    "App::PropertyLinkSubHidden",
    tuple[DocumentObject, list[str]],
)
PropertyLinkSubList = _prop_constructor(
    "App::PropertyLinkSubList",
    list[tuple[DocumentObject, list[str]]],
)
PropertyLinkSubListChild = _prop_constructor(
    "App::PropertyLinkSubListChild",
    list[tuple[DocumentObject, list[str]]],
)
PropertyLinkSubListGlobal = _prop_constructor(
    "App::PropertyLinkSubListGlobal",
    list[tuple[DocumentObject, list[str]]],
)
PropertyLinkSubListHidden = _prop_constructor(
    "App::PropertyLinkSubListHidden",
    list[tuple[DocumentObject, list[str]]],
)
PropertyLuminousIntensity = _prop_constructor("App::PropertyLuminousIntensity", float)
PropertyMagneticFieldStrength = _prop_constructor("App::PropertyMagneticFieldStrength", float)
PropertyMagneticFlux = _prop_constructor("App::PropertyMagneticFlux", float)
PropertyMagneticFluxDensity = _prop_constructor("App::PropertyMagneticFluxDensity", float)
PropertyMagnetization = _prop_constructor("App::PropertyMagnetization", float)
PropertyMap = _prop_constructor("App::PropertyMap", dict)
PropertyMass = _prop_constructor("App::PropertyMass", float)
PropertyMaterial = _prop_constructor("App::PropertyMaterial", object)
PropertyMaterialList = _prop_constructor("App::PropertyMaterialList", list[object])
PropertyMatrix = _prop_constructor("App::PropertyMatrix", App.Matrix)
PropertyMoment = _prop_constructor("App::PropertyMoment", float)
PropertyPath = _prop_constructor("App::PropertyPath", str)
PropertyPercent = _prop_constructor("App::PropertyPercent", float)
PropertyPersistentObject = _prop_constructor("App::PropertyPersistentObject", object)
PropertyPlacement = _prop_constructor("App::PropertyPlacement", App.Placement)
PropertyPlacementLink = _prop_constructor("App::PropertyPlacementLink", App.Placement)
PropertyPlacementList = _prop_constructor("App::PropertyPlacementList", list[App.Placement])
PropertyPosition = _prop_constructor("App::PropertyPosition", App.Vector)
PropertyPower = _prop_constructor("App::PropertyPower", float)
PropertyPrecision = _prop_constructor("App::PropertyPrecision", float)
PropertyPressure = _prop_constructor("App::PropertyPressure", float)
PropertyPythonObject = _prop_constructor("App::PropertyPythonObject", object)
PropertyQuantity = _prop_constructor("App::PropertyQuantity", float)
PropertyQuantityConstraint = _prop_constructor("App::PropertyQuantityConstraint", float)
PropertyRotation = _prop_constructor("App::PropertyRotation", App.Rotation)
PropertyShearModulus = _prop_constructor("App::PropertyShearModulus", float)
PropertySpecificEnergy = _prop_constructor("App::PropertySpecificEnergy", float)
PropertySpecificHeat = _prop_constructor("App::PropertySpecificHeat", float)
PropertySpeed = _prop_constructor("App::PropertySpeed", float)
PropertyStiffness = _prop_constructor("App::PropertyStiffness", float)
PropertyStiffnessDensity = _prop_constructor("App::PropertyStiffnessDensity", float)
PropertyStress = _prop_constructor("App::PropertyStress", float)
PropertyString = _prop_constructor("App::PropertyString", str)
PropertyStringList = _prop_constructor("App::PropertyStringList", list[str])
PropertyTemperature = _prop_constructor("App::PropertyTemperature", float)
PropertyThermalConductivity = _prop_constructor("App::PropertyThermalConductivity", float)
PropertyThermalExpansionCoefficient = _prop_constructor(
    "App::PropertyThermalExpansionCoefficient",
    float,
)
PropertyThermalTransferCoefficient = _prop_constructor(
    "App::PropertyThermalTransferCoefficient",
    float,
)
PropertyTime = _prop_constructor("App::PropertyTime", float)
PropertyUUID = _prop_constructor("App::PropertyUUID", str)
PropertyUltimateTensileStrength = _prop_constructor("App::PropertyUltimateTensileStrength", float)
PropertyVacuumPermittivity = _prop_constructor("App::PropertyVacuumPermittivity", float)
PropertyVector = _prop_constructor("App::PropertyVector", App.Vector)
PropertyVectorDistance = _prop_constructor("App::PropertyVectorDistance", float)
PropertyVectorList = _prop_constructor("App::PropertyVectorList", list[App.Vector])
PropertyVelocity = _prop_constructor("App::PropertyVelocity", float)
PropertyVolume = _prop_constructor("App::PropertyVolume", float)
PropertyVolumeFlowRate = _prop_constructor("App::PropertyVolumeFlowRate", float)
PropertyVolumetricThermalExpansionCoefficient = _prop_constructor(
    "App::PropertyVolumetricThermalExpansionCoefficient",
    float,
)
PropertyWork = _prop_constructor("App::PropertyWork", float)
PropertyXLink = _prop_constructor("App::PropertyXLink", DocumentObject)
PropertyXLinkList = _prop_constructor("App::PropertyXLinkList", DocumentObject)
PropertyXLinkSub = _prop_constructor("App::PropertyXLinkSub", DocumentObject)
PropertyXLinkSubHidden = _prop_constructor("App::PropertyXLinkSubHidden", DocumentObject)
PropertyXLinkSubList = _prop_constructor("App::PropertyXLinkSubList", list[DocumentObject])
PropertyYieldStrength = _prop_constructor("App::PropertyYieldStrength", float)
PropertyYoungsModulus = _prop_constructor("App::PropertyYoungsModulus", float)


##: Special constructor for Enumeration property type
##: ────────────────────────────────────────────────────────────────────────────
_ET = TypeVar("_ET")


def PropertyEnumeration(
    enum: type[_ET],
    *,
    name: str | None = None,
    section: str = "Data",
    default: _ET | None = None,
    description: str = "",
    mode: PropertyMode = PropertyMode.Default,
    observer_func: Callable | None = None,
    link_property: str | None = None,
    meta: bool = False,
) -> Property[_ET] | tuple[Property[_ET], PropertyMeta]:
    """Create Enumeration Property."""
    prop = Property(
        type="App::PropertyEnumeration",
        section=section,
        observer_func=observer_func,
        name=name,
        link_property=link_property,
        default=default,
        description=description,
        mode=mode,
        enum=enum,
    )

    if meta:
        return _property_and_meta(prop)
    return prop


##: Special constructor for Options property type
##: ────────────────────────────────────────────────────────────────────────────
def PropertyOptions(
    options_provider: Callable,
    *,
    name: str | None = None,
    section: str = "Data",
    default: Any = None,
    description: str = "",
    mode: PropertyMode = PropertyMode.Default,
    observer_func: Callable | None = None,
    link_property: str | None = None,
    meta: bool = False,
) -> Property[str] | tuple[Property[str], PropertyMeta]:
    """Create Options Property."""
    prop = Property(
        type="App::PropertyEnumeration",
        section=section,
        observer_func=observer_func,
        name=name,
        link_property=link_property,
        default=default,
        description=description,
        mode=mode,
        options=options_provider,
    )

    if meta:
        return _property_and_meta(prop)
    return prop


# ┌───────────────────────────────────────────────────────────────────────────┐
# │ Minimal set of user utilities                                             │
# └───────────────────────────────────────────────────────────────────────────┘


# ──────────────────────────────────────────────────────────────────────────────
def get_pd_active_body() -> DocumentObject | tuple[DocumentObject, DocumentObject, str] | None:
    """
    Retrieve the active PartDesign Body if any.

    :return PartDesign.Body: Active Body
    """
    if Gui.ActiveDocument and Gui.ActiveDocument.ActiveView:
        return Gui.ActiveDocument.ActiveView.getActiveObject("pdbody")
    return None


# ──────────────────────────────────────────────────────────────────────────────
def set_pd_shape(fp: DocumentObject, shape: Shape) -> None:
    """Prepare the shape for usage in PartDesign and sets `Shape` and `AddSubShape`."""
    # Validate type
    if not fp.TypeId.startswith("PartDesign::"):
        msg = "Object fp is not a PartDesign Feature"
        raise ValueError(msg)

    # Must be attachable
    if not fp.hasExtension("Part::AttachExtensionPython"):
        fp.addExtension("Part::AttachExtensionPython")
        fp.positionBySupport()
    shape.Placement = fp.Placement

    # Manage BaseFeature
    if hasattr(fp, "BaseFeature") and fp.BaseFeature is not None:
        if "Subtractive" in fp.TypeId:
            full_shape = fp.BaseFeature.Shape.cut(shape)
        else:
            full_shape = fp.BaseFeature.Shape.fuse(shape)
        full_shape.transformShape(fp.Placement.inverse().toMatrix(), True)  # noqa: FBT003
        fp.Shape = full_shape
    else:
        fp.Shape = shape

    # Manage Pattern Feature
    if hasattr(fp, "AddSubShape"):
        shape.transformShape(fp.Placement.inverse().toMatrix(), True)  # noqa: FBT003
        fp.AddSubShape = shape


# ──────────────────────────────────────────────────────────────────────────────
def set_immutable_prop(obj: ObjectRef, name: str, value: Any) -> None:
    """
    Force update a property with Immutable status.

    Temporarily removes the immutable flag, sets the value and restore the flag if required.

    :param ObjectRef obj: remote FreeCAD object
    :param str name: property
    :param Any value: the value
    """
    is_immutable = "Immutable" in obj.getPropertyStatus(name)
    if is_immutable:
        obj.setPropertyStatus(name, "-Immutable")
    try:
        setattr(obj, name, value)
    finally:
        if is_immutable:
            obj.setPropertyStatus(name, "Immutable")


# ──────────────────────────────────────────────────────────────────────────────
def get_selection(*args: tuple) -> tuple:
    """
    Return current selection in specific order and matching specific types.

    case 1: no args, just returns selection as list:
    ```python
    sel = get_selection()
    ```

    case 2: args are the required selection types:
    ```python
    ok, axis, obj = get_selection('PartDesign::Line'. '*')
    if ok:
        ...
    ```

    regex are supported for type matching and '*' is a general wildcard:

    ```python
    ok, axis, part, other = get_selection('PartDesign::Line', re.compile('Part::.*'), '*')
    if ok:
        ...
    ```

    this will parse selection for three elements, first one must be a
    `PartDesign::Line`, second one must be any object from the Part namespace,
    the last one can be anything. The user can select them
    in any order but they will be returned in the specified order. Take
    into account that wildcards will match anything so it is better to
    specify patterns from more specific to least specific.

    In this invocation schema, the first element of the returned tuple
    is a boolean that indicate if the selection matches the patterns.

    :param *[str|re.Pattern] *args: List of patterns
    :return List[DocumentObject]: If no arguments are supplied, the list of selected objects
    :return bool, *List[DocumentObject]: If arguments are supplied, the first element returned\
    says if the selection matches the patterns, the rest are the selected objects in order
    """
    n_args = len(args)
    if n_args == 0:
        return Gui.Selection.getSelection()

    selection: list[DocumentObject] = Gui.Selection.getSelection()
    result = [None] * n_args

    def matcher(arg: Any) -> Callable[[str], bool]:
        if arg == "*":
            return lambda _: True
        if isinstance(arg, str):
            return arg.__eq__
        if isinstance(arg, re.Pattern):
            return arg.fullmatch
        if isinstance(arg, (tuple, list)):
            return re.compile("|".join(arg)).fullmatch
        msg = f"Invalid selection pattern: {type(arg)}"
        raise TypeError(msg)

    matchers = tuple(matcher(arg) for arg in args)
    for obj in selection:
        obj_type = obj.TypeId
        for i in range(n_args):
            if result[i] is None and matchers[i](obj_type):
                result[i] = obj
                break

    return all(result), *result


# ──────────────────────────────────────────────────────────────────────────────
def _basic_modal_dlg(message: str, title: str = "Message", details: str = "") -> QtGui.QMessageBox:
    """
    Build a basic Qt dialog box.

    :param str message: summary
    :param str title: box title, defaults to "Message"
    :param str details: expanded message, defaults to None
    :return QtGui.QMessageBox: qt object for further customization
    """
    diag = QtGui.QMessageBox()
    diag.setIcon(QtGui.QMessageBox.Information)
    diag.setWindowTitle(title)
    diag.setText(message)
    if details:
        diag.setInformativeText(details)
    diag.setWindowModality(QtCore.Qt.ApplicationModal)
    layout = diag.layout()
    spacer = QtGui.QSpacerItem(400, 0, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
    layout.addItem(spacer, layout.rowCount(), 0, 1, layout.columnCount())
    return diag


# ──────────────────────────────────────────────────────────────────────────────
def message_box(message: str, title: str = "Message", details: str = "") -> None:
    """
    Show a basic message dialog (modal).

    If `App.GuiUp` is False, prints to the console.

    :param str message: summary
    :param str title: box title, defaults to "Message"
    :param str details: expandable text, defaults to None
    """
    if App.GuiUp:
        diag = _basic_modal_dlg(message, title, details)
        diag.exec_()
    else:
        print_log(
            textwrap.dedent(f"""
            -----------------
            # {title}
            ## {message}
            {details}
            -----------------
            """)  # noqa: COM812
        )


# ──────────────────────────────────────────────────────────────────────────────
def confirm_box(message: str, title: str = "Message", details: str = "") -> bool:
    """
    Ask for a confirmation with a basic dialog.

    Requires App.GuiUp == True

    :param str message: summary
    :param str title: box title, defaults to "Message"
    :param str details: expandable text, defaults to None
    :return bool: True if user accepts
    """
    if App.GuiUp:
        diag = _basic_modal_dlg(message, title, details)
        diag.setStandardButtons(QtGui.QMessageBox.Ok | QtGui.QMessageBox.Cancel)
        return diag.exec_() == QtGui.QMessageBox.Ok
    return False


##% ────────────────────────────────────────────────────────────────────────────
class TransactionAbortException(Exception):  # noqa: N818
    """Exception to signal transaction abort."""


##% ────────────────────────────────────────────────────────────────────────────
class TransactionCtrl:
    """Controller to signal transaction abort."""

    aborted: bool = False

    def abort(self) -> None:
        self.aborted = True
        raise TransactionAbortException()  # noqa: RSE102


##@ ────────────────────────────────────────────────────────────────────────────
@contextlib.contextmanager
def transaction(name: str, doc: Document = None) -> Generator[TransactionCtrl, Any, None]:
    """
    Context manager to execute code in a document transaction (undo/redo)

    :param str name: Name of the transaction in undo/redo stack
    :param Document doc: selected document, defaults to None
    :yield TransactionCtrl: transaction control
    """
    ctrl = TransactionCtrl()
    _doc = doc or App.activeDocument()
    if not _doc:
        msg = "There is not active document"
        raise ValueError(msg)
    try:
        _doc.openTransaction(name)
        yield ctrl
        _doc.commitTransaction()
    except TransactionAbortException:
        # Intentionally aborted transaction
        _doc.abortTransaction()
        print_log(f"Transaction '{name}' aborted intentionally")
    except:
        # Unexpected exception
        _doc.abortTransaction()
        print_err(f"Transaction '{name}' aborted unexpectedly")
        raise


##: Typing For documentation purposes
##: ────────────────────────────────────────────────────────────────────────────
if TYPE_CHECKING:

    class DataProxy(Protocol):
        """
        Documentation only protocol to illustrate the method signatures.

        All methods are optional. And Events are also optional in most cases.
        """

        def on_create(self, event: events.CreateEvent) -> None: ...
        def on_extension(self, event: events.ExtensionEvent) -> None: ...
        def on_attach(self, event: events.AttachEvent) -> None: ...
        def on_start(self, event: events.StartEvent) -> None: ...
        def on_restore(self, event: events.DocumentRestoredEvent) -> None: ...
        def on_execute(self, event: events.ExecuteEvent) -> None: ...
        def on_change(self, event: events.PropertyChangedEvent) -> None: ...
        def on_before_change(self, event: events.PropertyWillChangeEvent) -> None: ...
        def on_serialize(self, event: events.SerializeEvent) -> None: ...
        def on_deserialize(self, event: events.DeserializeEvent) -> None: ...
        def on_remove(self, event: events.RemoveEvent) -> None: ...
        def is_dirty(self) -> bool: ...
        def is_active(self) -> bool: ...
        def on_migrate_class(self, event: events.MigrationEvent) -> None: ...
        def on_migrate_upgrade(self, event: events.MigrationEvent) -> None: ...
        def on_migrate_downgrade(self, event: events.MigrationEvent) -> None: ...
        def on_migrate_complete(self, event: events.MigrationEvent) -> None: ...
        def on_migrate_error(self, event: events.MigrationEvent) -> None: ...

        @property
        def Object(self) -> DocumentObject: ...

        @property
        def ViewObject(self) -> ViewProviderDocumentObject: ...

        @classmethod
        def create(
            cls,
            name: str | None = None,
            label: str | None = None,
            doc: Document | None = None,
        ) -> DocumentObject: ...

    class ViewProxy(Protocol):
        """
        Documentation only protocol to illustrate the method signatures.

        All methods are optional. And Events are also optional in most cases.
        """

        def on_create(self, event: events.CreateEvent) -> None: ...
        def on_attach(self, event: events.AttachEvent) -> None: ...
        def on_start(self, event: events.StartEvent) -> None: ...
        def on_serialize(self, event: events.SerializeEvent) -> None: ...
        def on_deserialize(self, event: events.DeserializeEvent) -> None: ...
        def on_change(self, event: events.PropertyChangedEvent) -> None: ...
        def on_before_change(self, event: events.PropertyWillChangeEvent) -> None: ...
        def on_object_change(self, event: events.DataChangedEvent) -> None: ...

        def on_context_menu(self, event: events.ContextMenuEvent) -> None: ...
        def icon(self) -> str | None: ...
        def set_display_mode(self, mode: str) -> str: ...
        def display_modes(self) -> list[str]: ...
        def default_display_mode(self) -> str: ...

        def on_claim_children(self, event: events.ClaimChildrenEvent) -> list[DocumentObject]: ...
        def on_edit_start(self, event: events.EditStartEvent) -> bool | None: ...
        def on_edit_end(self, event: events.EditEndEvent) -> bool | None: ...
        def on_delete(self, event: events.DeleteEvent) -> bool: ...
        def on_dbl_click(self, event: events.DoubleClickEvent) -> bool: ...

        def can_drag_objects(self) -> bool: ...
        def can_drop_objects(self) -> bool: ...
        def can_drag_object(self, event: events.DragAndDropEvent) -> bool: ...
        def can_drop_object(self, event: events.DragAndDropEvent) -> bool: ...
        def on_drag_object(self, event: events.DragAndDropEvent) -> None: ...
        def on_drop_object(self, event: events.DragAndDropEvent) -> None: ...

        @property
        def Object(self) -> DocumentObject: ...

        @property
        def ViewObject(self) -> ViewProviderDocumentObject: ...


if not TYPE_CHECKING:

    class DataProxy: ...  # noqa: D101

    class ViewProxy: ...  # noqa: D101


Proxy: TypeAlias = DataProxy | ViewProxy
