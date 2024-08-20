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

__author__          = "Frank David Martínez Muñoz"
__copyright__       = "(c) 2024 Frank David Martínez Muñoz."
__license__         = "LGPL 2.1"
__version__         = "1.0.0-beta1"
__min_python__      = "3.8"
__min_freecad__     = "0.21"


# Conventions for sections in this file:
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
from dataclasses import dataclass
from typing import Any, Tuple, Callable, Dict, Iterable, List, Set, Union, NewType
from enum import IntEnum, Enum
import inspect
import re, sys, traceback, textwrap, contextlib
from pathlib import Path
from warnings import warn
import typing

if typing.TYPE_CHECKING:
    from FreeCAD import Document                        # type: ignore
    from FreeCAD import DocumentObject                  # type: ignore
    from FreeCAD import ParameterGrp                    # type: ignore
    from FreeCADGui import ViewProviderDocumentObject   # type: ignore
    from PySide2.QtWidgets import QMenu, QMessageBox    # type: ignore
    from Part import Shape                              # type: ignore
else:
    Document = NewType('Document', object)
    DocumentObject = NewType('DocumentObject', object)
    ViewProviderDocumentObject = NewType('ViewProviderDocumentObject', object)
    ParameterGrp = NewType('ParameterGrp', object)
    QMenu = NewType('QMenu', object)
    QMessageBox = NewType('QMessageBox', object)
    Shape = NewType('Shape', object)  # Part.Shape class

#: Conditional imports
#: ─────────────────────────────────────────────────────────────────────────────
try:
    # ┌───────────────────────────────────┐ 
    # │ FreeCAD Environment imports       │ 
    # └───────────────────────────────────┘    
    import FreeCAD as App  # type: ignore
    
    if App.GuiUp:
        from pivy import coin               # type: ignore
        import FreeCADGui as Gui            # type: ignore
        from PySide import QtGui, QtCore    # type: ignore

    def print_log(*args):
        App.Console.PrintLog(f"[info] {' '.join(str(a) for a in args)}\n")

    def print_err(*args):
        App.Console.PrintError(f"[error] {' '.join(str(a) for a in args)}\n")

except ImportError:
    # ┌────────────────────────────────────────────────────────┐ 
    # │ Standalone Environment imports (Faked FreeCAD types)   │ 
    # └────────────────────────────────────────────────────────┘    
    App = object()

    def print_log(*args):
        print('[info]', *args, file=sys.stdout)

    def print_err(*args):
        print('[error]', *args, file=sys.stderr)

print_err.__doc__ = "Print into the FreeCAD console with error level."
print_log.__doc__ = "Print into the FreeCAD console with info level."


#: FreeCAD defined c++ types (Just for the sake of documentation)
#: fake type hints compatible with python 3.8+
#: ─────────────────────────────────────────────────────────────────────────────
ObjectRef = Union[DocumentObject, ViewProviderDocumentObject]
PartDesign_Body = NewType('PartDesign_Body', object)  # PartDesign.Body class


#: User defined python Types (Just for the sake of documentation)
#: fake type hints compatible with python 3.8+
#: ─────────────────────────────────────────────────────────────────────────────
DataProxy = NewType('DataProxy', object)  # Python proxy of DocumentObject
ViewProxy = NewType('ViewProxy', object)  # Python proxy of ViewObject
Proxy = NewType('Proxy', object)


#: Defined symbols
#: ─────────────────────────────────────────────────────────────────────────────
_SO_VERSION             = 'SOInternalVersion'
_SO_META                = '__so_meta__'
_SO_REF                 = '__so_ref__'
_ON_EXTENSION           = 'on_extension'
_ON_ATTACH              = 'on_attach'
_ON_CREATE              = 'on_create'
_ON_START               = 'on_start'
_ON_RESTORE             = 'on_restore'
_ON_EXECUTE             = 'on_execute'
_ON_CHANGE              = 'on_change'
_ON_BEFORE_CHANGE       = 'on_before_change'
_ON_SERIALIZE           = 'on_serialize'
_ON_DESERIALIZE         = 'on_deserialize'
_ON_REMOVE              = 'on_remove'
_IS_DIRTY               = 'is_dirty'
_ON_CONTEXT_MENU        = 'on_context_menu'
_ON_CLAIM_CHILDREN      = 'on_claim_children'
_ON_EDIT_START          = 'on_edit_start'
_ON_EDIT_END            = 'on_edit_end'
_ON_DELETE              = 'on_delete'
_ON_DBL_CLICK           = 'on_dbl_click'
_GET_ICON               = 'icon'
_SET_DISPLAY_MODE       = 'set_display_mode'
_DISPLAY_MODES          = 'display_modes'
_DEFAULT_DISPLAY_MODE   = 'default_display_mode'
_ON_OBJECT_CHANGE       = 'on_object_change'
_CAN_DRAG_OBJECTS       = 'can_drag_objects'
_CAN_DROP_OBJECTS       = 'can_drop_objects'
_CAN_DRAG_OBJECT        = 'can_drag_object'
_CAN_DROP_OBJECT        = 'can_drop_object'
_ON_DRAG_OBJECT         = 'on_drag_object'
_ON_DROP_OBJECT         = 'on_drop_object'
_ON_MIGRATE_CLASS       = 'on_migrate_class'
_ON_MIGRATE_UP          = 'on_migrate_upgrade'
_ON_MIGRATE_DOWN        = 'on_migrate_downgrade'
_ON_MIGRATE_COMPLETE    = 'on_migrate_complete'
_ON_MIGRATE_ERROR       = 'on_migrate_error'
_SET_VERSION            = 'set_version'


#: Supported Property types
#: Source: https://wiki.freecad.org/FeaturePython_Custom_Properties
#: ─────────────────────────────────────────────────────────────────────────────
_FC_PROPERTY_TYPES = (
    'Acceleration', 'AmountOfSubstance', 'Angle', 'Area',
    'Bool', 'BoolList', 'Color', 'ColorList',
    'CurrentDensity', 'Density', 'Direction', 'DissipationRate',
    'Distance', 'DynamicViscosity', 'ElectricCharge', 'ElectricCurrent',
    'ElectricPotential', 'ElectricalCapacitance', 'ElectricalConductance', 
    'ElectricalConductivity', 'ElectricalInductance', 'ElectricalResistance', 
    'Enumeration', 'ExpressionEngine',
    'File', 'FileIncluded', 'Float', 'FloatConstraint',
    'FloatList', 'Font', 'Force', 'Frequency',
    'HeatFlux', 'Integer', 'IntegerConstraint', 'IntegerList',
    'IntegerSet', 'InverseArea', 'InverseLength', 'InverseVolume',
    'KinematicViscosity', 'Length', 'Link', 'LinkChild',
    'LinkGlobal', 'LinkHidden', 'LinkList', 'LinkListChild',
    'LinkListGlobal', 'LinkListHidden', 'LinkSub', 'LinkSubChild',
    'LinkSubGlobal', 'LinkSubHidden', 'LinkSubList', 'LinkSubListChild',
    'LinkSubListGlobal', 'LinkSubListHidden', 'LuminousIntensity', 
    'MagneticFieldStrength',
    'MagneticFlux', 'MagneticFluxDensity', 'Magnetization', 'Map',
    'Mass', 'Material', 'MaterialList', 'Matrix',
    'Path', 'Percent', 'PersistentObject', 'Placement',
    'PlacementLink', 'PlacementList', 'Position', 'Power',
    'Precision', 'Pressure', 'PythonObject', 'Quantity',
    'QuantityConstraint', 'Rotation', 'ShearModulus', 'SpecificEnergy',
    'SpecificHeat', 'Speed', 'Stiffness', 'Stress',
    'String', 'StringList', 'Temperature', 'ThermalConductivity',
    'ThermalExpansionCoefficient', 'ThermalTransferCoefficient', 'Time', 'UUID',
    'UltimateTensileStrength', 'VacuumPermittivity', 'Vector', 'VectorDistance',
    'VectorList', 'Velocity', 'Volume', 'VolumeFlowRate',
    'VolumetricThermalExpansionCoefficient', 'Work', 'XLink', 'XLinkList',
    'XLinkSub', 'XLinkSubList', 'YieldStrength', 'YoungsModulus'
)


#: Global private static registries
#: ─────────────────────────────────────────────────────────────────────────────
_extensions : Dict[str, 'ExtensionSupport'] = dict()


# ┌────────────────────────────────────────────────────────────────────────────┐
# │ private Utility functions                                                  │
# └────────────────────────────────────────────────────────────────────────────┘

# ──────────────────────────────────────────────────────────────────────────────
def _call(obj, name, *args, **kwargs):
    """Call a method on obj if exists"""
    func = getattr(obj, name, None)
    if func:
        return func(*args, **kwargs)


# ──────────────────────────────────────────────────────────────────────────────
def _snake_to_camel(text: str) -> str:
    """Transform text from snake naming to camel case"""
    if text:
        return "".join(token.capitalize() for token in text.split('_'))
    

# ──────────────────────────────────────────────────────────────────────────────
def _resolve_uri(path: str, base_dir: Path = None) -> str:
    '''Resolve relative paths if prefixed with "self:"'''
    if str(path).startswith('self:') and base_dir:
        rel_path_elements = path[5:].lstrip(' /').split('/')
        return str(Path(base_dir, *rel_path_elements))
    return path


# ──────────────────────────────────────────────────────────────────────────────
def _prop_constructor(prop_type):
    '''Create a constructor for a specific Property Type'''
    def constructor(*,
        name: str = None,
        section: str = 'Data',
        default: Any = None,
        description: str = '',
        mode: PropertyMode = PropertyMode.Default,
        observer_func: Callable = None,        
        link_property: Union[str, bool] = False):
        return Property(
            type=prop_type,
            section=section,
            observer_func=observer_func,
            name=name,
            link_property=link_property,
            default=default,
            description=description,
            mode=mode)
    return constructor


# ──────────────────────────────────────────────────────────────────────────────
def _is(types: Any) -> Callable:
    '''Returns a predicate to check the type of the passed object'''
    def predicate(obj: Any) -> bool:
        return isinstance(obj, types)
    return predicate


# ──────────────────────────────────────────────────────────────────────────────
def _get_properties(cls) -> Iterable[Tuple[str, 'Property']]:
    '''Returns the list of Properties defined in a proxy class'''
    return inspect.getmembers(cls, _is(Property))    


# ──────────────────────────────────────────────────────────────────────────────
def _get_display_modes(cls) -> Iterable[Tuple[str, 'DisplayMode']]:
    '''Returns the list of Display Modes defined in a proxy class'''
    return inspect.getmembers(cls, _is(DisplayMode))    


# ──────────────────────────────────────────────────────────────────────────────
def _t_forward(cls, forward_from: str, forward_to: str):
    """
    Create a function that forwards the call to another function

    :param str forward_from: name of the original function
    :param str forward_to: name of the target function
    """
    overridden = getattr(cls, forward_from, None)
    if overridden:
        raise NameError(f'{forward_from} is already reserved. ' +
                        f'use {forward_to} instead')
    
    forward = getattr(cls, forward_to, None)
    if forward:
        def handler(self, *args, **kwargs):
            return forward(self, *args, **kwargs)
        handler.__signature__ = inspect.signature(forward)
        handler.__doc__ = forward.__doc__
        handler.__name__ = forward_from
        setattr(cls, forward_from, handler)
    
#% ┌───────────────────────────────────────────────────────────────────────────┐
#% │ Core types                                                                │
#% └───────────────────────────────────────────────────────────────────────────┘


#% ─────────────────────────────────────────────────────────────────────────────
class _DocIntEnum(IntEnum):
    """
    IntEnum with member docs
    See: https://stackoverflow.com/a/50473952/1524027
    """
    def __new__(cls, value, doc=''):
        self = int.__new__(cls, value)
        self._value_ = value
        self.__doc__ = doc
        return self
    

#% ─────────────────────────────────────────────────────────────────────────────
class FeatureState(_DocIntEnum):
    """
    ScriptedObject state. See lifecycle
    """
    Attaching   = -1, 'FreeCAD is creating and binding the objects'
    Creating    =  0, 'Setting up all stuff like properties, extensions, etc...'
    Created     =  1, 'Already created'
    Active      =  2, 'Ready for execution'
    Restoring   =  3, 'Restoring from FCStd document'
    Restored    =  4, 'Fully restored from FCStd document'


#% ─────────────────────────────────────────────────────────────────────────────
class PropertyMode(_DocIntEnum):
    """
    Property mode flags
    """
    Default     = 0, "No special property type"
    ReadOnly    = 1, "Property is read-only in the editor"
    Transient   = 2, "Property won't be saved to file"
    Hidden      = 4, "Property won't appear in the editor"
    Output      = 8, "Modified property doesn't touch its parent container"
    NoRecompute = 16, "Modified property doesn't touch its container for recompute"
    NoPersist   = 32, "Property won't be saved to file at all"


#% ─────────────────────────────────────────────────────────────────────────────
class PropertyEditorMode(_DocIntEnum):
    """
    Editor Modes
    """
    Default     = 0, "No special mode"
    ReadOnly    = 1, "Property is read only in the editor"
    Hidden      = 2, "Property is hidden in the editor"


#% ─────────────────────────────────────────────────────────────────────────────
class EditMode(_DocIntEnum):
    """
    ViewProvider Edit Mode
    """
    Default     = 0, "The object will be edited using the mode defined \
                      internally to be the most appropriate for the object type"
    Transform   = 1, "The object will have its placement editable with the \
                      `Std TransformManip` command"
    Cutting     = 2, "This edit mode is implemented as available but currently \
                      does not seem to be used by any object"
    Color       = 3, "The object will have the color of its individual faces \
                      editable with the Part FaceColors command"

#% ─────────────────────────────────────────────────────────────────────────────
class Property:
    '''
    Proxy object to create, access and manipulate remote freecad properties 
    in DocumentObject and ViewObject instances.

    Ref: https://wiki.freecad.org/FeaturePython_Custom_Properties
    '''

    type: str                        # Type of the supported property
    binding: str                     # Name of the property on the proxy class
    observer_func: Callable          # Change listener
    observer_arity: int              # Change listener arity
    name: str                        # Actual name of the property
    link_property: Union[str, bool]  # Name of the Link Property to configure
    section: str                     # Capitalized single word due to FC limitations
    default: Any                     # Initial value
    description: str                 # GUI description
    mode: PropertyMode               # PropertyEditor mode for the property           
    enum: Enum                       # Type of enum used by "App::PropertyEnumeration"
    options: Callable                # Callable that provides a list of options

    # ──────────
    def __init__(self,
                 type: str,
                 binding: str = None,
                 section: str = 'Data',
                 observer_func: Callable = None,
                 name: str = None,
                 link_property: Union[str, bool] = False,
                 default: Any = None,
                 description: str = '',
                 mode: PropertyMode = PropertyMode.Default,
                 enum: Enum = None,
                 options: Callable = None):
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
        self.mode = mode
        self.enum = enum
        self.options = options

    #@ ─────────
    def observer(self, func):
        '''Decorator to register the listener for property change event'''
        self.observer_func = func
        sig = inspect.signature(func)
        self.observer_arity = len(sig.parameters)
        return func

    # ──────────
    def create(self, fp: ObjectRef):
        '''Adds the property to the object and initialize it'''
        if not self.name in fp.PropertiesList:
            fp.addProperty(self.type, self.name, self.section, self.description, self.mode)
            if self.enum:
                setattr(fp, self.name, [str(e.value) for e in list(self.enum)])
            elif self.options:
                setattr(fp, self.name, self.options())
            self.reset(fp)

    # ──────────
    def reset(self, fp: ObjectRef):
        '''Set the value to its default'''
        if self.default is not None:
            self.update(fp, self.default)

    # ──────────
    def update(self, obj: ObjectRef, value: Any):
        '''Set the value of the property in the remote object'''
        if hasattr(obj, self.name):
            if self.enum:
                setattr(obj, self.name, str(value.value))
            else:
                attr = getattr(obj, self.name)
                if hasattr(attr, 'Value'):
                    attr.Value = value
                else:
                    setattr(obj, self.name, value)

    # ──────────
    def read(self, obj: ObjectRef):
        '''Get the value of the property from the remote object'''
        if hasattr(obj, self.name):
            v = getattr(obj, self.name)
            if self.enum:
                if v is None:
                    return tuple(self.enum)[0]
                return self.enum(v)
            elif hasattr(v, 'Value'):
                return v.Value
            else:
                return v

    # ──────────
    def set_mode(self, obj: ObjectRef, mode: PropertyEditorMode):
        '''Change editor mode for the property'''
        if hasattr(obj, self.name):
            obj.setEditorMode(self.name, mode)

    # ──────────
    def set_status(self, obj: ObjectRef, status: str):
        '''Change editor status for the property'''
        if hasattr(obj, self.name):
            obj.setPropertyStatus(self.name, status)


#% ─────────────────────────────────────────────────────────────────────────────
class DisplayMode:
    """
    Declarator of Display Modes, allows to configure a mode and optionally
    a builder method to create and register the coin object.

    :param str name: Name of the display mode
    :param bool is_default: Configure the DM as default, defaults to False
    :param Callable[[ViewObject], coin.SoGroup] builder: Method to build the coin object if required
    """

    name: str
    _builder_func: Callable
    is_default: bool

    # ──────────
    def __init__(self, 
                 name: str = None, 
                 is_default: bool = False, 
                 builder: Callable = None):
        self.name = name
        self.is_default = is_default
        self._builder_func = builder

    #@ ─────────
    def builder(self, func):
        '''Decorator to bind a builder for the coin object'''
        self._builder_func = func
        return func


#% ─────────────────────────────────────────────────────────────────────────────
@dataclass
class _Template:
    '''
    Internal class used to inject tailored methods into the proxy objects to
    interface between FreeCAD internal objects and python proxy instances with
    the new API.
    '''

    name: str                   # Name of the method to be created
    aliases: Iterable[str]      # Aliases to be injected also
    builder: Callable           # The actual method builder
    allow_override: bool        # allow/deny user defined methods that collide
    override_error_msg: str     # Message to suggest alternative names

    # ──────────
    def __call__(self, meta: 'TypeMeta') -> None:
        '''Apply the template'''
        self.add(meta, self.name, self.allow_override)
        if self.aliases:
            for name in self.aliases:
                self.add(meta, name, True)

    # ──────────
    def add(self, meta: 'TypeMeta', name: str, allow_override: bool):
        '''Build the method and inject with the name'''
        overridden = getattr(meta.cls, name, None)
        if overridden and not allow_override:
            msg = self.override_error_msg or ''
            raise NameError(f"Attribute {name} is already defined. {msg}")
        attr = self.builder(overridden, meta)
        if attr:
            if hasattr(attr, '__name__'):
                attr.__name__ = name
            setattr(meta.cls, name, attr)

    # ──────────
    @property
    def __doc__(self):
        return self.builder.__doc__


#% ─────────────────────────────────────────────────────────────────────────────
class ExtensionSupport:
    '''
    Base class of extension managers
    '''

    name: str # Name of the supported extension

    # ──────────
    def __init__(self, name: str) -> None:
        self.name = name
        _extensions[name] = self

    # ──────────
    def on_create(self, proxy: Proxy, obj: ObjectRef, meta: 'TypeMeta'):
        '''Extension listener for on_create event'''
        pass

    # ──────────
    def on_restore(self, proxy: Proxy, obj: ObjectRef, meta: 'TypeMeta'):
        '''Extension listener for on_restore event'''
        pass

    # ──────────
    def on_attach(self, proxy: Proxy, obj: ObjectRef, meta: 'TypeMeta'):
        '''Extension listener for on_attach event'''
        pass

    # ──────────
    def on_start(self, proxy: Proxy, obj: ObjectRef, meta: 'TypeMeta'):
        '''Extension listener for on_start event'''
        self.add_extension(proxy, obj)

    # ──────────
    def on_execute(self, proxy: Proxy, obj: ObjectRef, meta: 'TypeMeta'):
        '''Extension listener for on_execute event'''
        pass

    # ──────────
    def on_add(self, proxy: Proxy, obj: ObjectRef, meta: 'TypeMeta'):
        '''Extension listener for on_add event'''
        pass

    # ──────────
    def on_change(self, proxy: Proxy, obj: ObjectRef, meta: 'TypeMeta', prop: str):
        '''Extension listener for on_change event'''
        pass

    # ──────────
    def on_class(self, meta: 'TypeMeta'):
        '''Extension listener for on_class event'''
        pass

    # ──────────
    def add_extension(self, proxy: Proxy, obj: ObjectRef, name: str = None):
        '''Adds the extension to the object'''
        _name = name or self.name
        if not _name in _extensions:
            raise NameError(f"Extension {_name} not found.")
        
        if not obj.hasExtension(_name):
            obj.addExtension(_name)
            self.on_add(proxy, obj, self.find_meta(proxy))
            _call(proxy, _ON_EXTENSION, obj, _name)

    # ──────────
    def find_meta(self, proxy: Proxy) -> 'TypeMeta':
        '''Returns the meta info associated with the proxy'''
        meta = getattr(proxy.__class__, _SO_META, None)
        if meta is None:
            raise TypeError(f"Invalid proxy type: {proxy.__class__.__name__}")


#% ─────────────────────────────────────────────────────────────────────────────
class Part_AttachExtensionPython(ExtensionSupport):
    '''Extension manager of: Part::AttachExtensionPython'''
    
    # ──────────
    def on_execute(self, proxy: DataProxy, obj: DocumentObject, meta: 'TypeMeta'):
        obj.positionBySupport()


#% ─────────────────────────────────────────────────────────────────────────────
class App_LinkBaseExtensionPython(ExtensionSupport):
    '''Extension manager of App::LinkBaseExtensionPython'''

    # ──────────
    def on_class(self, meta: 'TypeMeta'):
        meta.view_provider_name_override = 'Gui::ViewProviderLinkPython'

    # ──────────
    def resolve_link_prop(self, link_property: Union[str, bool], source: str) -> str:
        if isinstance(link_property, bool) and link_property:
            return source
        return link_property

    # ──────────
    def on_start(self, proxy: DataProxy, obj: DocumentObject, meta: 'TypeMeta'):
        super().on_start(proxy, obj, meta)
        mapping = {
            self.resolve_link_prop(prop.link_property, prop.name): prop.name 
            for prop in meta.properties.values()
            if prop.link_property
        }
        if len(mapping) > 0:
            obj.configLinkProperty(**mapping)


#% ─────────────────────────────────────────────────────────────────────────────
class App_LinkExtensionPython(App_LinkBaseExtensionPython):
    '''Extension manager of App::LinkBaseExtensionPython'''
    pass


#% ─────────────────────────────────────────────────────────────────────────────
class TypeMeta:
    '''
    Metadata of the proxy classes. Inspects the original class and provides
    mechanisms to enhance them by adding methods, properties, display modes, 
    extensions, etc...
    '''

    cls: any                                # proxy enhanced class
    properties: Dict[str, Property]         # map binding to property
    property_lookup: Dict[str, Property]    # map name to property
    display_modes: Dict[str, DisplayMode]   # display mode builders
    view_proxy: ViewProxy                   # associated ViewProxy
    extensions: Set[ExtensionSupport]       # Added extensions
    version: int                            # Proxy versions for migrations support
    object_type: str                        # Type of DocumentObject
    subtype: str                            # Subtype of Proxy
    base_dir: Path                          # Directory where the class is declared
    view_provider_name_override: str        # Forced type of the view provider
    icon: str                               # Icon path

    # ──────────
    def __init__(self, 
                 cls: Any, 
                 object_type: str = None,
                 base_dir: Path = None,
                 subtype: str = None,
                 view_proxy: Any = None, 
                 extensions: Iterable[str] = None,
                 version: int = 1,
                 view_provider_name_override: str = None,
                 icon: str = None):
        self.cls = cls
        self.version = version
        self.object_type = object_type
        self.subtype = subtype
        self.base_dir = base_dir
        self.properties = dict()
        self.property_lookup = dict()
        self.display_modes = dict()
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
            self.extensions = set(_extensions[name] for name in extensions)
        else:
            self.extensions = set()
        self.view_provider_name_override = view_provider_name_override
        cls.__so_meta__ = self
        self.bind_properties()

    # ──────────
    def bind_properties(self):
        '''Bind all available properties'''
        for prop in self.properties.values():
            self.bind_property(prop)

    # ──────────
    def bind_property(self, prop: Property):
        '''Bind a property to a local proxy attribute'''
        def getter(self):
            return prop.read(self.__so_ref__)
        
        def setter(self, value):
            prop.update(self.__so_ref__, value)

        setattr(self.cls, 
                prop.binding, 
                property(getter, setter, doc=prop.description))

    # ──────────
    def add_property(self, fp: ObjectRef, prop: Property):
        '''Create and register a property'''        
        if bool(prop.binding) and prop.binding in self.properties:
            raise NameError(
                f'Binding "{self.cls.__name__}.{prop.binding}" already exists')

        if prop.name in self.property_lookup:
            raise NameError(
                f'Property "{self.cls.__name__}.Object.{prop.name}" already exists')
        
        if prop.name in fp.PropertiesList:
            raise NameError(
                f'Property "{self.cls.__name__}.Object.{prop.name}" already exists')

        prop.create(fp)

    # ──────────
    def apply_extensions(self, proxy: Proxy, obj: ObjectRef, method_name: str, 
                         *args, **kwargs):
        '''Call extensions runtime lifecycle'''
        if self.extensions:
            for ext in self.extensions:
                method = getattr(ext, method_name)
                method(proxy, obj, self, *args, **kwargs)

    # ──────────
    def apply_extensions_on_class(self):
        '''Call extensions static lifecycle'''
        if self.extensions:
            for ext in self.extensions:
                ext.on_class(self)

    # ──────────
    def add_version_prop(self, obj: ObjectRef):
        '''Inject the fpo version into the proxy. (For migrations)'''
        if not _SO_VERSION in obj.PropertiesList:
            obj.addProperty(
                'App::PropertyInteger', 
                _SO_VERSION, 
                'SO', 
                'Internal Scripted Object Version', 
                PropertyMode.Hidden.value)
        setattr(obj, _SO_VERSION, self.version)

    # ──────────
    def init_properties(self, proxy: Proxy, obj: ObjectRef):
        for prop in self.properties.values():
            prop.create(obj)
            if prop.default is not None:
                proxy.__so_old__[prop.name] = prop.default

    # ──────────
    def init_display_modes(self, proxy: ViewProxy, obj: ViewProviderDocumentObject):
        if len(self.display_modes) > 0:
            for dm in self.display_modes.values():
                dm_obj = None
                if dm._builder_func:
                    dm_obj = dm._builder_func(proxy, obj)
                if not dm_obj:
                    dm_obj = coin.SoGroup()
                obj.addDisplayMode(dm_obj, dm.name)

#% ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Preference:
    group: str
    name: str
    default: Any = None
    value_type: type = None
    root: str = "BaseApp"
    many: bool = False

    # ─────────
    def __post_init__(self):
        if self.value_type is None:
            self.value_type = type(self.default) if self.default is not None else str
        
    # ─────────
    @property
    def group_key(self):
        return f"User parameter:{self.root}/{self.group}"
    
    # ─────────
    def read(self):
        group = App.ParamGet(self.group_key)
        try:
            if self.value_type == bool:
                v = group.GetBool(self.name)
                return self.default if v is None else v
            elif self.value_type == int:
                return group.GetInt(self.name) or self.default
            elif self.value_type == float:
                return group.GetFloat(self.name) or self.default
            elif self.value_type == str:
                return group.GetString(self.name) or self.default
        except BaseException:
            print_err(f"Error reading preference: {self}")
        return self.default

    # ─────────
    # Read/Write shortcut
    def __call__(self, *args, **kwargs):
        n = len(args)
        if n == 0:
            value = self.read()
            if value is None and len(kwargs) > 0:
                if len(kwargs) == 1 and 'default' in kwargs:
                    return kwargs['default']
                raise NameError(f"only 'default' named argument is acceptable")
            return value
        if n > 1:
            raise ValueError("This function accepts only one argument")
        self.write(args[0])
    
    # ─────────
    def write(self, value):
        group = App.ParamGet(self.group_key)
        try:
            if self.value_type == bool:
                if value is None:
                    group.RemBool(self.name)
                else:
                    group.SetBool(self.name, self.value_type(value))
            elif self.value_type == int:
                if value is None:
                    group.RemInt(self.name)
                else:
                    group.SetInt(self.name, self.value_type(value))
            elif self.value_type == float:
                if value is None:
                    group.RemFloat(self.name)
                else:
                    group.SetFloat(self.name, self.value_type(value))
            elif self.value_type == str:
                if value is None:
                    group.RemString(self.name)
                else:
                    group.SetString(self.name, self.value_type(value))
        except BaseException:
            print_err(f"Error writing preference: {self}")

    #% ─────────
    class ParamObserver:
        listeners = dict()

        # ─────────
        def __init__(self, group: ParameterGrp, callback: Callable) -> None:
            self.callback = callback
            group.AttachManager(self)
            Preference.ParamObserver.listeners[hash(self)] = group

        # ─────────
        def slotParamChanged(self, group, value_type, name, value):
            self.callback(group, value_type, name, value)

        # ─────────
        def unsubscribe(self):
            try:
                del Preference.ParamObserver.listeners[hash(self)]
            except:
                print_err(f"Invalid subscription or already removed: {self.callback.__name__}")

    # ─────────
    @staticmethod
    def subscribe(group: str, root: str = "BaseApp"):
        group = f"User parameter:{root}/{group}"
        def wrapper(func):
            param_group = App.ParamGet(group)
            return Preference.ParamObserver(param_group, func)
        return wrapper



#@ ┌───────────────────────────────────────────────────────────────────────────┐
#@ │ Decorators                                                                │
#@ └───────────────────────────────────────────────────────────────────────────┘

#@ ─────────────────────────────────────────────────────────────────────────────
def proxy(*,
          object_type: str = 'App::FeaturePython',
          subtype: str = None,
          view_proxy: ViewProxy = None, 
          extensions: Iterable[str] = None,
          view_provider_name_override: str = None,
          version: int = 1):
    
    '''
    Main decorator for DataProxy creation. Decorating a class with @proxy(...)
    adds support for the new API
    '''

    # base dir is useful for relative resource lookup
    base_dir = Path(inspect.stack()[1].filename).parent

    # Actual decorator that applies all the required transformations to the class
    def transformer(cls):
        meta = TypeMeta(cls, 
                        object_type, 
                        base_dir, 
                        subtype or cls.__name__, 
                        view_proxy, 
                        extensions, 
                        version,
                        view_provider_name_override)
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
        return cls
    
    return transformer


#@ ─────────────────────────────────────────────────────────────────────────────
def view_proxy(*,
        view_provider_name_override: str = None,
        extensions: Iterable[str] = None,
        icon: str = None):
    
    '''
    Decorator for ViewProxy creation. Decorating a class with @view_proxy(...)
    adds support for the new API.
    '''

    # base dir is useful for relative resource lookup
    base_dir = Path(inspect.stack()[1].filename).parent

    # Actual decorator that applies all the required transformations to the class
    def transformer(cls):
        meta = TypeMeta(cls, 
                        base_dir=base_dir, 
                        extensions=extensions, 
                        view_provider_name_override=view_provider_name_override,
                        icon=icon)
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
        # https://github.com/FreeCAD/FreeCAD/blob/d5b90e50af6a758af748179f289bb8f09e357266/src/Mod/Part/BOPTools/SplitFeatures.py#L129

        _t_forward(meta.cls, 'canDragObjects', _CAN_DRAG_OBJECTS)
        _t_forward(meta.cls, 'canDropObjects', _CAN_DROP_OBJECTS)
        _t_forward(meta.cls, 'canDragObject', _CAN_DRAG_OBJECT)
        _t_forward(meta.cls, 'canDropObject', _CAN_DROP_OBJECT)
        _t_forward(meta.cls, 'dragObject', _ON_DRAG_OBJECT)
        _t_forward(meta.cls, 'dropObject', _ON_DROP_OBJECT)
        
        return cls
    
    return transformer


#@ Internal decorator to create class attribute templates
#@ ─────────────────────────────────────────────────────────────────────────────
def template(*,
        name: str, 
        aliases: Iterable = None, 
        allow_override: bool = False,
        override_error_msg: str = None):
    def wrapper(func) -> _Template:
        return _Template(
            name, 
            aliases, 
            func, 
            allow_override, 
            override_error_msg)
    return wrapper


#$ ┌───────────────────────────────────────────────────────────────────────────┐
#$ │ DataProxy Templates                                                       │
#$ └───────────────────────────────────────────────────────────────────────────┘


#$ ─────────────────────────────────────────────────────────────────────────────
@template(name='__init__', allow_override=True)
def t_proxy_constructor(overridden: Any, meta: TypeMeta):
    '''
    Create a constructor for the DataProxy class. Calls
    the user defined constructor if exists
    '''
    def __init__(self, *args, **kwargs):
        self.__so_ref__ = None
        self.__so_old__ = dict()
        self.Type = meta.subtype
        self.__so_state__ = FeatureState.Attaching    
        if overridden:
            overridden(self, *args, **kwargs)
    if overridden:
        __init__.__doc__ = overridden.__doc__
        __init__.__signature__ = inspect.signature(overridden)
    return __init__


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='unsetupObject', 
        override_error_msg=f"Use {_ON_REMOVE} instead.")
def t_proxy_remove(overridden: Any, meta: TypeMeta):
    call = getattr(meta.cls, _ON_REMOVE, None)
    if call:
        def handler(self, fp: DocumentObject):
            return call(self, fp)
        return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(name=_SET_VERSION)
def t_proxy_set_version(overridden: Any, meta: TypeMeta):
    def handler(self, version: int) -> None:
        '''Update the Object version'''
        if not isinstance(version, int):
            raise ValueError('version must be an integer')
        if hasattr(self, _SO_REF):
            setattr(self.__so_ref__, _SO_VERSION, version)
        else:
            raise TypeError(f'Current class {type(self)} is not a proxy')
    return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(name='Object')
def t_proxy_object(overridden: Any, meta: TypeMeta):
    def handler(self) -> DocumentObject:
        '''Returns FreeCAD internal Feature Object'''
        return self.__so_ref__
    return property(handler)


#$ ─────────────────────────────────────────────────────────────────────────────
@template(name='ViewObject')
def t_proxy_vobject(overridden: Any, meta: TypeMeta):
    def handler(self) -> ViewProviderDocumentObject:
        '''Returns FreeCAD internal View Provider Object'''
        return self.Object.ViewObject
    return property(handler)


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='attach',
        override_error_msg=f"Use {_ON_ATTACH} instead")
def t_proxy_attach(overridden: Any, meta: TypeMeta):
    def handler(self, fp: DocumentObject):
        assert fp.Proxy == self
        self.__so_ref__ = fp
        self.__so_old__ = dict()
        self.Type = meta.subtype

        meta.apply_extensions(self, fp, _ON_ATTACH)
        _call(self, _ON_ATTACH, fp)

        self.__so_state__ = FeatureState.Creating

        meta.add_version_prop(fp)
        meta.init_properties(self, fp)

        meta.apply_extensions(self, fp, _ON_CREATE)
        _call(self, _ON_CREATE, fp)
        
        self.__so_state__ = FeatureState.Created
        
        meta.apply_extensions(self, fp, _ON_START)
        _call(self, _ON_START, fp)
        
        self.__so_state__ = FeatureState.Active
    return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(name='create')
def t_proxy_create(overridden: Any, meta: TypeMeta):
    def create(name: str = None, label: str = None, doc: Document = None):
        """Create the FreeCAD Objects, the Python Proxies and bind them"""
        _name = name or meta.subtype
        proxy = meta.cls()
        
        view_proxy = None
        if meta.view_proxy and App.GuiUp:
            view_proxy = meta.view_proxy(None)

        _doc = doc or App.activeDocument() or App.newDocument()
        fp = _doc.addObject(meta.object_type, _name, proxy, view_proxy, True)

        if hasattr(fp, 'ViewObject') and hasattr(fp.ViewObject, 'Proxy'):
            if view_proxy is None:
                fp.ViewObject.Proxy = 0
            elif fp.ViewObject.Proxy != view_proxy:
                fp.ViewObject.Proxy = view_proxy

        fp.Label = label or _name       
        return fp
    return staticmethod(create)


#$ ─────────────────────────────────────────────────────────────────────────────
@template(name='rebind')
def t_proxy_rebind(overridden: Any, meta: TypeMeta):
    def rebind(fp: DocumentObject):
        """Recreate the proxy objects and rebind them to FreeCAD objects"""
        proxy = meta.cls()
        proxy.__so_ref__ = fp
        fp.Proxy = proxy
        _call(proxy, 'attach', fp)
        if meta.view_proxy and App.GuiUp:
            view_proxy = meta.view_proxy()
            view_proxy.__so_ref__ = fp.ViewObject
            fp.ViewObject.Proxy = view_proxy
        else:
            fp.ViewObject.Proxy = 0
        return proxy
    return staticmethod(rebind)


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='onDocumentRestored',
        override_error_msg=f"Use {_ON_RESTORE} instead")
def t_proxy_restore(overridden: Any, meta: TypeMeta):
    def handler(self, fp: DocumentObject):
        self.__so_state__ = FeatureState.Restoring
        self.__so_ref__ = fp
        self.__so_old__ = dict()
        fp.Proxy = self

        # Restore last values
        for prop_name, prop in meta.property_lookup.items():
            self.__so_old__[prop_name] = prop.read(fp)

        meta.apply_extensions(self, fp, _ON_RESTORE)
        _call(self, _ON_RESTORE, fp)
        
        self._fp_state = FeatureState.Restored

        meta.apply_extensions(self, fp, _ON_START)
        _call(self, _ON_START, fp)
        
        self._fp_state = FeatureState.Active
    return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='execute',
        override_error_msg=f"Use {_ON_EXECUTE} instead")
def t_proxy_execute(overridden: Any, meta: TypeMeta):
    def handler(self, fp: DocumentObject):
        meta.apply_extensions(self, fp, _ON_EXECUTE)
        _call(self, _ON_EXECUTE, fp)
    return handler
    

#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='onBeforeChange',
        override_error_msg=f"Use {_ON_BEFORE_CHANGE} instead")
def t_proxy_before_change(overridden: Any, meta: TypeMeta):
    def handler(self, fp: DocumentObject, prop_name: str):
        if getattr(self, '__so_state__', None) == FeatureState.Active:
            old_value = getattr(fp, prop_name, None)
            self.__so_old__[prop_name] = old_value
            _call(self, _ON_BEFORE_CHANGE, fp, prop_name, old_value)
    return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='onChanged',
        override_error_msg=f"Use {_ON_CHANGE} instead")
def t_proxy_change(overridden: Any, meta: TypeMeta):
    def handler(self, fp: DocumentObject, prop_name: str):
        if getattr(self, '__so_state__', None) == FeatureState.Active:
            new_value = getattr(fp, prop_name)
            old_value = self.__so_old__.get(prop_name, None)
            if new_value != old_value:
                meta.apply_extensions(self, fp, _ON_CHANGE, prop_name)
                prop = meta.property_lookup.get(prop_name, None)
                if prop and prop.observer_func:
                    args = (self, fp, new_value, old_value)[0:prop.observer_arity]
                    prop.observer_func(*args)
                _call(self, _ON_CHANGE, fp, prop_name, new_value, old_value)
    return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(name='add_property')
def t_proxy_add_property(overridden: Any, meta: TypeMeta):
    def handler(self, prop: Property):
        meta.add_property(self.__so_ref__, prop)
    return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(name='set_property_mode')
def t_proxy_set_prop_mode(overridden: Any, meta: TypeMeta):
    def handler(self, *,
                mode: PropertyEditorMode,
                name: str = None,
                binding: str = None):
        if not (name or binding):
            raise NameError('name or binding argument must be provided')
        if name and binding:
            raise NameError('name and binding arguments are mutually exclusive')
        if binding:
            prop = meta.properties.get(binding, None)
            if prop is None:
                raise NameError(f"There is not property bound to name {binding}")
            self.__so_ref__.setPropertyMode(prop.name, mode)
        else:
            self.__so_ref__.setPropertyMode(name, mode)
    return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(name='set_property_status')
def t_proxy_set_prop_status(overridden: Any, meta: TypeMeta):
    def handler(self, *,
                status: str,
                name: str = None,
                binding: str = None):
        if not (name or binding):
            raise NameError('name or binding argument must be provided')
        if name and binding:
            raise NameError('name and binding arguments are mutually exclusive')
        if binding:
            prop = meta.properties.get(binding, None)
            if prop is None:
                raise NameError(f"There is not property bound to name {binding}")
            self.__so_ref__.setPropertyStatus(prop.name, status)
        else:
            self.__so_ref__.setPropertyStatus(name, status)
    return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='dumps', aliases=('__getstate__',),
        override_error_msg=f"Use {_ON_SERIALIZE} instead")
def t_dumps(overridden: Any, meta: TypeMeta):
    def handler(self) -> Dict[str, Any]:
        state = dict()
        if hasattr(self, 'Type'):
            state['_Type'] = self.Type
        _call(self, _ON_SERIALIZE, state)
        return state
    return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='loads', aliases=('__setstate__',),
        override_error_msg=f"Use {_ON_DESERIALIZE} instead")
def t_loads(overridden: Any, meta: TypeMeta):
    def handler(self, state: Dict[str, Any]):
        _type = state.get('_Type', None)
        if _type:
            self.Type = _type
        _call(self, _ON_DESERIALIZE, state)
    return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='mustExecute',
        override_error_msg=f"Use {_IS_DIRTY} instead")
def t_proxy_dirty(overridden: Any, meta: TypeMeta):
    get = getattr(meta.cls, _IS_DIRTY, None)
    if get:
        def handler(self, fp: DocumentObject):
            return get(self, fp)
        return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='getViewProviderName',
        override_error_msg=f"Set the `view_provider_name_override` argument " +
                            "in the decorator instead")
def t_proxy_view_provider_name_override(overridden: Any, meta: TypeMeta):
    if meta.view_provider_name_override:
        def handler(self, fp: DocumentObject):
            return meta.view_provider_name_override
        return handler
    
    view_meta = None
    if meta.view_proxy and hasattr(meta.view_proxy, _SO_META):
        view_meta = meta.view_proxy.__so_meta__

    if view_meta and view_meta.view_provider_name_override:
        def handler(self, fp: DocumentObject):
            return view_meta.view_provider_name_override
        return handler


#$ ┌───────────────────────────────────────────────────────────────────────────┐
#$ │ ViewProxy Templates                                                       │
#$ └───────────────────────────────────────────────────────────────────────────┘

#$ ─────────────────────────────────────────────────────────────────────────────
@template(name='__init__', allow_override=True)
def t_view_proxy_constructor(overridden: Any, meta: TypeMeta):
    '''
    Create the constructor for the ViewProxy. Calls the user defined
    constructor if any. If there is a user defined __init__, it must
    accept the ViewObject as it first argument_
    
    ```python
        def __init__(self, vp, ...)
    ```
    '''
    def __init__(self, vp: ViewProviderDocumentObject, /, *args, **kwargs):
        self.__so_ref__ = vp
        if vp:
            vp.Proxy = self
        if overridden:
            if object.__init__ == overridden:
                overridden(self)
            else:
                overridden(self, vp, *args, **kwargs)
    if overridden:
        __init__.__doc__ = overridden.__doc__
        __init__.__signature__ = inspect.signature(overridden)
    return __init__


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='attach',
        override_error_msg=f"Use {_ON_ATTACH} instead")
def t_view_proxy_attach(overridden: Any, meta: TypeMeta):
    def handler(self, vp):
        assert vp.Proxy == self
        self.__so_ref__ = vp
        self.__so_old__ = dict()
        _call(self, _ON_ATTACH, vp)
        meta.init_properties(self, vp)
        meta.init_display_modes(self, vp)
        meta.apply_extensions(self, vp, _ON_ATTACH)
        _call(self, _ON_START, vp)
    return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='setupContextMenu',
        override_error_msg=f"Use {_ON_CONTEXT_MENU} instead")
def t_view_proxy_ctx_menu(overridden: Any, meta: TypeMeta):
    call = getattr(meta.cls, _ON_CONTEXT_MENU, None)
    if call:
        def handler(self, vp: ViewProviderDocumentObject, menu: QMenu):
            call(self, menu)
        return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='claimChildren',
        override_error_msg=f"Use {_ON_CLAIM_CHILDREN} instead")
def t_view_proxy_claim_children(overridden: Any, meta: TypeMeta):
    call = getattr(meta.cls, _ON_CLAIM_CHILDREN, None)
    if call:
        def handler(self):
            return call(self) or []
        return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='setEdit',
        override_error_msg=f"Use {_ON_EDIT_START} instead")
def t_view_proxy_edit_start(overridden: Any, meta: TypeMeta):
    call = getattr(meta.cls, _ON_EDIT_START, None)
    if call:
        def handler(self, vp: ViewProviderDocumentObject, mode: int = 0):
            return call(self, vp, mode)
        return handler

#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='unsetEdit',
        override_error_msg=f"Use {_ON_EDIT_END} instead")
def t_view_proxy_edit_end(overridden: Any, meta: TypeMeta):
    call = getattr(meta.cls, _ON_EDIT_END, None)
    if call:
        def handler(self, vp: ViewProviderDocumentObject, mode: int = 0):
            return call(self, vp, mode)
        return handler

#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='onDelete',
        override_error_msg=f"Use {_ON_DELETE} instead")
def t_view_proxy_delete(overridden: Any, meta: TypeMeta):
    call = getattr(meta.cls, _ON_DELETE, None)
    if call:
        def handler(self, vp: ViewProviderDocumentObject, sub_elements):
            return call(self, vp, sub_elements)
        return handler

#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='doubleClicked',
        override_error_msg=f"Use {_ON_DBL_CLICK} instead")
def t_view_proxy_dbl_click(overridden: Any, meta: TypeMeta):
    call = getattr(meta.cls, _ON_DBL_CLICK, None)
    if call:
        def handler(self, vp: ViewProviderDocumentObject):
            return call(self, vp)
        return handler

#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='onChanged',
        override_error_msg=f"Use {_ON_CHANGE} instead")
def t_view_proxy_change(overridden: Any, meta: TypeMeta):
    def handler(self, vp: ViewProviderDocumentObject, prop_name: str):
        new_value = getattr(vp, prop_name, None)
        meta.apply_extensions(self, vp, _ON_CHANGE, prop_name)
        prop = meta.property_lookup.get(prop_name, None)
        if prop:
            if prop.observer_func:
                args = (self, new_value)[0:prop.observer_arity]
                prop.observer_func(*args)
        return _call(self, _ON_CHANGE, vp, new_value)
    return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='getIcon',
        override_error_msg=f"Use {_GET_ICON} instead or pass the icon " +
                            "argument in the decorator")
def t_view_proxy_icon(overridden: Any, meta: TypeMeta):
    get_icon = getattr(meta.cls, _GET_ICON, None)
    if get_icon or meta.icon:
        def handler(self):
            if get_icon:
                return _resolve_uri(get_icon(self), meta.base_dir)
            if meta.icon:
                return _resolve_uri(meta.icon, meta.base_dir)
        return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='setDisplayMode',
        override_error_msg=f"Use {_SET_DISPLAY_MODE} instead")
def t_view_proxy_set_dm(overridden: Any, meta: TypeMeta):
    get = getattr(meta.cls, _SET_DISPLAY_MODE, None)
    def handler(self, mode):
        if get:
            return get(self, mode)
        return mode
    return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='getDisplayModes',
        override_error_msg=f"Use {_DISPLAY_MODES} instead")
def t_view_proxy_get_dms(overridden: Any, meta: TypeMeta):
    meta_dm = getattr(meta.cls, _DISPLAY_MODES, None)
    if meta_dm:
        def handler(self, vp: ViewProviderDocumentObject):
            return meta_dm(self, vp)
    else:
        def handler(self, vp: ViewProviderDocumentObject):
            return [dm.name for dm in meta.display_modes.values()]
    return handler

#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='getDefaultDisplayMode',
        override_error_msg=f"Use {_DEFAULT_DISPLAY_MODE} instead")
def t_view_proxy_get_def_dm(overridden: Any, meta: TypeMeta):
    user = getattr(meta.cls, _DEFAULT_DISPLAY_MODE, None)
    if user:
        def handler(self):
            return user(self)
        return handler

    if not meta.display_modes:
        return None
    
    defaults = [dm.name for dm in meta.display_modes.values() if dm.is_default]

    if len(defaults) == 0:
        if len(meta.display_modes) == 1:
            defaults = [dm.name for dm in meta.display_modes.values()]
        else:
            modes = [dm.name for dm in meta.display_modes.values()]
            raise RuntimeError(
                f"One display mode must be declared as default: {modes}")

    if len(defaults) > 1:
        raise RuntimeError(
            f"Only one display mode can be declared as default: {defaults}")

    def handler(self):
        return  defaults[0]
    return handler


#$ ─────────────────────────────────────────────────────────────────────────────
@template(
        name='updateData',
        override_error_msg=f"Use {_ON_OBJECT_CHANGE} instead")
def t_view_proxy_object_change(overridden: Any, meta: TypeMeta):
    call = getattr(meta.cls, _ON_OBJECT_CHANGE, None)
    if call:
        def handler(self, obj: DocumentObject, prop_name: str):
            return call(self, obj, prop_name)
        return handler

#$ ─────────────────────────────────────────────────────────────────────────────
@template(name='Object')
def t_view_proxy_object(overridden: Any, meta: TypeMeta):
    def handler(self) -> DocumentObject:
        '''Returns FreeCAD internal Feature Object'''
        return self.__so_ref__.Object
    return property(handler)


#$ ─────────────────────────────────────────────────────────────────────────────
@template(name='ViewObject')
def t_view_proxy_vobject(overridden: Any, meta: TypeMeta):
    def handler(self) -> ViewProviderDocumentObject:
        '''Returns FreeCAD internal View Provider Object'''
        return self.__so_ref__
    return property(handler)


#; ┌───────────────────────────────────────────────────────────────────────────┐
#; │ Migrations                                                                │
#; └───────────────────────────────────────────────────────────────────────────┘

#% ─────────────────────────────────────────────────────────────────────────────
class MigrationMeta:
    """
    Migrations Meta dada
    """

    def __init__(self, old, current) -> None:
        self.old = old
        self.current = current
        self.inplace = old is current
        
        overridden = getattr(old, 'onDocumentRestored', None)
        onDocumentRestored = t_migrations_onDocumentRestored(overridden, self)
        setattr(old, 'onDocumentRestored', onDocumentRestored)


#@ ─────────────────────────────────────────────────────────────────────────────
def migrations(current=None):
    """
    Install migrations management into the class.

    :param ProxyClass current: most recent class, if omitted, the same class is used.
    """
    def wrapper(cls):
        cls._fp_migrations = MigrationMeta(cls, current or cls)
        return cls
    return wrapper


#$ ─────────────────────────────────────────────────────────────────────────────
def t_migrations_onDocumentRestored(overridden, meta: MigrationMeta):
    '''Proxy.onDocumentRestored override used for migrations'''
    current_version = 1
    proxy_meta: TypeMeta = getattr(meta.current, _SO_META, None)
    if proxy_meta:
        current_version = proxy_meta.version

    def handler(self, fp: DataProxy):
        self.__so_state__ = FeatureState.Restoring
        self.__so_ref__ = fp
        version = getattr(fp, _SO_VERSION, 1)
        if not meta.inplace:
            _migrate_class(self, meta, version, current_version, fp)
        else:
            if version < current_version:
                _migrate_upgrade(self, meta, version, current_version, fp)
            elif version > current_version:
                _migrate_downgrade(self, meta, version, current_version, fp)
        if overridden:
            overridden(self, fp)
    return handler

# Case: migration to a different class 
# ──────────────────────────────────────────────────────────────────────────────
def _migrate_class(self, 
                   meta: MigrationMeta, 
                   version: int, 
                   current_version: int, 
                   fp: DocumentObject):
    old_v_id = f'{meta.old.__name__}:{version}'
    new_v_id = f'{meta.current.__name__}:{current_version}'
    print_log(f"Document contains a different version of {old_v_id}. ",
              f"Current version is {new_v_id}")
    if hasattr(self, _ON_MIGRATE_CLASS):
        _try_migration(self, meta, version, current_version, fp, 
                       lambda: self.on_migrate_class(version, fp))

# Case: migration to a higher version
# ──────────────────────────────────────────────────────────────────────────────
def _migrate_upgrade(self, 
                     meta: MigrationMeta, 
                     version: int, 
                     current_version: int, 
                     fp: DocumentObject):
    old_v_id = f'{meta.current.__name__}:{version}'
    print_log(f"Document contains an older version of {old_v_id}. ",
              f"Current version is {current_version}")
    if hasattr(self, _ON_MIGRATE_UP):
        _try_migration(self, meta, version, current_version, fp, 
                       lambda: self.on_migrate_upgrade(version, fp))

# Case: migration to a lower version
# ──────────────────────────────────────────────────────────────────────────────
def _migrate_downgrade(self, 
                       meta: MigrationMeta, 
                       version: int, 
                       current_version: int, 
                       fp: DocumentObject):
    old_v_id = f'{meta.current.__name__}:{version}'
    print_log(f"Document contains a newer version of {old_v_id}. ",
              f"Current version is {current_version}")
    if hasattr(self, _ON_MIGRATE_DOWN):
        _try_migration(self, meta, version, current_version, fp, 
                       lambda: self.on_migrate_downgrade(version, fp))

# Apply migration logic and report error/status
# ──────────────────────────────────────────────────────────────────────────────
def _try_migration(self, 
                   meta: MigrationMeta, 
                   version: int, 
                   current_version: int, 
                   fp: DocumentObject, 
                   action: Callable):
    old_v_id = f'{meta.old.__name__}:{version}'
    new_v_id = f'{meta.current.__name__}:{current_version}'    
    message = f"from {old_v_id} to {new_v_id}"
    print_log(f"A migration handler was found, attempting to migrate {message}")
    try:
        action()
        new_version = getattr(fp, _SO_VERSION, version)
        if new_version != current_version or not isinstance(fp.Proxy, meta.current):
            print_err(f"!!! Failed to migrate {message}")
            if hasattr(self, _ON_MIGRATE_ERROR):
                self.on_migrate_error(version, fp)
        else:
            print_log(f"Migration completed {message}")
            if hasattr(self, _ON_MIGRATE_COMPLETE):
                self.on_migrate_complete(version, fp)
    except:
        print_err(f"!!! Failed to migrate {message}")
        traceback.print_exc(file=sys.stderr)
        if hasattr(self, _ON_MIGRATE_ERROR):
            self.on_migrate_error(version, fp)


#: ┌───────────────────────────────────────────────────────────────────────────┐
#: │ Dynamic module setup                                                      │
#: └───────────────────────────────────────────────────────────────────────────┘

#: Install Extension Managers
#: ─────────────────────────────────────────────────────────────────────────────
Part_AttachExtensionPython('Part::AttachExtensionPython')
App_LinkBaseExtensionPython('App::LinkBaseExtensionPython')
App_LinkExtensionPython('App::LinkExtensionPython')
ExtensionSupport('App::GeoFeatureGroupExtensionPython')
ExtensionSupport('App::GroupExtensionPython')
ExtensionSupport('App::OriginGroupExtensionPython')
ExtensionSupport('App::SuppressibleExtensionPython')
ExtensionSupport('TechDraw::CosmeticExtensionPython')
ExtensionSupport('Gui::ViewProviderExtensionPython')
ExtensionSupport('Gui::ViewProviderGeoFeatureGroupExtensionPython')
ExtensionSupport('Gui::ViewProviderGroupExtensionPython')
ExtensionSupport('Gui::ViewProviderOriginGroupExtensionPython')
ExtensionSupport('PartGui::ViewProviderAttachExtensionPython')
ExtensionSupport('PartGui::ViewProviderSplineExtensionPython')

#: Define constructors for each property type except for Enumeration
#: ─────────────────────────────────────────────────────────────────────────────
for _property_type in _FC_PROPERTY_TYPES:
    globals()[f"Property{_property_type}"] = \
        _prop_constructor(f"App::Property{_property_type}")
del _property_type

#: Special constructor for Enumeration property type
#: ─────────────────────────────────────────────────────────────────────────────
def PropertyEnumeration(
    enum: Enum,
    name: str = None,
    section: str = 'Data',
    default: Any = None,
    description: str = '',
    mode: PropertyMode = PropertyMode.Default,
    observer_func: Callable = None,        
    link_property: str = None):
    return Property(
        type='App::PropertyEnumeration',
        section=section,
        observer_func=observer_func,
        name=name,
        link_property=link_property,
        default=default,
        description=description,
        mode=mode,
        enum=enum)


#: Special constructor for Enumeration property type
#: ─────────────────────────────────────────────────────────────────────────────
def PropertyOptions(
    options_provider: Callable,
    name: str = None,
    section: str = 'Data',
    default: Any = None,
    description: str = '',
    mode: PropertyMode = PropertyMode.Default,
    observer_func: Callable = None,        
    link_property: str = None):
    return Property(
        type='App::PropertyEnumeration',
        section=section,
        observer_func=observer_func,
        name=name,
        link_property=link_property,
        default=default,
        description=description,
        mode=mode,
        options=options_provider)


# ┌───────────────────────────────────────────────────────────────────────────┐
# │ Minimal set of user utilities                                             │
# └───────────────────────────────────────────────────────────────────────────┘

# ──────────────────────────────────────────────────────────────────────────────
def get_pd_active_body() -> PartDesign_Body:
    """
    Retrieve the active PartDesign Body if any

    :return PartDesign_Body: Active Body
    """
    if Gui.ActiveDocument:
        if Gui.ActiveDocument.ActiveView:
            return Gui.ActiveDocument.ActiveView.getActiveObject("pdbody")

# ──────────────────────────────────────────────────────────────────────────────
def set_pd_shape(fp: DocumentObject, shape: Shape) -> None:
    """
    Prepare the shape for usage in PartDesign and sets `Shape` and `AddSubShape`
    """
    # Validate type
    if not fp.TypeId.startswith('PartDesign::'):
        raise ValueError('Object fp is not a PartDesign Feature')
    
    # Must be attachable
    if not fp.hasExtension('Part::AttachExtensionPython'):
        fp.addExtension('Part::AttachExtensionPython')
        fp.positionBySupport()
    shape.Placement = fp.Placement

    # Manage BaseFeature
    if hasattr(fp, "BaseFeature") and fp.BaseFeature != None:
        if "Subtractive" in fp.TypeId:
            full_shape = fp.BaseFeature.Shape.cut(shape)
        else:
            full_shape = fp.BaseFeature.Shape.fuse(shape)
        full_shape.transformShape(fp.Placement.inverse().toMatrix(), True)
        fp.Shape = full_shape
    else:
        fp.Shape = shape

    # Manage Pattern Feature
    if hasattr(fp,"AddSubShape"):
        shape.transformShape(fp.Placement.inverse().toMatrix(), True)
        fp.AddSubShape = shape    

# ──────────────────────────────────────────────────────────────────────────────
def set_immutable_prop(obj: ObjectRef, name: str, value: Any) -> None:
    """
    Force update a property with Immutable status. It temporarily removes the
    immutable flag, sets the value and restore the flag if required.

    :param ObjectRef obj: remote FreeCAD object
    :param str name: property
    :param Any value: the value
    """

    is_immutable = 'Immutable' in obj.getPropertyStatus(name)
    if is_immutable:
        obj.setPropertyStatus(name, '-Immutable')
    try:
        setattr(obj, name, value)
    finally:
        if is_immutable:
            obj.setPropertyStatus(name, 'Immutable')


# ──────────────────────────────────────────────────────────────────────────────
def get_selection(*args) -> Tuple:
    """
    Returns current selection in specific order and matching specific types.

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
    
    selection = Gui.Selection.getSelection()
    n_sel = len(selection)
    result = [None] * n_args

    def matcher(arg):
        if arg == '*':
            return lambda x: True
        elif isinstance(arg, str):
            return arg.__eq__
        elif isinstance(arg, re.Pattern):
            return arg.fullmatch
        elif isinstance(arg, (tuple, list)):
            return re.compile("|".join(arg)).fullmatch

    matchers = tuple(matcher(arg) for arg in args)
    for obj in selection:
        obj_type = obj.TypeId
        for i in range(n_args):
            if result[i] is None and matchers[i](obj_type):
                result[i] = obj
                break
    
    return all(result), *result

# ──────────────────────────────────────────────────────────────────────────────
def _basic_modal_dlg(
        message: str, 
        title: str = "Message", 
        details: str = None) -> QMessageBox:    
    """
    Build a basic Qt dialog box

    :param str message: summary
    :param str title: box title, defaults to "Message"
    :param str details: expanded message, defaults to None
    :return QMessageBox: qt object for further customization
    """
    diag = QtGui.QMessageBox()
    diag.setIcon(QtGui.QMessageBox.Information)
    diag.setWindowTitle(title)
    diag.setText(message)
    if details:
        diag.setInformativeText(details)
    diag.setWindowModality(QtCore.Qt.ApplicationModal)
    layout = diag.layout()
    spacer = QtGui.QSpacerItem(
        400, 0, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
    layout.addItem(spacer, layout.rowCount(), 0, 1, layout.columnCount())
    return diag


# ──────────────────────────────────────────────────────────────────────────────
def message_box(message: str, title: str = "Message", details: str = None):
    """
    Shows a basic message dialog (modal) if `App.GuiUp` is True, else prints to
    the console.

    :param str message: summary
    :param str title: box title, defaults to "Message"
    :param str details: expandable text, defaults to None
    """
    if App.GuiUp:
        diag = _basic_modal_dlg(message, title, details)
        diag.exec_()
    else:
        print_log(textwrap.dedent(f"""
            -----------------
            # {title}
            ## {message}
            {details}
            -----------------
            """))


# ──────────────────────────────────────────────────────────────────────────────
def confirm_box(message: str, title: str = "Message", details: str = None) -> bool:
    """
    Ask for a confirmation with a basic dialog. Requires App.GuiUp == True

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


#% ─────────────────────────────────────────────────────────────────────────────
class TransactionAbortException(Exception):
    pass

#% ─────────────────────────────────────────────────────────────────────────────
class TransactionCtrl:
    def abort(self):
        raise TransactionAbortException()
    
#@ ─────────────────────────────────────────────────────────────────────────────
@contextlib.contextmanager
def transaction(name: str, doc: Document = None):
    """
    Context manager to execute code in a document transaction (undo/redo)

    :param str name: Name of the transaction in undo/redo stack
    :param Document doc: selected document, defaults to None
    :yield TransactionCtrl: transaction control
    """
    ctrl = TransactionCtrl()
    _doc = doc or App.activeDocument()
    if not _doc:
        raise ValueError('There is not active document')
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
