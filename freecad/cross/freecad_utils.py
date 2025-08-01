"""Functions that could have belonged to FreeCAD."""

from __future__ import annotations

from abc import ABC
from copy import copy
from dataclasses import dataclass
import string
from typing import Any, Iterable, Optional
import sys
import numpy as np
import FreeCAD as fc
import Part

from .utils import true_then_false
from . import wb_constants

try:
    from PySide import QtCore, QtGui
except:
    from PySide2 import QtCore, QtGui


try:
    # For v0.21:
    from addonmanager_utilities import get_python_exe
except (ModuleNotFoundError, ImportError, AttributeError):
    # For v0.22/v1.0:
    from freecad.utils import get_python_exe


if hasattr(fc, 'GuiUp') and fc.GuiUp:
    def tr(text: str) -> str:
        return QtGui.QApplication.translate('cross', text)
else:
    def tr(text: str) -> str:
        return text

# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]


def with_fc_gui() -> bool:
    return hasattr(fc, 'GuiUp') and fc.GuiUp


def get_param(
    group: 'ParamGrp',
    param: str,
    default=None,
    _type=None,
) -> Any:
    """Return a parameter with type checking and default."""
    type_map = {
        'Integer': int,
        'Float': float,
        'Boolean': bool,
        'Unsigned Long': int,
        'String': str,
        int: int,
        float: float,
        bool: bool,
        str: str,
    }

    if group.IsEmpty() and (default is None):
        raise RuntimeError('Parameter {} not found'.format(param))

    if group.IsEmpty():
        return default

    if (_type is not None) and (_type not in type_map):
        raise ValueError('Unkown type')

    # group.GetBool() and similars return a default value of the given type
    # when the parameter is not found, so that we need to go through all
    # parameters to check if the parameter exists.
    for typ_, name, val in group.GetContents():
        if name != param:
            continue
        if (_type is not None) and (type_map[_type] is not type_map[typ_]):
            raise RuntimeError(
                'Parameter found with wrong type: {}'.format(
                typ_,
                ),
            )
        return val
    if default is None:
        raise RuntimeError('Parameter {} not found'.format(param))
    return default


def set_param(
    group: 'ParamGrp',
    param: str,
    value: Any,
) -> None:
    """Return a parameter with type checking and default."""
    fun_map = {
            int: 'SetInt',
            float: 'SetFloat',
            bool: 'SetBool',
            str: 'SetString',
    }

    if type(value) not in fun_map:
        raise ValueError('Unkown type')
    
    value = value.strip()

    getattr(group, fun_map[type(value)])(param, value)


# Adapted from https://github.com/FreeCAD/FreeCAD/blob
#   /fe9ebfc4c5ea5cd26786627434b4158171a80a29/src/Base/Tools.cpp#L155
# Function getIdentifier in Tools.cpp.
# The difference is that an underscore is added if `text` starts with a number,
# whereas getIdentifier replaces the first character.
def get_valid_property_name(text: str) -> str:
    """Return a valid property from any string."""
    # Check for first character whether it's a digit.
    if not text:
        return '_'
    if text[0] in string.digits:
        text = '_' + text
    # Strip illegal chars.
    valids = string.ascii_letters + string.digits
    return ''.join(c if c in valids else '_' for c in text)


def label_or(
        obj: DO,
        alternative: str = 'no label',
) -> str:
    """Return the `Label` or the alternative."""
    if not hasattr(obj, 'Label'):
        return 'not_a_FreeCAD_object'
    return obj.Label if hasattr(obj, 'Label') else alternative


def message(text: str, gui: bool = False) -> None:
    """Inform the user."""
    fc.Console.PrintMessage(text + '\n')
    if gui and fc.GuiUp:
        diag = QtGui.QMessageBox(
            QtGui.QMessageBox.Information,
            wb_constants.WORKBENCH_NAME, text,
        )
        diag.setWindowModality(QtCore.Qt.ApplicationModal)
        diag.exec_()


def warn(text: str, gui: bool = False) -> None:
    """Warn the user."""
    fc.Console.PrintWarning(text + '\n')
    if gui and fc.GuiUp:
        diag = QtGui.QMessageBox(
            QtGui.QMessageBox.Warning,
            wb_constants.WORKBENCH_NAME, text,
        )
        diag.setWindowModality(QtCore.Qt.ApplicationModal)
        diag.exec_()


def error(text: str, gui: bool = False) -> None:
    """Log an error to the user."""
    fc.Console.PrintError(text + '\n')
    if gui and fc.GuiUp:
        diag = QtGui.QMessageBox(
            QtGui.QMessageBox.Critical,
            wb_constants.WORKBENCH_NAME, text,
        )
        diag.setWindowModality(QtCore.Qt.ApplicationModal)
        diag.exec_()


def strip_subelement(sub_fullpath: str) -> str:
    """Return sub_fullpath without the last sub-element.

    A sub-element is a face, edge or vertex.
    Examples:

    - 'Face6' -> ''
    - 'Body.Box001.' -> 'Body.Box001'
    - 'Body.Box001.Face6' -> 'Body.Box001'

    Parameters
    ----------
    - subobject_fullpath: SelectionObject.SubElementNames[i], where
        SelectionObject is obtained with gui.Selection.getSelectionEx('', 0)
        (i.e. not gui.Selection.getSelectionEx()).
        Examples:
        - 'Face6' if you select the top face of a cube solid made in Part.
        - 'Body.Box001.' if you select the tip of a Part->Body->"additive
            primitive" in PartDesign.
        - 'Body.Box001.Face6' if you select the top face of a Part->Body->
            "additive primitive" in PartDesign.

    """
    if (not sub_fullpath) or ('.' not in sub_fullpath):
        return ''
    return sub_fullpath.rsplit('.', maxsplit=1)[0]


def get_subobject_by_name(
        object_: DO,
        subobject_name: str,
) -> Optional[DO]:
    """Return the appropriate object from object_.OutListRecursive."""
    for o in object_.OutListRecursive:
        if o.Name == subobject_name:
            return o


def get_subobjects_by_full_name(
        root_object: DO,
        subobject_fullpath: str,
) -> DOList:
    """Return the list of objects after root_object to the named object.

    The last part of ``subobject_fullpath`` is then a specific vertex, edge or
    face and is ignored.
    So, subobject_fullpath has the form 'name0.name1.Edge001', for example; in
    this case, the returned objects are
    [object_named_name0, object_named_name1].

    Parameters
    ----------
    - root_object: SelectionObject.Object, where SelectionObject is obtained
        with gui.Selection.getSelectionEx('', 0)
        (i.e. not gui.Selection.getSelectionEx()).
    - subobject_fullpath: SelectionObject.SubElementNames[i].
        Examples:
        - 'Face6' if you select the top face of a cube solid made in Part.
        - 'Body.Box001.' if you select the tip of a Part->Body->"additive
            primitive" in PartDesign.
        - 'Body.Box001.Face6' if you select the top face of a Part->Body->
            "additive primitive" in PartDesign.

    """
    objects = []
    names = strip_subelement(subobject_fullpath).split('.')
    subobject = root_object
    for name in names:
        subobject = get_subobject_by_name(subobject, name)
        if subobject is None:
            # This should not append.
            return []
        objects.append(subobject)
    return objects


def add_property(
        obj: DO,
        type_: str,
        name: str,
        category: str,
        help_: str,
        default: Any = None,
) -> tuple[DO, str]:
    """Add a dynamic property to the object.

    Return the `App::FeaturePython` object containing the property and the
    real property name.

    """
    name = get_valid_property_name(name)

    if name not in obj.PropertiesList:
        obj.addProperty(type_, name, category, tr(help_))
        if default is not None:
            setattr(obj, name, default)

    return obj, name


def get_properties_of_category(
        obj: DO,
        category: str,
) -> list[str]:
    """Return the list of properties belonging to the category."""
    properties: list[str] = []
    try:
        for p in obj.PropertiesList:
            if obj.getGroupOfProperty(p) == category:
                properties.append(p)
    except AttributeError:
        return []
    return properties


def is_derived_from(obj: DO, typeid: str, check_instance_doc_obj = True) -> bool:
    """Return True if the object is a object of the given type."""
    if check_instance_doc_obj and not isinstance(obj, DO):
        return False
    return hasattr(obj, 'isDerivedFrom') and obj.isDerivedFrom(typeid)


def has_type(obj: DO, typeid: str) -> bool:
    """Return True if the object has the given type, evaluating also its Proxy.

    Return True if
    - the object is derived from the given type
    - or obj.Proxy.Type is the given type
    - or obj._Type is the given type.

    """
    return (
        is_derived_from(obj, typeid)
        or (
            hasattr(obj, 'Proxy')
            and hasattr(obj.Proxy, 'Type')
            and obj.Proxy.Type == typeid
        )
        # Alternatively: some object have a `_Type` attribute.
        or (
            hasattr(obj, '_Type')
            and obj._Type == typeid
        )
    )


def is_body(obj: DO) -> bool:
    """Return True if the object is a 'Part::Box'."""
    return is_derived_from(obj, 'PartDesign::Body')


def is_box(obj: DO) -> bool:
    """Return True if the object is a 'Part::Box'."""
    return is_derived_from(obj, 'Part::Box')


def is_sphere(obj: DO) -> bool:
    """Return True if the object is a 'Part::Sphere'."""
    return is_derived_from(obj, 'Part::Sphere')


def is_cylinder(obj: DO) -> bool:
    """Return True if the object is a 'Part::Cylinder'."""
    return is_derived_from(obj, 'Part::Cylinder')


def is_mesh(obj: DO) -> bool:
    """Return True if the object is a 'Mesh::Feature'."""
    return is_derived_from(obj, 'Mesh::Feature')


def is_part_feature(obj: DO) -> bool:
    """Return True if the object is a 'Part::Feature'."""
    return is_derived_from(obj, 'Part::Feature')


def is_part(obj: DO) -> bool:
    """Return True if the object is a 'App::Part'."""
    return is_derived_from(obj, 'App::Part')


def is_origin(obj: DO) -> bool:
    """Return True if the object is a 'App::Origin'."""
    return is_derived_from(obj, 'App::Origin')


def is_group(obj: DO) -> bool:
    """Return True if the object is a 'App::DocumentObjectGroup'."""
    return is_derived_from(obj, 'App::DocumentObjectGroup')


def is_container(obj: DO) -> bool:
    """Return True if the object can contain other objects."""
    return is_part(obj) or is_group(obj)


def is_link(obj: DO) -> bool:
    """Return True if the object is a 'App::Link'."""
    return is_derived_from(obj, 'App::Link')

def is_assembly_from_assembly_wb(obj: DO) -> bool:
    """Return True if the object is a 'Assembly::AssemblyObject'."""
    return is_derived_from(obj, 'Assembly::AssemblyObject')

def is_link_to_assembly_from_assembly_wb(obj: DO) -> bool:
    """Return True if the object is a 'Assembly::AssemblyLink'."""
    return is_derived_from(obj, 'Assembly::AssemblyLink')

def is_join_from_assembly_wb(obj: DO) -> bool:
    """Return True if the object is a Joint from default assembly WB."""
    return is_derived_from(obj, 'App::FeaturePython') and hasattr(obj, 'JointType')

def is_grounded_join_from_assembly_wb(obj: DO) -> bool:
    """Return True if the object is a Grounded Joint from default assembly WB."""
    return is_derived_from(obj, 'App::FeaturePython') and hasattr(obj, 'ObjectToGround')

def is_selection_object(obj: DO) -> bool:
    """Return True if the object is a 'Gui::SelectionObject'."""
    return is_derived_from(obj, 'Gui::SelectionObject', check_instance_doc_obj = False)


def get_linked_obj(obj: DO, recursive=True) -> Optional[DO]:
    """Return the linked object or the object itself."""

    if recursive and is_link(obj):
        return get_linked_obj(obj.LinkedObject, recursive)
    else:
        if is_link(obj):
            return obj.LinkedObject
        else:
            return obj


def first_object_with_volume(obj: DO) -> Optional[DO]:
    """Return the first object with positive volume.

    Return the first object with positive volume from part, body, or link
    (deepest linked body or body in part).
    Return None if no child with volume is found.

    """
    linked_obj = get_linked_obj(obj)  # Deepest linked obj.

    return first_object_with_volume_in_deepest_part(linked_obj)


def first_object_with_volume_in_deepest_part(obj):
    if is_part(obj):
        try:
            for part_member in obj.Group:
                if is_part(part_member):
                    return first_object_with_volume_in_deepest_part(part_member)
                elif volume_mm3(part_member) > 0.0:
                    return part_member
        except KeyError:
            # error('Part - ', linked_obj.Label, ' - ', linked_obj.Label, ' - has not solid object')
            pass

    if volume_mm3(obj) > 0.0:
        return obj

    return None



def is_lcs(obj: DO) -> bool:
    """Return True if the object is a 'PartDesign::CoordinateSystem'."""
    return is_derived_from(obj, 'PartDesign::CoordinateSystem')


def has_placement(obj: DO) -> bool:
    """Return True if obj has a Placement."""
    return (
        hasattr(obj, 'Placement')
        and isinstance(obj.Placement, fc.Placement)
    )


def is_same_placement(
        p1: fc.Placement,
        p2: fc.Placement,
        trans_tol: float = 1e-6,
        rot_tol: float = 1e-7,
) -> bool:
    """Return True if both placements represent the same transform."""
    return (
        p2.Base.isEqual(p1.Base, trans_tol)
        and p2.Rotation.isSame(p1.Rotation, rot_tol)
    )


def make_group(
        doc_or_group: [fc.Document | DO],
        name: str,
        visible: bool = True,
) -> DO:
    """Create or retrieve a group."""
    if is_group(doc_or_group):
        doc = doc_or_group.Document
    else:
        doc = doc_or_group
    candidates = doc.getObjectsByLabel(name)
    existing_group = candidates[0] if candidates else None
    if existing_group and is_group(existing_group):
        return existing_group
    group = doc.addObject('App::DocumentObjectGroup', name)
    group.Label = name
    if is_group(doc_or_group):
        doc_or_group.addObject(group)
    if hasattr(group, 'Visibility'):
        group.Visibility = visible
    return group


def add_object(
        container: [fc.Document | DO],
        type_: str,
        name: str,
) -> DO:
    """Create a new object into the container.

    The object's label will be set `name` but, according to your settings in
    FreeCAD, the label will not be set if there's already an object with that
    Label. The object's name might be also different if FreeCAD decides so.

    """
    if is_container(container):
        doc = container.Document
    else:
        doc = container
    if not isinstance(doc, fc.Document):
        raise RuntimeError('First argument is not a Document, a Group, or a Part')
    obj = doc.addObject(type_, name)
    obj.Label = name
    if is_container(container):
        container.addObject(obj)
    return obj


def get_leafs_and_subnames(obj: DO) -> list[tuple[DO, str]]:
    """Return all leaf subobjects and their path.

    Return a list of (object, path), where path (also called subname) can be
    used to retrieve the physical placement of the object, for example with
    `obj.getSubObject(path)`.

    Parameters
    ----------

    - obj: a FreeCAD object that has the attribute `getSubObjects()`.
           If the object doesn't have `getSubObjects()`, it's considered a leaf
           and (obj, '') is returned.

    """
    def get_subobjects_recursive(
            obj: DO,
            subname: str,
    ) -> list[tuple[DO, str]]:
        if (not hasattr(obj, 'getSubObjects')) or (not obj.getSubObjects()):
            # A leaf node.
            return [(obj, subname)]
        outlist: list[tuple[DO, str]] = []
        subnames = obj.getSubObjects()
        for name in subnames:
            o = obj.getSubObjectList(name)[-1]
            outlist += get_subobjects_recursive(o, f'{subname}{name}')

        return outlist

    return get_subobjects_recursive(obj, '')


def validate_types(
        objects: DOList,
        types: list[str],
        respect_order: [bool | list[bool]] = False,
) -> DOList:
    """Sort objects by required types.

    Return a list of objects sorted by the order in `types`.
    If `respect_order` is True, the required type must be in the same order as
    the objects in the input list. If `respect_order` is a list of booleans, it
    must have the same length as `types` and the strict order is only required if
    the corresponding boolean is True.
    Raises a RuntimeError if a listed type has no appropriate object in the input
    list.

    """
    if len(objects) < len(types):
        raise RuntimeError('More types required that the number of objects')
    if isinstance(respect_order, bool):
        respect_order = [respect_order] * len(types)
    if len(respect_order) != len(types):
        raise RuntimeError(
            '`respect_order` must be a boolean or a'
            ' list of booleans with the same length as `types`',
        )
    if not true_then_false(respect_order):
        raise RuntimeError(
            '`respect_order` must be a list of booleans with no'
            ' False after a True',
        )

    copy_of_objects = copy(objects)
    copy_of_types = copy(types)
    objects_of_precise_type: DOList = []
    indexes_of_any: list[int] = []
    for i_in_types, (type_, exact_position) in enumerate(zip(types, respect_order)):
        object_found = False
        any_type = (type_ in [None, 'any', 'Any'])
        i_in_objects = -1
        for j, o in enumerate(copy_of_objects):
            if object_found:
                # Indirectly, next `type_`.
                continue
            if any_type:
                indexes_of_any.append(i_in_types)
                copy_of_types.pop(0)
                object_found = True
                i_in_objects = j
                # Indirectly, next `type_`.
                continue
            if has_type(o, type_):
                objects_of_precise_type.append(o)
                copy_of_objects.remove(o)
                copy_of_types.pop(0)
                object_found = True
                i_in_objects = j
                # Indirectly, next `type_`.
                continue
        if not object_found:
            raise RuntimeError(f'No object of type "{type_}"')
        if (
            exact_position
            and (i_in_types != i_in_objects)
            and (not any_type)
        ):
            if hasattr(o, 'TypeId'):
                raise RuntimeError(
                    f'Object at position {i_in_objects + 1} is not'
                    f' of type "{type_}" but of type "{o.TypeId}"',
                )
            else:
                raise RuntimeError(
                    f'Object at position {i_in_objects + 1} is not'
                    f' of type "{type_}"',
                )
    outlist: DOList = []
    for i in range(len(types)):
        if i in indexes_of_any:
            outlist.append(copy_of_objects.pop(0))
        else:
            outlist.append(objects_of_precise_type.pop(0))
    return outlist


def get_included_files(obj: DO) -> list[fc.Document]:
    """Return the list of files included in the object.

    Return the list of files included in the object, possibly including
    `obj.Document`.

    """
    docs: list[fc.Document] = []
    for subobj, subname in get_leafs_and_subnames(obj):
        if not hasattr(subobj, 'Document'):
            continue
        if not is_link(subobj):
            continue
        if subobj.Document not in docs:
            docs.append(subobj.Document)
    return docs


def includes_external_files(obj: DO) -> bool:
    """Return True if the object includes external files."""
    included_files = get_included_files(obj)
    return (len(included_files) > 0) and (obj.Document not in included_files)


class ProxyBase(ABC):
    """A base class for proxies of dynamic (scripted) objects in FreeCAD."""

    def __init__(self, object_name: str, properties: list[str]):
        # Name of the attribute being the FreeCAD object.
        self._object_name: str = object_name

        # List of properties that the FreeCAD object must have to be ready to
        # execute.
        self._properties: list[str] = properties

    def is_execute_ready(self, debug=False) -> bool:
        """Return True if the object and all properties are defined.

        Return True if `self` has the attribute `self._object_name` and
        `self._object_name` has all attributes given in `self._properties`.

        If `debug` is True, print a warning if the object is missing or the
        name of the missing property.

        """
        if not hasattr(self, '_object_name'):
            if debug:
                warn('Attribute "_object_name" not found in `self`')
            return False
        try:
            obj = getattr(self, self._object_name)
        except AttributeError:
            if debug:
                warn(f'Attribute "{self._object_name}" not found in `self`')
            return False
        for p in self._properties:
            if not hasattr(obj, p):
                if debug:
                    warn(f'Attribute "{p}" not found in "self.{self._object_name}"')
                return False
        return True

    def update_prop(
            self,
            prop: str,
            value: Any,
            tolerance: float | None = None,
            debug=False,
    ) -> None:
        """Update a property of the object if needed."""
        if not hasattr(self, '_object_name'):
            if debug:
                warn('Attribute "_object_name" not found in `self`')
            return
        try:
            obj = getattr(self, self._object_name)
        except AttributeError:
            if debug:
                warn(f'Attribute "{self._object_name}" not found in `self`')
            return
        if not hasattr(obj, prop):
            if debug:
                warn(f'Attribute "{prop}" not found in "self.{self._object_name}"')
            return
        old_value = getattr(obj, prop)
        if old_value != value:
            if debug:
                # Get the name from the object or from `obj.Object` if `obj` is
                # a view object.
                name = f'{obj.Name}' if hasattr(obj, 'Name') else f'{obj.Object.Name}'
                label = f'{obj.Label}' if hasattr(obj, 'Label') else f'{obj.Object.Label}'
                message(f'{name}.{prop} = {value:.15f} was {old_value:.15f} ({label})')
            if type(old_value) is float:
                if (
                        (tolerance is None)
                        or (abs(old_value - value) > tolerance)
                ):
                    setattr(obj, prop, value)
            else:
                setattr(obj, prop, value)


def convert_units(
        value: float,
        from_: str,
        to_: str,
) -> float:
    """Convert a value from one unit to another.

    >>> convert_units(1, 'm', 'mm')
    1000.0
    >>> convert_units(90, 'deg', 'rad')
    1.5707963267948966
    """
    # As of 2023-08-31 (0.21.1.33694) `float` must be used as workaround
    # Cf. https://forum.freecad.org/viewtopic.php?t=82905.
    return float(fc.Units.Quantity(value, from_).getValueAs(to_))


def quantity_as(
        q: fc.Units.Quantity,
        to_: str,
) -> float:
    """Convert a quantity to another unit.

    >>> convert_quantity(fc.Units.Quantity('1 m'), 'mm')
    1000.0
    >>> convert_quantity(fc.Units.Quantity('90 deg'), 'rad')
    1.5707963267948966

    """
    # As of 2023-08-31 (0.21.1.33694) `float` must be used as workaround
    # Cf. https://forum.freecad.org/viewtopic.php?t=82905.
    return float(q.getValueAs(to_))


def unit_type(
        v: [str | fc.Units.Quantity],
) -> str:
    """Return the unit type of a value.

    return fc.Units.Quantity(v).Unit.Type, e.g. Length, Angle, etc.

    >>> unit_type('1 mm')
    Length
    >>> unit_type('m')
    Length
    >>> unit_type('deg')
    Angle
    >>> unit_type(fc.Units.Quantity('mm*mm'))
    Area

    """
    return fc.Units.Quantity(v).Unit.Type


@dataclass
class Material:
    """A class to store material data."""
    # Absolute path to the material card (*.FCMat).
    card_path: str
    material_name: Optional[str] = None
    density: Optional[fc.Units.Quantity] = None


def material_from_material_editor(
        card_path: str,
) -> Material:
    """Return the material data from the Material Editor

    Return the material data from the Material Editor
    (FEM -> Model -> Materials -> Material Editor).

    """
    material = Material(card_path)
    material.card_path = card_path
    import MaterialEditor
    material_editor = MaterialEditor.MaterialEditor(card_path=card_path)
    try:
        material.material_name = material_editor.cards[material_editor.card_path]
        material.density = fc.Units.Quantity(
                material_editor.materials[material_editor.card_path]['Density'],
        )
    except (KeyError, AttributeError):
        material.material_name = None
        material.density = None
    return material


def fc_matrix_to_numpy(matrix: fc.Matrix):
    """
    Converts a FreeCAD matrix to a numpy array

    Args:
        matrix (Base.Matrix): Matrix from FreeCAD

    Returns:
        np.ndarray: Matrix as a numpy array
    """
    # Extract the components of the inertia matrix
    Ixx, Ixy, Ixz = matrix.A11, matrix.A12, matrix.A13
    Iyx, Iyy, Iyz = matrix.A21, matrix.A22, matrix.A23
    Izx, Izy, Izz = matrix.A31, matrix.A32, matrix.A33

    # Create a numpy array from the inertia matrix components
    inertia_matrix = np.array([
        [Ixx, Ixy, Ixz],
        [Iyx, Iyy, Iyz],
        [Izx, Izy, Izz],
    ])

    return inertia_matrix


def numpy_to_fc_matrix(np_array):
    """
    Converts a numpy array to a FreeCAD matrix

    Args:
        np_array (np.ndarray): Matrix as a numpy array

    Returns:
        Base.Matrix: Matrix as a FreeCAD Base.Matrix object
    """
    # Check if the input is a 3x3 numpy array
    if np_array.shape!= (3, 3):
        raise ValueError("Input must be a 3x3 numpy array")

    # Create a new Base.Matrix object
    matrix = fc.Matrix()

    # Set the components of the matrix
    matrix.A11, matrix.A12, matrix.A13 = np_array[0, 0], np_array[0, 1], np_array[0, 2]
    matrix.A21, matrix.A22, matrix.A23 = np_array[1, 0], np_array[1, 1], np_array[1, 2]
    matrix.A31, matrix.A32, matrix.A33 = np_array[2, 0], np_array[2, 1], np_array[2, 2]

    return matrix


def parallel_axis_theorem(m, r, I):
    """
    Apply the parallel axis theorem to the inertia tensor.

    Parameters:
    m (float): Mass of the body.
    r (numpy.array): Radius vector from the new reference point to the old reference point.
    I (numpy.array): Inertia tensor relative to the old reference point.

    Returns:
    numpy.array: Inertia tensor relative to the new reference point.
    """
    E = np.eye(3)  # Identity matrix
    r_outer = np.outer(r, r)  # Outer product of r with itself
    I = fc_matrix_to_numpy(I)
    I_new = I + m * (np.linalg.norm(r)**2 * E - r_outer)
    return numpy_to_fc_matrix(I_new)


def matrix_of_inertia(
        obj: Optional[fc.DocumentObject],
) -> Optional[fc.Matrix]:
    """Return the matrix of inertia of the given object assuming a density of 1."""
    try:
        return obj.Shape.MatrixOfInertia
    except (AttributeError, IndexError, RuntimeError):
        pass
    try:
        check_solid_exist = obj.Shape.Solids[0]
        commonMatrixOfInertia = fc.Matrix()
        for solid in obj.Shape.Solids:
            if solid.Volume > 0.0:
                r = solid.CenterOfGravity - obj.Shape.CenterOfGravity
                commonMatrixOfInertia += parallel_axis_theorem(solid.Volume, r, solid.MatrixOfInertia)
        return commonMatrixOfInertia
    except (AttributeError, IndexError, RuntimeError):
        pass
    return None


def correct_matrix_of_inertia(
        matrix_of_inertia: fc.Matrix,
        volume_mm3: float,
        mass: float,
) -> fc.Matrix:
    # convert matrix of inertia considering mass

    # Looks freecad uses mass = volume and therefore default density is 1
    # my formula for correction of matrix_of_inertia is:
    # matrix_of_inertia / volume (because it equal mass) * real_mass

    # Formula works but with wrong scale. I entered this ratio for correct scale.
    # If you can rewrite formula without ratio do plz.
    ratio_for_correct_scale = 1 / 1e6
    # Implementation note fc.Matrix doesn't support division by a scalar.
    return matrix_of_inertia * mass * (1.0 / volume_mm3) * ratio_for_correct_scale


def volume_mm3(
        obj: Optional[fc.DocumentObject],
) -> Optional[float]:
    """Return the volume of the given object in mm³."""
    try:
        if obj.TypeId == 'PartDesign::CoordinateSystem':
            return 0
    except (AttributeError, IndexError, RuntimeError):
        pass

    try:
        return obj.Shape.Volume
    except (AttributeError, IndexError, RuntimeError):
        pass

    try:
        return obj.Shape.Solids[0].Volume
    except (AttributeError, IndexError, RuntimeError):
        pass

    return 0


def center_of_gravity_mm(
        obj: Optional[fc.DocumentObject],
) -> Optional[fc.Vector]:
    """Return the center of gravity (aka center of mass) of the object in mm."""
    try:
        return obj.Shape.CenterOfGravity
    except (AttributeError, IndexError, RuntimeError):
        pass
    try:
        return obj.Shape.Solids[0].CenterOfGravity
    except (AttributeError, IndexError, RuntimeError):
        pass
    return None


def lcs_attachmentsupport_name():
    """Return LCS AttachmentSupport method name dependent of FreeCAD version."""
    fc_version_info = fc.Version()
    fc_version_info[1]
    fc_version_info[2]
    if(int(fc_version_info[1]) < 22 and int(fc_version_info[0]) == 0):
        return 'Support'
    else:
        return 'AttachmentSupport'


def get_python_name() -> str:
    return 'python' + str(sys.version_info.major) + '.' + str(sys.version_info.minor)


def get_parents_names(obj: fc.DocumentObject) -> list[str]:
    """Get object parents names"""
    parents_names = []
    for parent, path in obj.Parents:
        name = parent.Name
        if name is None:
            name = 'None'
        parents_names = [name] + path.split('.')
        break
    return parents_names


def copy_obj_geometry(old_obj: DO, new_obj: DO, copy_compound_shape_for_part: bool = True) -> DO:
    """ Copy geometry properties from old object to new one"""
    if hasattr(old_obj, "Shape"):
        new_obj.Shape = old_obj.Shape
    elif hasattr(old_obj, "Mesh"):      # upgrade with wmayer thanks #http://forum.freecadweb.org/viewtopic.php?f=13&t=22331
        new_obj.Shape = old_obj.Mesh
    elif hasattr(old_obj, "Points"):
        new_obj.Shape = old_obj.Points
    elif copy_compound_shape_for_part and is_part(old_obj):
        # get compound shape for all objects inside Part
        new_obj.Shape = Part.getShape(old_obj)

    return new_obj


def get_compound(
        objs: list[DO],
        compound_placement: fc.Placement,
        compound_name: str,
        compound_el_name: str,
        zeroing_real_objs_placement: bool = False,
) -> Optional[DO]:
    """Make compound of objects and it`s nested bodies"""
    try:
        doc = objs[0].Document
    except:
        error('Can`t get document for make compound of objects')
        return None
    part_tmp = doc.addObject("App::Part", 'tmp_' + compound_name)
    part_tmp.Visibility = False
    for obj in objs:
        compound_el = doc.addObject("Part::Feature", compound_el_name)
        part_tmp.Visibility = False
        compound_el = copy_obj_geometry(obj, compound_el)
        if zeroing_real_objs_placement:
            compound_el.Placement = fc.Placement()
        part_tmp.addObject(compound_el)
    # make result compound of Part (inside it compounds of objs)
    compound = doc.addObject("Part::Feature", compound_name)
    compound.Visibility = False
    compound = copy_obj_geometry(part_tmp, compound)
    compound.Placement = compound_placement
    part_tmp.removeObjectsFromDocument()
    doc.removeObject(part_tmp.Name)
    return compound
