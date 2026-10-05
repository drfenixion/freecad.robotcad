from typing import Optional, Callable, Any, Tuple
import numpy as np
import numpy.linalg as la
from numpy.typing import NDArray, ArrayLike

import FreeCAD as fc


# Console output levels accepted by calculate()'s `output_level` argument.
# LOG_LEVEL_WARN suppresses LOG_LEVEL_INFO messages (the tree-walk trace);
# warnings are always printed.
LOG_LEVEL_INFO = 'info'
LOG_LEVEL_WARN = 'warn'

# Property group shown in the FreeCAD property panel for every property the
# "Override inertial properties" command adds (see
# command_override_inertial_properties.py, which imports the names below
# rather than redefining them, since it depends on this module and not the
# other way around). Kept separate from the "Inertial Parameters" group
# used by CROSS Link/Robot's own Mass/CenterOfMass/Ixx..Izz properties, so
# the two never mix on the same object, and so future, non-inertia-related
# overrides can be added to the same "RobotCad" group later.
PROPERTY_GROUP = 'RobotCad'

# Override mode enumeration values, in the order shown in the property
# panel. OVERRIDE_MODE_NONE is first so it is selected by default when the
# property is first added to an object.
OVERRIDE_MODE_NONE = 'none'
OVERRIDE_MODE_DENSITY = 'density'
OVERRIDE_MODE_MASS = 'mass'
OVERRIDE_MODE_MANUAL = 'manual'
OVERRIDE_MODE_IGNORE = 'ignore'
OVERRIDE_MODES = [
    OVERRIDE_MODE_NONE,
    OVERRIDE_MODE_DENSITY,
    OVERRIDE_MODE_MASS,
    OVERRIDE_MODE_MANUAL,
    OVERRIDE_MODE_IGNORE,
]

PROP_OVERRIDE_MODE = 'RobotCadOverrideMode'
PROP_MASS = 'RobotCadMass'
PROP_DENSITY = 'RobotCadDensity'
PROP_CENTER_OF_MASS = 'RobotCadCenterOfMass'
PROP_IXX = 'RobotCadIxx'
PROP_IXY = 'RobotCadIxy'
PROP_IXZ = 'RobotCadIxz'
PROP_IYY = 'RobotCadIyy'
PROP_IYZ = 'RobotCadIyz'
PROP_IZZ = 'RobotCadIzz'

# All override properties except the mode enumeration itself, in the order
# they are added to an object / shown in the property panel.
OVERRIDE_VALUE_PROPERTY_NAMES = (
    PROP_MASS, PROP_DENSITY, PROP_CENTER_OF_MASS,
    PROP_IXX, PROP_IXY, PROP_IXZ, PROP_IYY, PROP_IYZ, PROP_IZZ,
)


def print_matrix(M: np.ndarray) -> None:
    """ Print matrix """
    for i in range(M.shape[0]):
        for j in range(M.shape[1]):
            print('%20.4f' % M[i, j], end=', ')
        print()


def rotation_matrix3(rot: fc.Rotation) -> NDArray:
    """ Return the 3x3 rotation matrix (numpy) equivalent to an fc.Rotation. """
    M = rot.toMatrix()
    return np.array([[M.A11, M.A12, M.A13],
                      [M.A21, M.A22, M.A23],
                      [M.A31, M.A32, M.A33]])


def screw_tf_matrix(T: fc.Matrix) -> NDArray:
    """ Construct screw transform matrix from 4x4 transform matrix """
    # rotation matrix
    R = np.array([[T.A11, T.A12, T.A13],
                  [T.A21, T.A22, T.A23],
                  [T.A31, T.A32, T.A33]])
    # scew-symmetric matrix
    S = np.array([[   0.0, -T.A34,  T.A24],
                  [ T.A34,    0.0, -T.A14],
                  [-T.A24,  T.A14,    0.0]])
    # screw transform matrix
    return np.block([[      R, np.zeros((3,3)) ],
                     [  S @ R,               R ]])


def inertia_matrix6d(J: fc.Matrix, Pc: fc.Vector, m: float) -> ArrayLike:
    """ Construct generalized 6x6 rigid body inertia matrix.

    Arguments
    ---------
    J: FreeCAD.Matrix
        Inertia matrix in center of mass.
    Pc: FreeCAD.Vector
        Center of mass location.
    m: float
        Mass.

    Return
    ------
    ArrayLike
        6x6 generalized inertia matrix of rigid body.
    """
    # inertia matrix
    J = np.array([[J.A11, J.A12, J.A13],
                  [J.A21, J.A22, J.A23],
                  [J.A31, J.A32, J.A33]])
    # scew-symmetric matrix
    S = np.array([[  0.0, -Pc.z,    Pc.y],
                  [  Pc.z,    0.0, -Pc.x],
                  [ -Pc.y, Pc.x,    0.0]])
    # rigid body inertia tensor
    mS = m * S
    return np.block([[J - mS @ S,            mS],
                     [      mS.T, m * np.eye(3)]])


def mass_com_inertia(JJ: np.ndarray) -> Tuple[float, fc.Vector, fc.Matrix]:
    """ Extract the inertia matrix, center of mass location and mass from a generalized inertia matrix. """
    m = (JJ[3,3] + JJ[4,4] + JJ[5,5]) / 3.0
    cog = 0.5 * fc.Vector(-JJ[1,5] - JJ[5,1], JJ[0,5] + JJ[5,0], -JJ[0,4] - JJ[4,0]) / m
    R = fc.Matrix()
    for i in range(3):
        R.setRow(i, fc.Vector(JJ[0,0:3]))
    return m, cog, R


def get_property(obj: fc.DocumentObject, name: str, ptype: Optional[str] = None, units: Optional[str] = None, default: Optional[Any] = None, check_func: Optional[Callable[[Any], bool]] = None, check_desc: str = '') -> Any:
    """ Get FreeCAD object property by name

    Arguments
    ---------
    name: str
        Property name.
    ptype: str
        Property FreeCAD type ID.
    default: Any, optional
        Do not raise exception on missing property and return this value.
    check_func: Callable, optional.
        Check function for property value.
    check_desc: str, default ''
        Error details if check has failed.

    Returns
    -------
    Any
        Property value.

    """
    # check that property exists
    if name not in obj.PropertiesList:
        if default is None:
            raise KeyError(f'Object {obj.Label} ({obj.FullName}) must contain "{name}" property.')
        else:
            return default
    # check property type
    if obj.getTypeIdOfProperty(name) != ptype:
        raise TypeError(f'Object {obj.Label} ({obj.FullName}) property "{name}" type must be {ptype}.')
    # get property value
    p = obj.getPropertyByName(name)
    if units is not None:
        value = p.getValueAs(units).Value
    elif isinstance(p, fc.Units.Quantity):
        value = p.Value
    else:
        value = p
    # check value
    if check_func is not None and not check_func(value):
        raise ValueError(f'Object {obj.Label} ({obj.FullName}) property "{name}" value check failed: {check_desc}')
    # return result
    return value


def get_property_optional(obj: fc.DocumentObject, name: str, ptype: str, units: Optional[str] = None, check_func: Optional[Callable[[Any], bool]] = None, check_desc: str = '') -> Optional[Any]:
    """ Get FreeCAD object property by name if it is present.

    Arguments
    ---------
    name: str
        Property name.
    ptype: str
        Property FreeCAD type ID.
    check_func: Callable, optional.
        Check function for property value.
    check_desc: str, default ''
        Error details if check has failed.

    Returns
    -------
    Any
        Property value or None.

    """
    # check that property exists
    if name not in obj.PropertiesList:
        return None
    else:
        return get_property(obj, name, ptype, units=units, check_func=check_func, check_desc=check_desc)


def calculate(
        obj: fc.DocumentObject,
        ignore_material: bool = False,
        ignore_placement: bool = False,
        parent_material = None,
        level: int = 0,
        output_level: str = LOG_LEVEL_INFO,
):
    ''' Calculate the 6x6 inertia matrix of an object.

    Walks along the object tree, finds solid objects with volume and calculates the overall inertia matrix using material properties.
    Default behavior can be overridden by specifying object properties. These properties are added by the
    "Override inertial properties" command (see command_override_inertial_properties.py) and are not
    present on an object unless that command was run on it.

    Object properties
    -----------------
    RobotCadOverrideMode: App::PropertyEnumeration, optional
        Property specifies calculation mode for this object. If not present, 'none' mode is used.
        Possible values:
        * 'none': Use the solid's shape and material to calculate inertia. This mode can be overridden in subobjects.
        * 'density': Material is ignored. RobotCadDensity is used for this object. Override properties
            in child objects are ignored.
        * 'mass': Material is ignored. Shape density is assumed uniform and RobotCadMass is known.
            Override properties in child objects are ignored.
        * 'manual': Object mass, center of mass and inertia matrix are provided manually via
            RobotCadMass, RobotCadCenterOfMass and RobotCadIxx..RobotCadIzz. Override properties
            in child objects are ignored.
        * 'ignore': This object and its whole subtree are not processed at all (not even to
            find linked/contained/solid objects). The generalized inertia matrix for this object
            is assumed zero, as if it had neither mass nor volume. Override properties in child
            objects are ignored, since the child objects themselves are never visited.
    RobotCadMass: App::PropertyMass, optional
        Mass for 'mass' and 'manual' override mode.
    RobotCadDensity: App::PropertyDensity, optional
        Density for 'density' override mode.
    RobotCadCenterOfMass: App::PropertyPlacement, optional
        Center of gravity coordinates for 'manual' mode (Base, in mm). The placement's Rotation
        gives the orientation of the principal axes of inertia that RobotCadIxx..RobotCadIzz below
        are expressed in, relative to this object's local frame.
    RobotCadIxx, RobotCadIxy, RobotCadIxz, RobotCadIyy, RobotCadIyz, RobotCadIzz: App::PropertyFloat, optional
        Components of the 3x3 inertia matrix relative to center of gravity for 'manual' mode, in kg*mm^2
        (FreeCAD's native inertia unit, same as Shape.MatrixOfInertia), expressed in the frame given by
        RobotCadCenterOfMass.Rotation.

    Arguments
    ---------
    obj: DocumentObject
        Root object.
    ignore_material: bool, default False
        Ignore material properties throughout the subtree. In this case density is assumed equal to identity (1).
    ignore_placement: bool, default False
        Ignore root object placement. Inertia matrix is calculated in the object's local frame.
    parent_material: Material, optional
        Material to use if ShapeMaterial is not provided or its name is 'Default'.
    level: int, default 0
        Level in the tree of objects. Used to compute the debug output indentation interval.
    output_level: str, default LOG_LEVEL_INFO
        Console verbosity: LOG_LEVEL_INFO prints both the trace of the tree walk and warnings;
        LOG_LEVEL_WARN prints only warnings. Passed unchanged to recursive calls.

    Return
    ------
    np.ndarray
        6x6 inertia matrix in coordinat frame of root object container (ignore_placement = False)
		or in local coordinate frame of root object (ignore_placement = True). 
		.

    '''

    print_leader = '  ' * level

    def log(msg_level: str, message: str) -> None:
        """ Print `message` through FreeCAD's Console, honoring `output_level`.

        `msg_level` (LOG_LEVEL_INFO or LOG_LEVEL_WARN) is this message's own
        severity; it is only printed when `output_level` is verbose enough
        to include it. `print_leader` is preserved in the output: it
        encodes the recursion depth and is what gives the console log its
        tree-shaped structure.
        """
        if msg_level == LOG_LEVEL_INFO and output_level != LOG_LEVEL_INFO:
            return
        line = f'{msg_level.upper()}:{print_leader} {message}'
        if msg_level == LOG_LEVEL_WARN:
            fc.Console.PrintWarning(line + '\n')
        else:
            fc.Console.PrintMessage(line + '\n')

    #
    # process properties
    #
    override_mode = get_property_optional(
        obj, PROP_OVERRIDE_MODE, 'App::PropertyEnumeration',
        check_func=lambda v: v in OVERRIDE_MODES,
        check_desc=f'value must be one of {OVERRIDE_MODES}',
    )
    # get override-mode parameters
    if override_mode is None:
        for name in OVERRIDE_VALUE_PROPERTY_NAMES:
            if name in obj.PropertiesList:
                log(LOG_LEVEL_WARN, f'object {obj.Label} ({obj.FullName}) property "{name}" is supplied but property "{PROP_OVERRIDE_MODE}" is missing.')
        override_mode = OVERRIDE_MODE_NONE
    elif override_mode == OVERRIDE_MODE_MASS:
        override_mass = get_property(obj, PROP_MASS, 'App::PropertyMass', units='kg')
    elif override_mode == OVERRIDE_MODE_DENSITY:
        override_density = get_property(obj, PROP_DENSITY, 'App::PropertyDensity', units='kg/mm^3')
    elif override_mode == OVERRIDE_MODE_MANUAL:
        ixx = get_property(obj, PROP_IXX, 'App::PropertyFloat')
        ixy = get_property(obj, PROP_IXY, 'App::PropertyFloat')
        ixz = get_property(obj, PROP_IXZ, 'App::PropertyFloat')
        iyy = get_property(obj, PROP_IYY, 'App::PropertyFloat')
        iyz = get_property(obj, PROP_IYZ, 'App::PropertyFloat')
        izz = get_property(obj, PROP_IZZ, 'App::PropertyFloat')
        com_placement = get_property(obj, PROP_CENTER_OF_MASS, 'App::PropertyPlacement')
        m = get_property(obj, PROP_MASS, 'App::PropertyMass', units='kg')

        # assembly rigid body inertia matrix
        JJ = np.zeros((6,6))
        JJ[0, 0:3] = ixx, ixy, ixz
        JJ[1, 0:3] = ixy, iyy, iyz
        JJ[2, 0:3] = ixz, iyz, izz
        JJ[3:6, 3:6] = m*np.eye(3)

        # check rotational inertia matrix: positive semidefinite, triangle ineq.
        J = JJ[0:3, 0:3]
        principal_moments = la.eigvalsh(J)
        if (principal_moments < 0).any():
            raise ValueError(
                f'Object {obj.Label} ({obj.FullName}) properties "{PROP_IXX}".."{PROP_IZZ}"'
                ' value check failed: matrix must be positive semi-definite'
                ' (no negative principal moment of inertia).',
            )
        lam_min, lam_mid, lam_max = np.sort(principal_moments)
        tolerance = 1.0e-9 * max(1.0, abs(lam_max))
        if lam_max > lam_mid + lam_min + tolerance:
            raise ValueError(
                f'Object {obj.Label} ({obj.FullName}) properties "{PROP_IXX}".."{PROP_IZZ}"'
                ' value check failed: principal moments of inertia must satisfy the triangle'
                ' inequality (the largest must not exceed the sum of the other two).',
            )

        # perform transform from inertia frame to object frame
        X = screw_tf_matrix(com_placement.Matrix.inverse())
        JJ = X.T @ JJ @ X
        # now apply object transform
        if not ignore_placement and hasattr(obj, 'Placement'):
            Y = screw_tf_matrix(obj.Placement.Matrix.inverse())
            JJ = Y.T @ JJ @ Y
        # return result: child objects (if any exists) are ignored
        return JJ

    elif override_mode == OVERRIDE_MODE_IGNORE:
        # This object (subtree) is excluded from the calculation entirely.
        log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): ignore, subtree not processed, inertia assumed zero.')
        return np.zeros((6,6))

    # determine calculation mode
    if ignore_material:
        calculate_volume = True
        override_mode = OVERRIDE_MODE_NONE
    else:
        calculate_volume = override_mode != OVERRIDE_MODE_NONE
    
    # determine material
    material = parent_material
    material_is_inherited = True
    # get object material
    if hasattr(obj, 'ShapeMaterial') and obj.ShapeMaterial.Name != 'Default':
        material = obj.ShapeMaterial
        material_is_inherited = False

    #
    # link objects
    #
    linked_obj = obj.getLinkedObject()
    if linked_obj is not obj:
        # logging
        log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): link, ignore_material: {ignore_material}, override_mode: {override_mode}.')
        # calculate inertia of linked object
        JJ = calculate(linked_obj, ignore_material=calculate_volume, ignore_placement=not obj.LinkTransform, parent_material=material, level=level + 1, output_level=output_level)
        # apply link transform
        if not ignore_placement:
            X = screw_tf_matrix(obj.Placement.Matrix.inverse())
            JJ = X.T @ JJ @ X
        # override material
        if override_mode == OVERRIDE_MODE_DENSITY:
            JJ *= override_density
        elif override_mode == OVERRIDE_MODE_MASS:
            mass = (JJ[3,3] + JJ[4,4] + JJ[5,5]) / 3.0
            if mass > 0.0:
                JJ *= override_mass / mass
            else:
                log(LOG_LEVEL_WARN, f'object {obj.Label} ({obj.FullName}) does not have volume.')
        # return result
        return JJ

    #
    # object-containers
    #
    subnames = obj.getSubObjects()
    if len(subnames) > 0:
        # logging
        log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): container, ignore_material: {ignore_material}, override_mode: {override_mode}.')
        # process all subobjects
        JJ = np.zeros((6,6))
        for subname in subnames:
            # get subobject and calculate its inertia
            subobj = obj.getSubObject(subname, 1)
            JJ += calculate(subobj, ignore_material=calculate_volume, ignore_placement=False, parent_material=material, level=level + 1, output_level=output_level)
        # apply container transform
        if not ignore_placement and hasattr(obj, 'Placement'):
            X = screw_tf_matrix(obj.Placement.Matrix.inverse())
            JJ = X.T @ JJ @ X
        # override material
        if override_mode == OVERRIDE_MODE_DENSITY:
            JJ *= override_density
        elif override_mode == OVERRIDE_MODE_MASS:
            mass = (JJ[3,3] + JJ[4,4] + JJ[5,5]) / 3.0
            if mass > 0:
                JJ *= override_mass / mass
            else:
                log(LOG_LEVEL_WARN, f'object {obj.Label} ({obj.FullName}) does not have volume.')
        # return result
        return JJ

    #
    # Solid objects
    #
    if hasattr(obj, 'Shape') and not obj.Shape.isNull():
        shape = obj.Shape
        # determine shape type and extract inertia
        if shape.ShapeType == 'Solid':
            # single solid body
            JJ = inertia_matrix6d(shape.MatrixOfInertia, shape.CenterOfMass, shape.Mass)
            volume = shape.Volume
        elif shape.ShapeType == 'Compound':
            # compound object
            JJ = np.zeros((6,6))
            volume = 0.0
            for solid in shape.Solids:
                JJ += inertia_matrix6d(solid.MatrixOfInertia, solid.CenterOfMass, solid.Mass)
                volume += solid.Volume
        else:
            # object does not have volume
            log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}) does not have volume/mass.')
            return np.zeros((6,6))
        # reverse object placement
        if ignore_placement:
            X = screw_tf_matrix(obj.Placement.Matrix)
            JJ = X.T @ JJ @ X
        # determine volume
        mass = (JJ[3,3] + JJ[4,4] + JJ[5,5]) / 3.0
        if mass > 0.0:
            JJ *= volume / mass
        # material
        if override_mode == OVERRIDE_MODE_NONE and not calculate_volume:
            # apply material settings
            if material is None:
                material_name = obj.ShapeMaterial.Name if hasattr(obj, 'ShapeMaterial') else 'None'
                log(LOG_LEVEL_WARN, f'object {obj.Label} ({obj.FullName}) material is "{material_name}", inherited material is "None", assuming zero mass.')
                JJ *= 0.0
            else:
                inherit_text = '(inherited)' if material_is_inherited else ''
                # get material density
                try:
                    density = fc.Units.Quantity(material.PhysicalProperties['Density']).getValueAs('kg/mm^3').Value
                except (KeyError, AttributeError, ValueError) as e:
                    log(LOG_LEVEL_WARN, f'object {obj.Label} ({obj.FullName}) material "{material.Name}" {inherit_text} does not have "Density" property ({e}), assume zero mass.')
                    JJ *= 0.0
                else:
                    # apply material density
                    JJ *= density
                    # logging
                    log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): solids/compound, material: {material.Name} {inherit_text}, density: {1000*density} g/mm^3, mass: {1000*JJ[3,3]} g.')
        elif override_mode == OVERRIDE_MODE_DENSITY:
            # override density
            JJ *= override_density
            # logging
            log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): solids/compound, override_density: {1000*override_density} g/mm^3, mass: {1000*JJ[3,3]} g')
        elif override_mode == OVERRIDE_MODE_MASS:
            # override mass
            if volume > 0.0:
                JJ *= override_mass / volume
                # logging
                log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): solids/compound, override_mass: {1000*override_mass} g.')
            else:
                log(LOG_LEVEL_WARN, f'object {obj.Label} ({obj.FullName}) does not have volume.')
        else:
            # volume calculation, do nothing
            log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): solids/compound, volume: {volume} mm^3.')

        # return result
        return JJ

    #
    # Other objects
    #
    log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): unknown type without volume/mass.')
    return np.zeros((6,6))
