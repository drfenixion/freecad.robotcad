from typing import Optional, Callable, Any, Tuple
import numpy as np
import numpy.linalg as la
from numpy.typing import NDArray, ArrayLike

import FreeCAD as fc


# Console output levels accepted by calculate()'s `output_level` argument.
# LOG_LEVEL_INFO: the whole tree-like trace and the warnings are printed.
# LOG_LEVEL_WARN: only warnings are printed.
LOG_LEVEL_INFO = 'info'
LOG_LEVEL_WARN = 'warn'
# All log levels in the order shown in the property panel. The first one is
# the default.
LOG_LEVELS = [LOG_LEVEL_WARN, LOG_LEVEL_INFO]


# Override properties for inertia calculator. For more information see
# calculate() documentation.
PROPERTY_GROUP = 'RobotCad'

# Override mode enumeration values, in the order shown in the property
# panel. 'none' is considered default mode.
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
    """ Print FreeCAD inertia matrix """
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
    # skew-symmetric matrix
    S = np.array([[   0.0, -T.A34,  T.A24],
                  [ T.A34,    0.0, -T.A14],
                  [-T.A24,  T.A14,    0.0]])
    # screw transform matrix
    return np.block([[      R, np.zeros((3,3)) ],
                     [  S @ R,               R ]])


def transform_to_frame(JJ: NDArray, placement: fc.Placement) -> NDArray:
    """ Express a 6x6 inertia matrix in the frame where `placement` is defined.

    Arguments
    ---------
    JJ: np.ndarray
        6x6 mass matrix of a rigid body in a local frame.
    placement: fc.Placement
        Placement of that local frame in the target frame (local to target
        transform), e.g. the global placement of an object.

    Return
    ------
    np.ndarray
        6x6 mass matrix of the same body in the target frame.
    """
    X = screw_tf_matrix(placement.Matrix.inverse())
    return X.T @ JJ @ X


def inertia_matrix6d(J: fc.Matrix, Pc: fc.Vector, m: float) -> ArrayLike:
    """ Construct rigid body mass matrix (generalized 6x6 inertia matrix).

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
        6x6 mass matrix of rigid body.
    """
    # inertia matrix
    J = np.array([[J.A11, J.A12, J.A13],
                  [J.A21, J.A22, J.A23],
                  [J.A31, J.A32, J.A33]])
    # skew-symmetric matrix
    S = np.array([[  0.0, -Pc.z,    Pc.y],
                  [  Pc.z,    0.0, -Pc.x],
                  [ -Pc.y, Pc.x,    0.0]])
    # rigid body inertia tensor
    mS = m * S
    return np.block([[J - mS @ S,            mS],
                     [      mS.T, m * np.eye(3)]])


def mass_com_inertia(JJ: np.ndarray) -> Tuple[float, fc.Vector, NDArray]:
    """ Extract mass, center of mass and inertia from a rigid body mass matrix.

    Inverse of `inertia_matrix6d()`: `JJ` is the generalized 6x6 matrix

        JJ = [[J - m*S@S,      m*S],
              [    m*S.T, m*eye(3)]]

    where `S = skew(com)` and `J` is the inertia matrix about the center of
    mass. All results are expressed in the same frame and units as `JJ`.

    Return
    ------
    (float, fc.Vector, np.ndarray)
        Mass, center of mass location and 3x3 inertia matrix relative to the
        center of mass (axes parallel to the axes of the frame `JJ` is
        expressed in). For a matrix without mass (m <= 0) the center of mass
        is the origin and the inertia matrix is zero.
    """
    m = (JJ[3, 3] + JJ[4, 4] + JJ[5, 5]) / 3.0
    if m <= 0.0:
        return 0.0, fc.Vector(0.0, 0.0, 0.0), np.zeros((3, 3))
    # mS block is m * skew(com); use its antisymmetric part to suppress
    # numerical asymmetry.
    mS = 0.5 * (JJ[0:3, 3:6] - JJ[3:6, 0:3])
    com = fc.Vector(mS[2, 1], mS[0, 2], mS[1, 0]) * (1.0 / m)
    S = np.array([[0.0, -com.z, com.y],
                  [com.z, 0.0, -com.x],
                  [-com.y, com.x, 0.0]])
    J = JJ[0:3, 0:3] + m * (S @ S)
    # symmetrize
    J = 0.5 * (J + J.T)
    return m, com, J

def scale_inertia(JJ: NDArray, scale: fc.Vector | NDArray, copy: bool = True) -> NDArray:
    ''' Apply scale transform to 6d rigid body mass matrix. Assume density is constant.

    Arguments:
    ---------
    JJ: array_like
        6x6 mass matrix to transform.
    scale: array_like
        3-elements scale vector.
    copy: bool, default True
        Return object copy instead of in-place modification.

    Return
    ------
    NDArray:
       Scaled around origin 6x6 mass matrix.
    '''
    # copy object
    if copy:
        JJ = np.copy(JJ)
    # convert scale and check for identity
    scale = np.array(scale)
    if all(scale == 1.0):
        return JJ
    # update mass
    mass_scale = abs(np.prod(scale))
    m = (JJ[3, 3] + JJ[4, 4] + JJ[5, 5]) / 3.0
    m *= mass_scale
    JJ[3, 3], JJ[4, 4], JJ[5, 5] = m, m, m
    # update inertia matrix: extract geometric moments, scale them, convert backward
    J = JJ[0:3,0:3]
    G = 0.5*np.trace(J)*np.eye(3) - J
    G = scale.reshape((3,1)) * G * scale.reshape((1,3))
    G *= mass_scale
    JJ[0:3,0:3] = np.trace(G)*np.eye(3) - G
    # update off-diagonal skew-symmetric blocks
    skew_scale = np.array([[0.0,      scale[2], scale[1]],
                           [scale[2], 0.0,      scale[0]],
                           [scale[1], scale[0], 0      ]])

    JJ[0:3,3:6] *= skew_scale*mass_scale
    JJ[3:6,0:3] *= skew_scale*mass_scale
    return JJ

# Leader of the console log lines: tree structure of the objects.
# Indentation added for each tree level and branch symbol in the end. ASCII alternative: '|-> '.
LOG_INDENT = '    '
LOG_BRANCH = '└─> '

def log_leader(level: int) -> str:
    """ Return the leader of a console log line at tree `level`.
    """
    if level <= 0:
        return ''
    return LOG_INDENT * (level - 1) + LOG_BRANCH


def log_message(output_level: str, msg_level: str, level: int, message: str) -> None:
    """ Print `message` through FreeCAD's Console, honoring `output_level`.

    Arguments
    ---------
    output_level: str
        Console verbosity: LOG_LEVEL_INFO prints both info messages and
        warnings; LOG_LEVEL_WARN prints only warnings.
    msg_level: str
        This message's own severity (LOG_LEVEL_INFO or LOG_LEVEL_WARN); it is
        only printed when `output_level` is verbose enough to include it.
    level: int
        Level in the tree of objects. It encodes the recursion depth and is
        what gives the console log its tree-shaped structure: level 0 is
        printed as is, deeper levels get a leader made of `LOG_INDENT` for
        each level above the first and a branch symbol `LOG_BRANCH`, e.g.::

            INFO: Calculate inertial properties of "Assembly"
            INFO: └─> object Assembly: container
            INFO:     └─> object Box1: solids
            INFO:     └─> object Box2: solids
    message: str
        Message text.
    """
    if msg_level == LOG_LEVEL_INFO and output_level != LOG_LEVEL_INFO:
        return
    line = f'{msg_level.upper()}: {log_leader(level)}{message}'
    if msg_level == LOG_LEVEL_WARN:
        fc.Console.PrintWarning(line + '\n')
    else:
        fc.Console.PrintMessage(line + '\n')


def get_property(obj: fc.DocumentObject, name: str, ptype: str, units: Optional[str] = None, default: Optional[Any] = None, check_func: Optional[Callable[[Any], bool]] = None, check_desc: str = '') -> Any:
    """ Get FreeCAD object property by name.

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
        calculate_volume: bool = False,
        ignore_placement: bool = False,
        parent_material = None,
        level: int = 0,
        output_level: str = LOG_LEVEL_INFO,
):
    ''' Calculate the 6x6 inertia matrix of an object.

    It traverses the object tree, finds solid objects with volume and calculates the total inertia matrix
    taking into account material properties. The mass matrix (generalized 6x6 inertia matrix of a rigid
    body) is expressed in the frame of the root object's container: the Placement of the root object is
    taken into account (see `ignore_placement`).

    Default behavior can be overridden by specifying object properties from group 'RobotCad'.

    Object properties
    -----------------
    RobotCadOverrideMode: App::PropertyEnumeration, optional
        Property specifies calculation mode for this object. If not present, 'none' mode is used.
        Possible values:
        * 'none': Use the solid's shape and material to calculate inertia. This mode can be overridden
          in subobjects.
        * 'density': Material is ignored. RobotCadDensity is used for this object. Override properties
          in child objects are ignored. The object subtree is processed in calculate volume mode:
          only 'ignore' override mode is applied.
        * 'mass': Material is ignored. Density is assumed uniform, with the total mass given by the
          RobotCadMass property. The object subtree is processed in calculate volume mode:
          only 'ignore' override mode is applied.
        * 'manual': Object mass, center of mass and inertia matrix are provided manually via
          RobotCadMass, RobotCadCenterOfMass and RobotCadIxx..RobotCadIzz properties. All override
          properties in child objects are ignored.
        * 'ignore': This object and its whole subtree are not processed at all. Inertia and mass are
          assumed zero.
    RobotCadMass: App::PropertyMass, optional
        Mass for 'mass' and 'manual' override mode.
    RobotCadDensity: App::PropertyDensity, optional
        Density for 'density' override mode.
    RobotCadCenterOfMass: App::PropertyPlacement, optional
        Center of mass for 'manual' mode (Base, in mm, in this object's local frame). The placement's
        Rotation gives the orientation, relative to this object's local frame, of the axes in which
        RobotCadIxx..RobotCadIzz below are expressed (e.g. the principal axes of inertia).
    RobotCadIxx, RobotCadIxy, RobotCadIxz, RobotCadIyy, RobotCadIyz, RobotCadIzz: App::PropertyFloat, optional
        Components of the 3x3 inertia matrix about the center of mass for 'manual' mode, in kg*mm^2
        (FreeCAD's native inertia unit, same as Shape.MatrixOfInertia), expressed in the axes given by
        RobotCadCenterOfMass.Rotation.

    Arguments
    ---------
    obj: DocumentObject
        Root object.
    calculate_volume: bool, default False
        Ignore material properties throughout the subtree. In this case density is assumed equal to 1,
        i.e. the result is the volume matrix (mm^3 instead of kg). In volume mode only 'ignore' override
        mode is applied.
    ignore_placement: bool, default False
        Ignore root object placement. Inertia matrix is calculated in the object's local frame.
    parent_material: Material, optional
        Material to use if ShapeMaterial is not provided or its name is 'Default'.
    level: int, default 0
        Level in the tree of objects. Gives the indentation of the console log (see `log_message()`).
    output_level: str, default LOG_LEVEL_INFO
        Console verbosity: LOG_LEVEL_INFO prints both the trace of the tree walk and warnings;
        LOG_LEVEL_WARN prints only warnings. Passed unchanged to recursive calls.

    Return
    ------
    np.ndarray
        6x6 mass matrix of the rigid body in the frame of the root object's container
        (ignore_placement = False) or in the local frame of the root object (ignore_placement = True).

    '''

    def log(msg_level: str, message: str) -> None:
        """ Print `message` at this call's tree `level`, see `log_message()`. """
        log_message(output_level, msg_level, level, message)

    #
    # process properties
    #
    override_mode = get_property_optional(
        obj, PROP_OVERRIDE_MODE, 'App::PropertyEnumeration',
        check_func=lambda v: v in OVERRIDE_MODES,
        check_desc=f'value must be one of {OVERRIDE_MODES}',
    )
    # get override-mode parameters and process ignore settings
    if override_mode is None:
        for name in OVERRIDE_VALUE_PROPERTY_NAMES:
            if name in obj.PropertiesList:
                log(LOG_LEVEL_WARN, f'object {obj.Label} ({obj.FullName}) property "{name}" is supplied but property "{PROP_OVERRIDE_MODE}" is missing.')
        override_mode = OVERRIDE_MODE_NONE
    elif override_mode == OVERRIDE_MODE_IGNORE:
        # This object (subtree) is excluded from the calculation entirely.
        log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): ignore, subtree not processed, inertia assumed zero.')
        return np.zeros((6,6))

    # determine calculation mode
    if calculate_volume:
        override_mode = OVERRIDE_MODE_NONE
    else:
        # process override mode settings
        if override_mode == OVERRIDE_MODE_MASS:
            calculate_volume = True
            override_mass = get_property(obj, PROP_MASS, 'App::PropertyMass', units='kg')
        elif override_mode == OVERRIDE_MODE_DENSITY:
            calculate_volume = True
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

    # determine material
    material = parent_material
    material_is_inherited = True
    # get object material
    if hasattr(obj, 'ShapeMaterial') and obj.ShapeMaterial.Name != 'Default':
        material = obj.ShapeMaterial
        material_is_inherited = False

    #
    # link-like objects
    #
    # required properties: LinkedObject via getLinkedObject() != self
    # optional properties: ScaleVector, LinkTransform
    #
    linked_obj = obj.getLinkedObject(recursive=False)
    if linked_obj is not obj:
        # logging
        log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): link, calculate_volume: {calculate_volume}, override_mode: {override_mode}.')
        # calculate inertia of linked object
        JJ = calculate(linked_obj, calculate_volume=calculate_volume, ignore_placement=not getattr(obj, 'LinkTransform', True), parent_material=material, level=level + 1, output_level=output_level)
        # apply link scale
        if hasattr(obj, 'ScaleVector'):
            JJ = scale_inertia(JJ, obj.ScaleVector)
        # apply link transform
        if not ignore_placement and hasattr(obj, 'Placement'):
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
    # there is four ways of processing such objects:
    # * via Base/Linked object and ElementList
    #    - works for expanded multi-element Links (elements are shown) and expanded LinkArrays
    #    - do not work for not-expanded multi-element Links (elements are not shown), not-expanded LinkArrays, general Arrays
    #    - individual scale on ElementList leve
    #    - fused/overlapped objects is not supported
    # * via Base/Linked object and PlacementList
    #    - works for not-expanded multi-element Links, not-expanded LinkArrays and general Arrays
    #    - do not work for expanded multi-element Links, expanded LinkArrays
    #    - individual scale via ScaleList for LinkArrays
    #    - fused/overlapped objects is not supported
    # * via subobject list (getSubObjects()) and further walk
    #    - works for general containers, expanded Links, expanded LinkArrays
    #    - do not work for not-expanded Links, not-expanded LinkArrays (individual placement is missed) and Arrays
    #    - individual on subobjects level
    #    - fused/overlapped objects is not supported
    # * via Shape.Solids
    #    - works for all solids objects with Shape
    #    - do not work correctly for not-expanded LinkArrays and multi-elements Links with individual scale
    #    - individual scales is not supported
    #    - fused/overlapped object support only for Arrays with Fuse set.
    #
    ### container type detection ###
    container_type = None
    base = None
    element_list = None
    placement_list = None
    subname_list = None
    # multilink -> process via ElementList
    if hasattr(obj, 'LinkedObject') and getattr(obj, 'ElementCount', 0) > 0 and hasattr(obj, 'ShowElement'):
        base = obj.LinkedObject
        if obj.ShowElement and hasattr(obj, 'ElementList'):
            container_type = 'expanded multi-element Link'
            element_list = obj.ElementList
        elif hasattr(obj, 'PlacementList'):
            container_type = 'not-expanded multi-element Link'
            placement_list = obj.PlacementList
    # arrays
    elif hasattr(obj, 'Base'):
        base = obj.Base
        # link arrays -> process via ElementList or PlacementList
        if hasattr(obj, 'ExpandArray'):
            if obj.ExpandArray and hasattr(obj, 'ElementList'):
                container_type = 'expanded link array'
                element_list = obj.ElementList
            elif not obj.ExpandArray and hasattr(obj, 'PlacementList'):
                container_type = 'not-expanded link array'
                placement_list = obj.PlacementList
        # non-fused arrays -> process via PlacementList
        elif hasattr(obj, 'PlacementList') and not getattr(obj, 'Fuse', False):
            container_type = 'general non-fused array'
            placement_list = obj.PlacementList
    # general containers -> process via subname_list
    elif len(obj.getSubObjects()) > 0:
        container_type = 'container'
        subname_list = obj.getSubObjects()
    ### container objects processing ###
    JJ = np.zeros((6,6))
    if (element_list is not None) or (placement_list is not None):
        # check base if base object exists
        if base is None:
            log(LOG_LEVEL_WARN, f'object {obj.Label} ({obj.FullName}): {container_type} does not have Base/LinkedObject, zero mass assumed.')
            return np.zeros((6,6))
        # general logging
        log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): {container_type}, calculate_volume: {calculate_volume}, override_mode: {override_mode}.')
        # calculate base object inertia
        JJb = calculate(base, calculate_volume=calculate_volume, ignore_placement=not getattr(obj, 'LinkTransform', False), parent_material=material, level=level + 1, output_level=output_level)
        # process arrays
        if element_list is not None:
            # via ElementList
            for element in element_list:
                X = screw_tf_matrix(element.Placement.Matrix.inverse())
                JJ += X.T @ scale_inertia(JJb, element.ScaleVector) @ X
        elif hasattr(obj, 'ScaleList'):
            # via PlacementList with ScaleList
            for placement, scale in zip(placement_list, obj.ScaleList):
                X = screw_tf_matrix(placement.Matrix.inverse())
                JJ += X.T @ scale_inertia(JJb, scale) @ X
        else:
            # via PlacementList only
            for placement in placement_list:
                X = screw_tf_matrix(placement.Matrix.inverse())
                JJ += X.T @ JJb @ X
    elif subname_list is not None:
        log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): {container_type}, calculate_volume: {calculate_volume}, override_mode: {override_mode}.')
        # process arrays via subname list
        for subname in subname_list:
            subobj = obj.getSubObject(subname, 1)
            JJ += calculate(subobj, calculate_volume=calculate_volume, ignore_placement=False, parent_material=material, level=level + 1, output_level=output_level)
    ### finish container processing
    if container_type is not None:
        # apply container-level scale
        if hasattr(obj, 'ScaleVector'):
            JJ = scale_inertia(JJ, obj.ScaleVector)
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
        # extract solids mass and inertia properties
        if shape.Solids:
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
        # normalize to volume (density 1)
        mass = (JJ[3,3] + JJ[4,4] + JJ[5,5]) / 3.0
        if mass > 0.0:
            JJ *= volume / mass
        # mode selection:
        if override_mode == OVERRIDE_MODE_NONE:
            if calculate_volume:
                # volume calculation, do nothing
                log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): solids/compound, volume: {volume} mm^3.')
            elif material is not None:
                # apply material
                inherit_text = '(inherited)' if material_is_inherited else ''
                # get material density
                try:
                    density = fc.Units.Quantity(material.PhysicalProperties['Density']).getValueAs('kg/mm^3').Value
                except (KeyError, AttributeError, ValueError) as e:
                    log(LOG_LEVEL_WARN, f'object {obj.Label} ({obj.FullName}) material "{material.Name}" {inherit_text} does not have "Density" property ({e}), assuming zero mass.')
                    JJ *= 0.0
                else:
                    # apply material density
                    JJ *= density
                    # logging
                    log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): solids/compound, material: {material.Name} {inherit_text}, density: {1000*density} g/mm^3, mass: {1000*JJ[3,3]} g.')
            else:
                # no material is provided
                material_name = obj.ShapeMaterial.Name if hasattr(obj, 'ShapeMaterial') else 'None'
                log(LOG_LEVEL_WARN, f'object {obj.Label} ({obj.FullName}) material is "{material_name}", inherited material is "None", assuming zero mass.')
                JJ *= 0.0
        elif override_mode == OVERRIDE_MODE_DENSITY:
            # override density
            JJ *= override_density
            # logging
            log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): solids/compound, override_density: {1000*override_density} g/mm^3, mass: {1000*JJ[3,3]} g.')
        elif override_mode == OVERRIDE_MODE_MASS:
            # override mass
            if volume > 0.0:
                JJ *= override_mass / volume
                # logging
                log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): solids/compound, override_mass: {1000*override_mass} g.')
            else:
                log(LOG_LEVEL_WARN, f'object {obj.Label} ({obj.FullName}) does not have volume.')

        # return result
        return JJ

    #
    # Other objects
    #
    log(LOG_LEVEL_INFO, f'object {obj.Label} ({obj.FullName}): unknown type without volume/mass.')
    return np.zeros((6,6))

