from dataclasses import dataclass
from typing import Optional
import FreeCAD as fc
import FreeCADGui as fcgui
from ..freecad_utils import DO, center_of_gravity_mm, first_object_with_volume, get_compound
from ..freecad_utils import correct_matrix_of_inertia
from ..freecad_utils import error
from ..freecad_utils import material_from_material_editor
from ..freecad_utils import matrix_of_inertia
from ..freecad_utils import quantity_as
from ..freecad_utils import volume_mm3
from ..freecad_utils import warn
from ..gui_utils import tr
from ..wb_utils import is_link, is_link_selected, is_robot, ros_name
from ..wb_utils import is_robot_selected
from ..wb_constants import INERTIA_METHOD_ADVANCED, INERTIA_METHOD_BASIC
from .inertia_calculator import LOG_LEVEL_INFO, LOG_LEVEL_WARN
from .inertia_calculator import calculate, log_message, mass_com_inertia

# Inertia calculator results are in kg*mm^2, link inertia is in kg*m^2.
_KG_MM2_TO_KG_M2 = 1.0e-6

def center_of_mass_placement_in_link_frame(center_of_mass: fc.Vector, mount_placement: fc.Placement):
    """ Return placement of center of mass frame relative to link mount frame. 

        `mount_placement` is local frame placement in link mount frame. Function changes its origin location
        to `center_of_mass`.
    """
    return fc.Placement(mount_placement * center_of_mass, mount_placement.Rotation, fc.Vector())

@dataclass
class _LinkInertialProperties:
    """Result of the inertial properties calculation of one link.

    Values are in the units of the link properties: `center_of_mass` is a
    placement in the link frame (Base in mm), `mass_kg` is in kg and `ixx`..
    `izz` are in kg*m^2 and expressed in the axes given by the Rotation of
    `center_of_mass`.

    """

    # None: keep the link's current Mass (the Mass was entered by the user).
    mass_kg: Optional[float]
    center_of_mass: fc.Placement
    ixx: float
    ixy: float
    ixz: float
    iyy: float
    iyz: float
    izz: float


def _calculate_link_basic(
        doc: fc.Document,
        link: DO,
        default_material,
) -> Optional[_LinkInertialProperties]:
    """Calculate inertial properties of a link with the "basic" method.

    The first Real element with volume is used, with the material of the link
    (or the default material of the robot) or with the mass of the link if
    `CalculateInertiaBasedOnMass` is set.

    Return None (after reporting why) if the link is skipped.

    """
    print('Start process inertia and mass of link - Label: ', link.Label, ' Label2: ', link.Label2)

    # both MaterialNotCalculate and CalculateInertiaBasedOnMass cannot be true
    if link.CalculateInertiaBasedOnMass and link.MaterialNotCalculate:
        error(f'''MaterialNotCalculate and CalculateMaterialBasedOnMass are both true for "{link.Label}"\n
              both cannot be true \n change state of one and try again \n''')
        return None

    if link.MaterialNotCalculate:
        return None

    if not link.Real:
        error(f'Link "{link.Label}" skipped. No bound Real element for Link.', gui=True)
        return None

    compound = get_compound(
        link.Real,
        fc.Placement(),
        compound_name='inert_link_' + ros_name(link),
        compound_el_name='inert_el_' + ros_name(link),
        zeroing_real_objs_placement=True,
    )
    elem_with_volume = first_object_with_volume(compound)

    if not elem_with_volume:
        doc.removeObject(compound.Name)
        error(f'Link "{link.Label}" does not link to any child with volume.', gui=True)
        return None

    center_of_gravity = center_of_gravity_mm(elem_with_volume)
    elem_matrix_of_inertia = matrix_of_inertia(elem_with_volume)
    elem_volume_mm3 = volume_mm3(elem_with_volume)
    doc.removeObject(elem_with_volume.Name)

    elem_material = material_from_material_editor(link.MaterialCardPath)

    #  show and error if material is not specified , inertia calculation based on mass and
    #  material not calculate is false
    if (elem_material.material_name is None) and (default_material.material_name is None):
        if not link.CalculateInertiaBasedOnMass and not link.MaterialNotCalculate:
            error(
                f'Link "{link.Label}" skipped.'
                ' No material specified for Link and no default material specified for robot element.', gui=True,
            )
            return None

    if center_of_gravity is None:
        error(
            f'Link "{link.Label}" skipped.'
            ' Can not get CenterOfGravity of bound Real element.',
            gui=True,
        )
        return None

    if elem_matrix_of_inertia is None:
        error(f'Cannot get the matrix of inertia of the object bound by "{link.Label}".Real[0].', gui=True)
        return None

    if not link.CalculateInertiaBasedOnMass:
        if elem_material.material_name is None:
            material = default_material
            warn(f'No material specified for Link "{link.Label}". Using material specified in the containing robot.', gui=False)
        else:
            material = elem_material

        if ((material.density is None)
                or (material.density.Value <= 0.0)):
            error(f'Link "{link.Label}" skipped. Material density not strictly positive.', gui=True)
            return None

    volume = fc.Units.Quantity(elem_volume_mm3, 'mm^3')
    if not link.CalculateInertiaBasedOnMass:
        mass_kg = quantity_as(volume * material.density, 'kg')
        result_mass_kg = mass_kg
    else:
        if link.Mass <= 0.0:
            error(f'Link "{link.Label}" skipped. Mass ({link.Mass}) is not strictly positive.', gui=True)
            return None
        mass_kg = link.Mass.Value
        result_mass_kg = None  # keep the mass entered by the user

    center_of_mass_placement = center_of_mass_placement_in_link_frame(center_of_gravity, link.MountedPlacement)
    # TODO: have matrix_of_inertia return a specified unit without correction
    elem_matrix_of_inertia = correct_matrix_of_inertia(elem_matrix_of_inertia, elem_volume_mm3, mass_kg)
    
    print(f'Link "{link.Label}": mass: {mass_kg} kg, center of mass in link mount frame: {center_of_mass_placement.Base} mm.',)
    return _LinkInertialProperties(
        mass_kg=result_mass_kg,
        center_of_mass=center_of_mass_placement,
        ixx=elem_matrix_of_inertia.A11,
        ixy=elem_matrix_of_inertia.A12,
        ixz=elem_matrix_of_inertia.A13,
        iyy=elem_matrix_of_inertia.A22,
        iyz=elem_matrix_of_inertia.A23,
        izz=elem_matrix_of_inertia.A33,
    )


def _calculate_link_advanced(
        link: DO,
        output_level: str,
) -> Optional[_LinkInertialProperties]:
    """Calculate inertial properties of a link with the "advanced" method.

    Same convention as the "basic" method (and the URDF `<inertial>` tag): the
    inertia tensor is expressed in the axes of the Real element's local frame,
    the center of mass is expressed in the link mount frame
    (`MountedPlacement * center of mass`) and the Rotation of the center of
    mass placement is `MountedPlacement.Rotation`, i.e. the orientation of the
    inertia axes relative to the link frame.

    Return None (after reporting why) if the link is skipped.

    """
    log_message(
        output_level, LOG_LEVEL_INFO, 0,
        f'Calculate inertial properties of link "{link.Label}" ({ros_name(link)}), method: {INERTIA_METHOD_ADVANCED}.',
    )

    if not link.Real:
        error(f'Link "{link.Label}" skipped. No bound Real element for Link.', gui=True)
        return None

    if len(link.Real) > 1:
        # A link has one Real element: a link is attached to one joint.
        warn(
            f'Link "{link.Label}" has {len(link.Real)} Real elements, only the first one'
            f' ("{link.Real[0].Label}") is used. Group parts of a link in an Assembly or'
            ' a Part container.',
        )

    try:
        JJ = calculate(
            link.Real[0],
            ignore_placement=True,
            level=1,
            output_level=output_level,
        )
    except (ValueError, KeyError, TypeError) as e:
        error(f'Link "{link.Label}" skipped. {e}', gui=True)
        return None

    mass_kg, center_of_mass, inertia_kg_mm2 = mass_com_inertia(JJ)
    if mass_kg <= 0.0:
        error(
            f'Link "{link.Label}" skipped. Calculated mass is zero.'
            ' Assign materials to the parts of the link (or set the RobotCad override properties)'
            f' and check the console log (set robot "InertiaCalculatorLogLevel" to "{LOG_LEVEL_INFO}").',
            gui=True,
        )
        return None

    inertia = inertia_kg_mm2 * _KG_MM2_TO_KG_M2
    center_of_mass_placement = center_of_mass_placement_in_link_frame(center_of_mass, link.MountedPlacement)
    log_message(
        output_level, LOG_LEVEL_INFO, 1,
        f'link "{link.Label}": mass: {mass_kg} kg, center of mass in link mount frame: {center_of_mass_placement.Base} mm.',
    )
    return _LinkInertialProperties(
        mass_kg=mass_kg,
        center_of_mass=center_of_mass_placement,
        ixx=inertia[0, 0],
        ixy=inertia[0, 1],
        ixz=inertia[0, 2],
        iyy=inertia[1, 1],
        iyz=inertia[1, 2],
        izz=inertia[2, 2],
    )


def _apply_to_link(link: DO, properties: _LinkInertialProperties) -> None:
    """Write calculated inertial properties to the link."""
    if properties.mass_kg is not None:
        # clear any expression that might have been used to enter the value for mass manually
        # without clearing an existing expression before updating mass the value for mass does not update
        link.clearExpression('Mass')
        link.Mass = properties.mass_kg

    link.CenterOfMass = properties.center_of_mass
    # clear expression for all inertia properties incase there are any
    for exp in ['Ixx', 'Ixy', 'Ixz', 'Iyy', 'Iyz', 'Izz']:
        link.clearExpression(exp)

    link.Ixx = properties.ixx
    link.Ixy = properties.ixy
    link.Ixz = properties.ixz
    link.Iyy = properties.iyy
    link.Iyz = properties.iyz
    link.Izz = properties.izz

    link_frame_global_placement = link.Placement * link.MountedPlacement.inverse()  # global coords of the link frame
    link.CenterOfMassGlobalCoords = link_frame_global_placement * link.CenterOfMass  # global coords of link CenterOfMass


class _CalculateMassAndInertiaCommand:
    def GetResources(self) -> dict:
        return {
            'Pixmap': 'calculate_mass_and_inertia.svg',
            'MenuText': tr('Calculate: mass, inertia, center of mass, placement of joints relative to total CoM'),
            'ToolTip': tr(
                'Select robot and press this button.\n'
                'Tool will calculate mass, inertia, center of mass, placement of joints relative to total CoM\n'
                'and fills links data.\n'
                '\n'
                'The calculation method is selected by the robot property "InertiaCalculationMethod"'
                '(group "Inertial Parameters"): "basic" (default) or "advanced".\n'
                '\n'
                'Basic method\n'
                'Robot link density is assumed uniform with density specified via link material.\n'
                'If link does not have own material, default material will be taken\n'
                'from robot element. If "CalculateMaterialBasedOnMass" property is set then density\n'
                'is calulated from user specified "Mass" property and caluated link volume.\n'
                'If "MaterialNotCalculate" link property is true calcuation is skipped.\n' 
                'In this case you should manually fill inertial parameters.\n'
                '\n'
                'Advanced method\n'
                'Uses the materials assigned to the parts of the assembly and the optional RobotCad override\n'
                'properties set by "Override inertial properties" tool (a compound link is an Assembly or Part container).\n'
                'Materials and inertia calculation properties of links and robot are ignored (e.g. "CalculateMaterialBasedOnMass").\n'
                'Calculation trace is printed to the Report View, its verbosity can be adjusted by \n'
                '"InertiaCalculatorLogLevel robot property.\n'
                '\n'
                'You can visually check inertia placement'
                'in Gazebo. \nTurn on display of inertia in Gazebo'
                'and check what generated inertia blocks'
                'approximately \nsame size and same'
                'position/orientation as their links. \nInertia'
                'block orientation tilt to towards the mass'
                'displacement is ok for unsymmetrical bodies.',
            ),
        }

    def Activated(self) -> None:
        doc = fc.activeDocument()
        objs = fcgui.Selection.getSelection()

        # Implementation note: the command is active only when a robot is selected.
        links = []
        robot = None
        if is_robot(objs[0]):
            robot = objs[0]
            links = robot.Proxy.get_links()
        else:
            for obj in objs:
                if is_link(obj):
                    links.append(obj)
                    if not robot and len(obj.InList) and is_robot(obj.InList[0]):
                        robot = obj.InList[0]

        if robot is None:
            error('Cannot find the robot of the selected links.', gui=True)
            return

        method = getattr(robot, 'InertiaCalculationMethod', INERTIA_METHOD_BASIC)
        output_level = getattr(robot, 'InertiaCalculatorLogLevel', LOG_LEVEL_WARN)
        default_material = material_from_material_editor(robot.MaterialCardPath)

        doc.openTransaction(tr('Calculate mass and inertia'))
        for link in links:
            if method == INERTIA_METHOD_ADVANCED:
                properties = _calculate_link_advanced(link, output_level)
            else:
                properties = _calculate_link_basic(doc, link, default_material)
            if properties is None:
                continue
            _apply_to_link(link, properties)

        # calculate total mass of robot
        massTotal = 0
        allMassesFilled = True
        for link in robot.Proxy.get_links():
            if float(link.Mass) > 0:
                massTotal += float(link.Mass)
            else:
                allMassesFilled = False

        if allMassesFilled:
            robot.Mass = massTotal
        else:
            robot.Mass = 0

        # calculate robot.CenterOfMassGlobalCoords, joint.PlacementRelTotalCenterOfMass
        if robot.Mass > 0:
            CoMTotal = fc.Vector()
            for link in robot.Proxy.get_links():
                CoMTotal += 1 / robot.Mass.Value * link.Mass.Value * link.CenterOfMassGlobalCoords.Base

            robot.CenterOfMassGlobalCoords = fc.Placement(
                CoMTotal,
                fc.Rotation(),
                fc.Vector(),
            )

            for joint in robot.Proxy.get_joints():
                joint.PlacementRelTotalCenterOfMass = robot.CenterOfMassGlobalCoords.inverse() * joint.Placement

        print('Finished calculating mass and inertia.')
        doc.recompute()
        doc.commitTransaction()

    def IsActive(self):
        return is_robot_selected() or is_link_selected()


fcgui.addCommand('CalculateMassAndInertia', _CalculateMassAndInertiaCommand())
