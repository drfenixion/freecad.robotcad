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
from ..wb_utils import is_link, ros_name
from ..wb_utils import is_robot_selected


class _CalculateMassAndInertiaCommand:
    def GetResources(self) -> dict:
        return {
            'Pixmap': 'calculate_mass_and_inertia.svg',
            'MenuText': tr('Calculate: mass, inertia, center of mass, placement of joints relative to total CoM'),
            'ToolTip': tr(
                'Select robot and press this button.'
                ' It will calculate mass, inertia, center of mass, placement of joints relative to total CoM'
                ' based on density (from material) and fills links data. If link does not'
                ' have material, default material will be taken'
                ' from robot element. Link will skipped if'
                ' property of link - "MaterialNotCalculate" is'
                ' true (in that case you sould manually fill link inertia and mass).'
                ' You can visually check inertia placement'
                ' in Gazebo. Turn on display of inertia in Gazebo'
                ' and check what generated inertia blocks'
                ' approximately same size and same'
                ' position/orientation as their links. Inertia'
                ' block orientation tilt to towards the mass'
                ' displacement is ok for unsymmetrical bodies.',
            ),
        }

    def Activated(self) -> None:
        doc = fc.activeDocument()
        objs = fcgui.Selection.getSelection()

        # Implementation note: the command is active only when a robot is selected.
        robot = objs[0]

        default_material = material_from_material_editor(robot.MaterialCardPath)

        doc.openTransaction(tr('Calculate mass and inertia'))
        # TODO refactor this code block to be more readable (split to functions)
        for link in robot.Proxy.get_links():
            print('Start process inertia and mass of link - Label: ', link.Label, ' Label2: ', link.Label2)

            #both MaterialNotCalculate and CalculateInertiaBasedOnMass cannot be true
            if link.CalculateInertiaBasedOnMass and link.MaterialNotCalculate:
                error(f'''MaterialNotCalculate and CalculateMaterialBasedOnMass are both true for "{link.Label}"\n
                      both cannot be true \n change state of one and try again \n''')
                continue

            if  link.MaterialNotCalculate:
                continue

            if not link.Real:
                error(f'Link "{link.Label}" skipped. No bound Real element for Link.', gui=True)
                continue

            compound = get_compound(
                link.Real,
                link.Placement,
                compound_name = "inert_link_" + ros_name(link),
                compound_el_name = "inert_el_" + ros_name(link),
            )
            elem_with_volume = first_object_with_volume(compound)

            if not elem_with_volume:
                error(f'Link "{link.Label}" does not link to any child with volume.', gui=True)
                continue

            center_of_gravity = center_of_gravity_mm(elem_with_volume)
            elem_matrix_of_inertia = matrix_of_inertia(elem_with_volume)
            elem_volume_mm3 = volume_mm3(elem_with_volume)
            doc.removeObject(elem_with_volume.Name)

            elem_material = material_from_material_editor(link.MaterialCardPath)

            #  show and error if material is not specified , inertia calculation based on mass and
            #  material not calculate is false
            if (elem_material.material_name is None) and (default_material.material_name is None) :
                if not link.CalculateInertiaBasedOnMass and not link.MaterialNotCalculate:
                    error(
                    f'Link "{link.Label}" skipped.'
                    ' No material specified for Link and no default material specified for robot element.', gui=True,
                    )
                    continue

            if center_of_gravity is None:
                error(
                    f'Link "{link.Label}" skipped.'
                    ' Can not get CenterOfGravity of bound Real element.',
                    gui=True,
                )
                continue

            if elem_matrix_of_inertia is None:
                error(f'Cannot get the matrix of inertia of the object bound by "{link.Label}".Real[0].', gui=True)
                continue

            if not link.CalculateInertiaBasedOnMass:
                if elem_material.material_name is None:
                    material = default_material
                    warn(f'No material specified for Link "{link.Label}". Using material specified in the containing robot.', gui=False)
                else:
                 material = elem_material

                if ((material.density is None)
                     or (material.density.Value <= 0.0)):
                    error(f'Link "{link.Label}" skipped. Material density not strictly positive.', gui=True)
                    continue

            volume = fc.Units.Quantity(elem_volume_mm3, 'mm^3')
            if not link.CalculateInertiaBasedOnMass :
                # clear any expression that might have been used to enter the value for mass manually
                # without clearing an existing expression before updating mass the value for mass does not update
                link.clearExpression("Mass")
                link.Mass = quantity_as(volume * material.density, 'kg')
            else:
                if link.Mass <= 0.0:
                    error(f'Link "{link.Label}" skipped. Mass ({link.Mass}) is not strictly positive.', gui=True)
                    continue

            # TODO: have matrix_of_inertia return a specified unit without correction
            elem_matrix_of_inertia = correct_matrix_of_inertia(elem_matrix_of_inertia, elem_volume_mm3, link.Mass)

            link.CenterOfMass = fc.Placement(link.MountedPlacement * center_of_gravity, link.MountedPlacement.Rotation, fc.Vector())
            # clear expression for all inertia properties incase there are any
            for exp in ['Ixx','Ixy','Ixz','Iyy','Iyz','Izz']:
                link.clearExpression(exp)

            link.Ixx = elem_matrix_of_inertia.A11
            link.Ixy = elem_matrix_of_inertia.A12
            link.Ixz = elem_matrix_of_inertia.A13
            link.Iyy = elem_matrix_of_inertia.A22
            link.Iyz = elem_matrix_of_inertia.A23
            link.Izz = elem_matrix_of_inertia.A33

            linkParentLCSPlacement = link.Placement * link.MountedPlacement.inverse() # global coords of link parent LCS
            link.CenterOfMassGlobalCoords = linkParentLCSPlacement * link.CenterOfMass # global coords of link CenterOfMass

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
        return is_robot_selected()


fcgui.addCommand('CalculateMassAndInertia', _CalculateMassAndInertiaCommand())
