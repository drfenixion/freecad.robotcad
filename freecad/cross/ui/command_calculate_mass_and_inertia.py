import FreeCAD as fc
import FreeCADGui as fcgui

from ..freecad_utils import error
from ..freecad_utils import get_material, get_matrix_of_inertia, get_volume, get_center_of_gravity, get_first_object_with_volume, correct_matrix_of_inertia, get_linked_obj
from ..gui_utils import tr
from ..wb_utils import is_robot_selected, is_link


class CalculateMassAndInertiaCommand:
    def GetResources(self):
        return {'Pixmap': 'calculate_mass_and_inertia.svg',
                'MenuText': tr('Calculate mass and inertia'),
                'ToolTip': tr('Select robot and press this button. It will calculate mass and inertia based on density and fills links data. If link does not have material, default material will be taken from robot element. Link will skipped if property of link - "MaterialNotCalculate" is true. You can visually check inertia placement in Gazebo. Turn on display of inertia in Gazebo and check what generated inertia blocks approximately same size and same position/orientation as their links. Inertia block orientation tilt to towards the mass displacement is ok for unsymmetrical bodies.'),
                }
    
    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction(tr('Calculate mass and inertia'))
        objs = fcgui.Selection.getSelection()
        
        robot = objs[0]
        
        defaultMaterial = get_material(robot.MaterialCardPath)

        for elem in robot.Group:
            if is_link(elem):
                print('Start process inertia and mass of elem - Label: ', elem.Label, ' Label2: ', elem.Label2)
                if elem.MaterialNotCalculate == False:
                    
                    if  len(elem.Real) > 0:
                        if  len(elem.Collision) > 0:
                            real = elem.Real[0]

                            first_body = get_first_object_with_volume(real)
                            elemCenterOfGravity = get_center_of_gravity(first_body)
                            elemMatrixOfInertia = get_matrix_of_inertia(first_body)
                            elemVolumeMM3 = get_volume(first_body)
                            elemMaterial = get_material(elem.MaterialCardPath)

                            if elemMaterial['card_name'] != False or defaultMaterial['card_name'] != False:
                                
                                if elemVolumeMM3 != False:

                                    if elemCenterOfGravity != False:

                                        if elemMatrixOfInertia != False:
                                            
                                            if elemMaterial['card_name'] == False:
                                                material = defaultMaterial
                                                print('   No material specified for Link. Used default material of robot element')
                                            else:
                                                material = elemMaterial

                                            if material['density'] > 0:
                                                elemVolumeM3 = elemVolumeMM3 / 1000000000 # convert mm3 to m3
                                                elem.Mass = elemVolumeM3 * material['density']
                                                elemMatrixOfInertia = correct_matrix_of_inertia(elemMatrixOfInertia, elemVolumeMM3, elem.Mass)
                                                
                                                basic_obj = get_linked_obj(real)
                                                # correction if basic obj has not zero placement
                                                elemCenterOfGravity = elemCenterOfGravity - basic_obj.Placement.Base
                                                elem.CenterOfMass = fc.Placement(elem.MountedPlacement * elemCenterOfGravity, elem.MountedPlacement.Rotation, fc.Vector()) 
                                                
                                                elem.Ixx = elemMatrixOfInertia.A11
                                                elem.Ixy = elemMatrixOfInertia.A12
                                                elem.Ixz = elemMatrixOfInertia.A13
                                                elem.Iyy = elemMatrixOfInertia.A22
                                                elem.Iyz = elemMatrixOfInertia.A23
                                                elem.Izz = elemMatrixOfInertia.A33
                                            else:
                                                error('   Link skipped. Material density less or equal zero.') 
                                        else:
                                            error('   Link skipped. Can not get MatrixOfInertia of binded Real element.')                                        
                                    else:
                                        error('   Link skipped. Can not get CenterOfGravity of binded Real element.')
                                else:
                                    error('   Link skipped. Can not get volume of binded Real element.')
                            else:
                                error('   Link skipped. No material specified for Link and no default material specified for robot element.')
                        else:
                            error('   Link skipped. No binded Collision element for Link.')
                    else:
                        error('   Link skipped. No binded Real element for Link.')
                else:
                    print('   Link skipped. Option "MaterialNotCalculate" is True.')

                print('Finish process inertia and mass of elem - Label: ', elem.Label, ' Label2: ', elem.Label2)

        doc.recompute()
        doc.commitTransaction()

    def IsActive(self):
        return is_robot_selected()


fcgui.addCommand('CalculateMassAndInertia', CalculateMassAndInertiaCommand())
