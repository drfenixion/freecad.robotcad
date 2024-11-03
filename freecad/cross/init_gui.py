
import FreeCADGui as fcgui

from .ui import command_assembly_from_urdf
from .ui import command_box_from_bounding_box
from .ui import command_get_planning_scene
from .ui import command_kk_edit
from .ui import command_new_attached_collision_object  # noqa: F401
from .ui import command_new_joint
from .ui import command_new_joints_filled
from .ui import command_new_link
from .ui import command_new_links_filled
from .ui import command_new_observer
from .ui import command_new_pose
from .ui import command_new_robot
from .ui import command_explode_links
from .ui import command_new_trajectory
from .ui import command_new_controller
from .ui import command_new_workcell
from .ui import command_new_xacro_object
from .ui import command_new_lcs_at_robot_link_body
from .ui import command_reload # Developer tool.
from .ui import command_robot_from_urdf
from .ui import command_set_joints
from .ui import command_set_placement
from .ui import command_set_placement_fast
from .ui import command_set_placement_in_absolute_coordinates
from .ui import command_set_placement_by_orienteer
from .ui import command_rotate_joint_x
from .ui import command_rotate_joint_y
from .ui import command_rotate_joint_z
from .ui import command_simplify_mesh
from .ui import command_sphere_from_bounding_box
from .ui import command_cylinder_from_bounding_box
from .ui import command_update_planning_scene
from .ui import command_urdf_export
from .ui import command_set_material
from .ui import command_calculate_mass_and_inertia
from .ui import command_transfer_project_to_external_code_generator
from .ui import command_wb_settings
from .wb_utils import ICON_PATH
from . import wb_constants


class CrossWorkbench(fcgui.Workbench):
    """Class which gets initiated at startup of the GUI."""

    MenuText = wb_constants.WORKBENCH_NAME
    ToolTip = 'ROS-related workbench'
    Icon = str(ICON_PATH / 'robotcad_overcross_joint.svg')

    def GetClassName(self):
        return 'Gui::PythonWorkbench'

    def Initialize(self):
        """This function is called at the first activation of the workbench.

        This is the place to import all the commands.

        """
        # The order here defines the order of the icons in the GUI.
        toolbar_commands = [
            'NewRobot',  # Defined in ./ui/command_new_robot.py.
            'ExplodeLinks',  # Defined in ./ui/command_explode_links.py.
            'NewLink',  # Defined in ./ui/command_new_link.py.
            'NewLinksFilled',  # Defined in ./ui/command_new_links_filled.py.
            'NewJoint',  # Defined in ./ui/command_new_joint.py.
            'NewJointsFilled',  # Defined in ./ui/command_new_joints_filled.py.
            'NewController',  # Defined in ./ui/command_new_controller.py.
            'NewWorkcell',  # Defined in ./ui/command_new_workcell.py.
            'NewXacroObject',  # Defined in ./ui/command_new_xacro_object.py.
            'NewLCSAtRobotLinkBody',  # Defined in ./ui/command_new_lcs_at_robot_link_body.py.
            'SetCROSSPlacementFast',  # Defined in ./ui/command_set_placement_fast.py.
            'SetCROSSPlacementInAbsoluteCoordinates',  # Defined in ./ui/command_set_placement_in_absolute_coordinates.py.
            'SetCROSSPlacementByOrienteer',  # Defined in ./ui/command_set_placement_by_orienteer.py.
            # 'SetCROSSPlacement',  # Defined in ./ui/command_set_placement.py.
            'RotateJointX',  # Defined in ./ui/command_rotate_joint_x.py.
            'RotateJointY',  # Defined in ./ui/command_rotate_joint_y.py.
            'RotateJointZ',  # Defined in ./ui/command_rotate_joint_z.py.
            'BoxFromBoundingBox',  # Defined in ./ui/command_box_from_bounding_box.py.
            'SphereFromBoundingBox',  # Defined in ./ui/command_sphere_from_bounding_box.py.
            'CylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_from_bounding_box.py.
            'SimplifyMesh',  # Defined in ./ui/command_simplify_mesh.py.
            'GetPlanningScene',  # Defined in ./ui/command_get_planning_scene.py.
            'UpdatePlanningScene',  # Defined in ./ui/command_update_planning_scene.py.
            'NewAttachedCollisionObject',  # Defined in ./ui/command_new_attached_collision_object.py.
            'NewPose',  # Defined in ./ui/command_new_pose.py.
            'NewTrajectory',  # Defined in ./ui/command_new_trajectory.py.
            'KKEdit',  # Defined in ./ui/command_kk_edit.py.
            'SetJoints',  # Defined in ./ui/command_set_joints.py.
            'SetMaterial',  # Defined in ./ui/command_set_material.py.
            'CalculateMassAndInertia',  # Defined in ./ui/command_calculate_mass_and_inertia.py.
            'UrdfImport',  # Defined in ./ui/command_robot_from_urdf.py.
            'AssemblyFromUrdf',  # Defined in ./ui/command_assembly_from_urdf.py.
            'UrdfExport',  # Defined in ./ui/command_urdf_export.py.
            'TransferProjectToExternalCodeGenerator',  # Defined in ./ui/command_transfer_project_to_external_code_generator.py.            
            'WbSettings',  # Defined in ./ui/command_wb_settings.py.
            'Reload',  # Comment out to disable this developer tool.
        ]
        self.appendToolbar('RobotCAD', toolbar_commands)

        # Same as commands but with NewObserver and without Reload.
        menu_commands = [
            'NewRobot',  # Defined in ./ui/command_new_robot.py.
            'ExplodeLinks',  # Defined in ./ui/command_explode_links.py.
            'NewLink',  # Defined in ./ui/command_new_link.py.
            'NewLinksFilled',  # Defined in ./ui/command_new_links_filled.py.
            'NewJoint',  # Defined in ./ui/command_new_joint.py.
            'NewJointsFilled',  # Defined in ./ui/command_new_joints_filled.py.
            'NewController',  # Defined in ./ui/command_new_controller.py.
            'NewWorkcell',  # Defined in ./ui/command_new_workcell.py.
            'NewXacroObject',  # Defined in ./ui/command_new_xacro_object.py.
            'Separator',
            'NewLCSAtRobotLinkBody',  # Defined in ./ui/command_new_lcs_at_robot_link_body.py.
            'SetCROSSPlacementFast',  # Defined in ./ui/command_set_placement_fast.py.
            'SetCROSSPlacementInAbsoluteCoordinates',  # Defined in ./ui/command_set_placement_in_absolute_coordinates.py.
            'SetCROSSPlacementByOrienteer',  # Defined in ./ui/command_set_placement_by_orienteer.py.
            # 'SetCROSSPlacement',  # Defined in ./ui/command_set_placement.py.
            'RotateJointX',  # Defined in ./ui/command_rotate_joint_x.py.
            'RotateJointY',  # Defined in ./ui/command_rotate_joint_y.py.
            'RotateJointZ',  # Defined in ./ui/command_rotate_joint_z.py.
            'Separator',
            'BoxFromBoundingBox',  # Defined in ./ui/command_box_from_bounding_box.py.
            'SphereFromBoundingBox',  # Defined in ./ui/command_sphere_from_bounding_box.py.
            'CylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_from_bounding_box.py.
            'SimplifyMesh',  # Defined in ./ui/command_simplify_mesh.py.
            'Separator',
            'GetPlanningScene',  # Defined in ./ui/command_get_planning_scene.py.
            'UpdatePlanningScene',  # Defined in ./ui/command_update_planning_scene.py.
            'NewAttachedCollisionObject',  # Defined in ./ui/command_new_attached_collision_object.py.
            'NewPose',  # Defined in ./ui/command_new_pose.py.
            'NewTrajectory',  # Defined in ./ui/command_new_trajectory.py.
            'NewObserver',  # Defined in ./ui/command_new_observer.py.
            'KKEdit',  # Defined in ./ui/command_kk_edit.py.
            'SetJoints',  # Defined in ./ui/command_set_joints.py.
            'Separator',
            'SetMaterial',  # Defined in ./ui/command_set_material.py.
            'CalculateMassAndInertia',  # Defined in ./ui/command_calculate_mass_and_inertia.py.
            'Separator',
            'UrdfImport',  # Defined in ./ui/command_robot_from_urdf.py.
            'AssemblyFromUrdf',  # Defined in ./ui/command_assembly_from_urdf.py.
            'UrdfExport',  # Defined in ./ui/command_urdf_export.py.
            'TransferProjectToExternalCodeGenerator',  # Defined in ./ui/command_transfer_project_to_external_code_generator.py.      
            'Separator',      
            'WbSettings',  # Defined in ./ui/command_wb_settings.py.
        ]
            
        self.appendMenu('RobotCAD', menu_commands)

        fcgui.addIconPath(str(ICON_PATH))
        # fcgui.addLanguagePath(joinDir('Resources/translations'))

    def Activated(self):
        """Code run when a user switches to this workbench."""
        pass

    def Deactivated(self):
        """Code run when this workbench is deactivated."""
        pass


fcgui.addWorkbench(CrossWorkbench())
