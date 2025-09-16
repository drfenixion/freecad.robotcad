
import FreeCADGui as fcgui

from .ui import command_assembly_from_urdf  # noqa: F401
from .ui import command_box_from_bounding_box  # noqa: F401
from .ui import command_calculate_mass_and_inertia  # noqa: F401
from .ui import command_duplicate_robot # noqa: F401
from .ui import command_get_planning_scene  # noqa: F401
from .ui import command_kk_edit  # noqa: F401
from .ui import command_new_attached_collision_object  # noqa: F401
from .ui import command_new_joint # noqa: F401
from .ui import command_new_joints_filled # noqa: F401
from .ui import command_new_joints_filled_spider_connect # noqa: F401
from .ui import command_new_link # noqa: F401
from .ui import command_new_links_filled # noqa: F401
from .ui import command_new_observer # noqa: F401
from .ui import command_new_pose # noqa: F401
from .ui import command_new_robot # noqa: F401
from .ui import command_explode_links # noqa: F401
from .ui import command_new_trajectory # noqa: F401
from .ui import command_new_controller # noqa: F401
from .ui import command_new_sensor # noqa: F401
from .ui import command_open_models_library # noqa: F401
from .ui import command_new_workcell # noqa: F401
from .ui import command_new_xacro_object # noqa: F401
from .ui import command_new_lcs_at_robot_link_body # noqa: F401
from .ui import command_reload # Developer tool. # noqa: F401
from .ui import command_robot_from_urdf # noqa: F401
from .ui import command_set_joints # noqa: F401
from .ui import command_set_placement # noqa: F401
from .ui import command_set_placement_fast # noqa: F401
from .ui import command_set_placement_fast_child_to_parent # noqa: F401
from .ui import command_set_placement_fast_parent_to_child # noqa: F401
from .ui import command_set_placement_fast_sensor # noqa: F401
from .ui import command_set_placement_in_absolute_coordinates # noqa: F401
from .ui import command_set_placement_by_orienteer # noqa: F401
from .ui import command_set_placement_by_orienteer_with_hold_chain # noqa: F401
from .ui import command_rotate_joint_x # noqa: F401
from .ui import command_rotate_joint_y # noqa: F401
from .ui import command_rotate_joint_z # noqa: F401
from .ui import command_simplify_mesh # noqa: F401
from .ui import command_sphere_from_bounding_box # noqa: F401
from .ui import command_cylinder_x_aligned_from_bounding_box # noqa: F401
from .ui import command_cylinder_y_aligned_from_bounding_box # noqa: F401
from .ui import command_cylinder_z_aligned_from_bounding_box # noqa: F401
from .ui import command_create_collision_copy_obj # noqa: F401
from .ui import command_update_planning_scene # noqa: F401
from .ui import command_urdf_export # noqa: F401
from .ui import command_set_material # noqa: F401
from .ui import command_calculate_mass_and_inertia # noqa: F401
from .ui import command_world_generator # noqa: F401
from .ui import command_transfer_project_to_external_code_generator # noqa: F401
from .ui import command_wb_settings # noqa: F401

from .wb_utils import ICON_PATH
from . import wb_constants

######
###########
# link commands
#######
#############
class link_command_group:
    def __init__(self):
        pass
    def GetCommands(self):
        return (
            'NewLink',  # Defined in ./ui/command_new_link.py.
            'NewLinksFilled',  # Defined in ./ui/command_new_links_filled.py.))
        )

    def GetResources(self):
        return {'MenuText': "create link", 'ToolTip': "Link commands"}
fcgui.addCommand("NewLinkCommandGroup",link_command_group())  

#####
########
# joint commands 
####
########## 
class joint_command_group:
    def __init__(self):
        pass
    def GetCommands(self):
        return (
        'NewJoint',  # Defined in ./ui/command_new_joint.py.
        'NewJointsFilled',  # Defined in ./ui/command_new_joints_filled.py.
        'NewJointsFilledSpider',  # Defined in ./ui/command_new_joints_filled_spider_connect.py.
        )

    def GetResources(self):
        return {'MenuText':"create joint", 'ToolTip': "Joint Commands"}
fcgui.addCommand("NewJointCommandGroup",joint_command_group())
#############
####
# placement commands
###############
class placement_command_group:
    def __init__(self):
        pass
    def GetCommands(self):
        return (
    'SetCROSSPlacementFast',  # Defined in ./ui/command_set_placement_fast.py.
    'SetCROSSPlacementFastChildToParent',  # Defined in ./ui/command_set_placement_fast_child_to_parent.py.
    'SetCROSSPlacementFastParentToChild',  # Defined in ./ui/command_set_placement_fast_parent_to_child.py.
    'SetCROSSPlacementInAbsoluteCoordinates',  # Defined in ./ui/command_set_placement_in_absolute_coordinates.py.
    'SetCROSSPlacementByOrienteer',  # Defined in ./ui/command_set_placement_by_orienteer.py.
    'SetCROSSPlacementByOrienteerWithHoldChain',  # Defined in ./ui/command_set_placement_by_orienteer_with_hold_chain.py.
    'SetCROSSPlacementFastSensor',  # Defined in ./ui/command_set_placement_fast_sensor.py.
        )

    def GetResources(self):
        
        return {'MenuText': "set Placement", 'ToolTip':"placement commands"}

fcgui.addCommand("NewPlacementCommandGroup",placement_command_group())

# ######
# #############################
# export and import 
#####
class export_command_group:
    def __init__(self):
        pass
    def GetCommands(self):
        return (
            'UrdfImport',  # Defined in ./ui/command_robot_from_urdf.py.
            'AssemblyFromUrdf',  # Defined in ./ui/command_assembly_from_urdf.py.
            'UrdfExport',  # Defined in ./ui/command_urdf_export.py.
            'TransferProjectToExternalCodeGenerator',  # Defined in ./ui/command_transfer_project_to_external_code_generator.py.
        )

    def GetResources(self):
        return {'MenuText':"export n import", 'ToolTip': "export n import commands"}
fcgui.addCommand("ExportCommandGroup",export_command_group())

############
#############
#########################
# workbench
############
######
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
        creation_n_editing=[
            # Creation and editing.
            'NewRobot',  # Defined in ./ui/command_new_robot.py.
            'ExplodeLinks',  # Defined in ./ui/command_explode_links.py.
            "NewLinkCommandGroup",
            'NewLCSAtRobotLinkBody',  # Defined in ./ui/command_new_lcs_at_robot_link_body.py.
            "NewJointCommandGroup",
            'NewController',  # Defined in ./ui/command_new_controller.py.
            'NewSensor',  # Defined in ./ui/command_new_sensor.py.
            'OpenModelsLibrary',  # Defined in ./ui/command_open_models_library.py.
            'NewWorkcell',  # Defined in ./ui/command_new_workcell.py.
            'NewXacroObject',  # Defined in ./ui/command_new_xacro_object.py.
            'KKEdit',  # Defined in ./ui/command_kk_edit.py.
            
        ]
        self.appendToolbar("creation n editing",creation_n_editing)
        creating_n_editing_ext=creation_n_editing=[
            'DuplicateRobot',  # Defined in ./ui/command_duplicate_robot.py.
        ]
        placement=[
            "NewPlacementCommandGroup",
            'RotateJointX',  # Defined in ./ui/command_rotate_joint_x.py.
            'RotateJointY',  # Defined in ./ui/command_rotate_joint_y.py.
            'RotateJointZ',  # Defined in ./ui/command_rotate_joint_z.py.
        ]
        self.appendToolbar("placement",placement)
        collision=[
            'BoxFromBoundingBox',  # Defined in ./ui/command_box_from_bounding_box.py.
            'SphereFromBoundingBox',  # Defined in ./ui/command_sphere_from_bounding_box.py.
            'ZAlignedCylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_z_aligned_from_bounding_box.py.
            'XAlignedCylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_x_aligned_from_bounding_box.py.
            'YAlignedCylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_y_aligned_from_bounding_box.py.
            'CreateCollisionCopyObj',  # Defined in ./ui/command_create_collision_copy_obj.py.
        ]
        self.appendToolbar("collision",collision)
        live_debugging=[
             # "Live" debugging.
            'GetPlanningScene',  # Defined in ./ui/command_get_planning_scene.py.
            'UpdatePlanningScene',  # Defined in ./ui/command_update_planning_scene.py.
            'NewAttachedCollisionObject',  # Defined in ./ui/command_new_attached_collision_object.py.
            'NewPose',  # Defined in ./ui/command_new_pose.py.
            'NewTrajectory',  # Defined in ./ui/command_new_trajectory.py.
            'SetJoints',  # Defined in ./ui/command_set_joints.py.
        ]
        mesh_simplification=[
             # Mesh simplification.
            'SimplifyMesh',  # Defined in ./ui/command_simplify_mesh.py.
            'Separator',
        ]
        physical_properties=[
            'SetMaterial',  # Defined in ./ui/command_set_material.py.
            'CalculateMassAndInertia',  # Defined in ./ui/command_calculate_mass_and_inertia.py.
        ]
        self.appendToolbar("Physical properties",physical_properties)
        import_n_export=[
            # Import / export.
            "ExportCommandGroup"
        ]
        self.appendToolbar("world",[
            'WorldGenerator',  # Defined in ./ui/command_world_generator.py.
        ])
        self.appendToolbar("import n export",import_n_export)
        settings=[
             # Workbench settings.
            'WbSettings',  # Defined in ./ui/command_wb_settings.py.
        ]
        self.appendToolbar("settings",settings)
        Development=[
            'Reload',  # Comment out to disable this developer tool.
        ]
        self.appendToolbar("Development",Development)
       
        # Same as commands but with NewObserver and without Reload.
        menu_commands = [
            # Mesh simplification.
            'SimplifyMesh',  # Defined in ./ui/command_simplify_mesh.py.
            'WorldGenerator',  # Defined in ./ui/command_world_generator.py.
            # Workbench settings.
            'WbSettings',  # Defined in ./ui/command_wb_settings.py.
            'Separator',
        ]
        self.appendMenu('RobotCAD', menu_commands)
        self.appendMenu(['RobotCAD',"Create_n_edit"],[
              # Creation and editing.
            'NewRobot',  # Defined in ./ui/command_new_robot.py.
            'ExplodeLinks',  # Defined in ./ui/command_explode_links.py.
            'NewLink',  # Defined in ./ui/command_new_link.py.
            'NewLinksFilled',  # Defined in ./ui/command_new_links_filled.py.
            'NewJoint',  # Defined in ./ui/command_new_joint.py.
            'NewJointsFilled',  # Defined in ./ui/command_new_joints_filled.py.
            'NewJointsFilledSpider',  # Defined in ./ui/command_new_joints_filled_spider_connect.py.
            'NewController',  # Defined in ./ui/command_new_controller.py.
            'NewSensor',  # Defined in ./ui/command_new_sensor.py.
            'OpenModelsLibrary',  # Defined in ./ui/command_open_models_library.py.
            'NewWorkcell',  # Defined in ./ui/command_new_workcell.py.
            'NewXacroObject',  # Defined in ./ui/command_new_xacro_object.py.
            'KKEdit',  # Defined in ./ui/command_kk_edit.py.
            'DuplicateRobot',  # Defined in ./ui/command_duplicate_robot.py.
            ])
        self.appendMenu(['RobotCAD',"Placement"],[
            # Placement
            'NewLCSAtRobotLinkBody',  # Defined in ./ui/command_new_lcs_at_robot_link_body.py.
            'SetCROSSPlacementFast',  # Defined in ./ui/command_set_placement_fast.py.
            'SetCROSSPlacementFastChildToParent',  # Defined in ./ui/command_set_placement_fast_child_to_parent.py.            
            'SetCROSSPlacementFastParentToChild',  # Defined in ./ui/command_set_placement_fast_parent_to_child.py.
            'SetCROSSPlacementInAbsoluteCoordinates',  # Defined in ./ui/command_set_placement_in_absolute_coordinates.py.
            'SetCROSSPlacementByOrienteer',  # Defined in ./ui/command_set_placement_by_orienteer.py.
            'SetCROSSPlacementByOrienteerWithHoldChain',  # Defined in ./ui/command_set_placement_by_orienteer_with_hold_chain.py.
            'SetCROSSPlacementFastSensor',  # Defined in ./ui/command_set_placement_fast_sensor.py.
            'RotateJointX',  # Defined in ./ui/command_rotate_joint_x.py.
            'RotateJointY',  # Defined in ./ui/command_rotate_joint_y.py.
            'RotateJointZ',  # Defined in ./ui/command_rotate_joint_z.py.
        ])
        self.appendMenu(["RobotCAD","Collisions"],[
             # Collisions
            'BoxFromBoundingBox',  # Defined in ./ui/command_box_from_bounding_box.py.
            'SphereFromBoundingBox',  # Defined in ./ui/command_sphere_from_bounding_box.py.
            'ZAlignedCylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_z_aligned_from_bounding_box.py.
            'XAlignedCylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_x_aligned_from_bounding_box.py.
            'YAlignedCylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_y_aligned_from_bounding_box.py.
            'CreateCollisionCopyObj',  # Defined in ./ui/command_create_collision_copy_obj.py.
        ])
        self.appendMenu(["RobotCAD","Material Properties"],[
             # Definition of inertial properties.
            'SetMaterial',  # Defined in ./ui/command_set_material.py.
            'CalculateMassAndInertia',  # Defined in ./ui/command_calculate_mass_and_inertia.py.
        ])
        self.appendMenu(["RobotCAD","debugging"],[
            # "Live" debugging.
            'GetPlanningScene',  # Defined in ./ui/command_get_planning_scene.py.
            'UpdatePlanningScene',  # Defined in ./ui/command_update_planning_scene.py.
            'NewAttachedCollisionObject',  # Defined in ./ui/command_new_attached_collision_object.py.
            'NewPose',  # Defined in ./ui/command_new_pose.py.
            'NewTrajectory',  # Defined in ./ui/command_new_trajectory.py.
            'NewObserver',  # Defined in ./ui/command_new_observer.py.
            'SetJoints',  # Defined in ./ui/command_set_joints.py.
            'Separator',
        ])
        self.appendMenu(["RobotCAD","Import_n_export"],[
             # Import / export.
            'UrdfImport',  # Defined in ./ui/command_robot_from_urdf.py.
            'AssemblyFromUrdf',  # Defined in ./ui/command_assembly_from_urdf.py.
            'UrdfExport',  # Defined in ./ui/command_urdf_export.py.
            'TransferProjectToExternalCodeGenerator',  # Defined in ./ui/command_transfer_project_to_external_code_generator.py.
        ])
        fcgui.addIconPath(str(ICON_PATH))
        # fcgui.addLanguagePath(joinDir('Resources/translations'))

    def Activated(self):
        """Code run when a user switches to this workbench."""
        pass

    def Deactivated(self):
        """Code run when this workbench is deactivated."""
        pass


fcgui.addWorkbench(CrossWorkbench())
