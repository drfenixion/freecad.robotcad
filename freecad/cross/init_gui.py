
import FreeCADGui as fcgui

from .ui import command_assembly_from_urdf  # noqa: F401
from .ui import command_box_from_bounding_box  # noqa: F401
from .ui import command_calculate_mass_and_inertia  # noqa: F401
from .ui import command_cylinder_from_bounding_box  # noqa: F401
from .ui import command_duplicate_robot # noqa: F401
from .ui import command_get_planning_scene  # noqa: F401
from .ui import command_kk_edit  # noqa: F401
from .ui import command_new_attached_collision_object  # noqa: F401
from .ui import command_new_joint  # noqa: F401
from .ui import command_new_link  # noqa: F401
from .ui import command_new_observer  # noqa: F401
from .ui import command_new_pose  # noqa: F401
from .ui import command_new_robot  # noqa: F401
from .ui import command_new_trajectory  # noqa: F401
from .ui import command_new_workcell  # noqa: F401
from .ui import command_new_xacro_object  # noqa: F401
from .ui import command_reload  # Developer tool.  # noqa: F401
from .ui import command_robot_from_urdf  # noqa: F401
from .ui import command_set_joints  # noqa: F401
from .ui import command_set_material  # noqa: F401
from .ui import command_set_placement  # noqa: F401
from .ui import command_simplify_mesh  # noqa: F401
from .ui import command_sphere_from_bounding_box  # noqa: F401
from .ui import command_update_planning_scene  # noqa: F401
from .ui import command_urdf_export  # noqa: F401
from .ui import command_wb_settings  # noqa: F401
from .wb_utils import ICON_PATH


class CrossWorkbench(fcgui.Workbench):
    """Class which gets initiated at startup of the GUI."""

    MenuText = 'CROSS - ROS workbench'
    ToolTip = 'ROS-related workbench'
    Icon = str(ICON_PATH / 'cross.svg')

    def GetClassName(self):
        return 'Gui::PythonWorkbench'

    def Initialize(self):
        """This function is called at the first activation of the workbench.

        This is the place to import all the commands.

        """
        # The order here defines the order of the icons in the GUI.
        toolbar_commands = [
            'NewRobot',  # Defined in ./ui/command_new_robot.py.
            'NewLink',  # Defined in ./ui/command_new_link.py.
            'NewJoint',  # Defined in ./ui/command_new_joint.py.
            'NewWorkcell',  # Defined in ./ui/command_new_workcell.py.
            'NewXacroObject',  # Defined in ./ui/command_new_xacro_object.py.
            'SetCROSSPlacement',  # Defined in ./ui/command_set_placement.py.
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
            'WbSettings',  # Defined in ./ui/command_wb_settings.py.
            'Reload',  # Comment out to disable this developer tool.
        ]
        self.appendToolbar('CROSS', toolbar_commands)

        # Same as commands but with NewObserver and without Reload.
        menu_commands = [
                # Creation and editing.
                'NewRobot',  # Defined in ./ui/command_new_robot.py.
                'NewLink',  # Defined in ./ui/command_new_link.py.
                'NewJoint',  # Defined in ./ui/command_new_joint.py.
                'NewWorkcell',  # Defined in ./ui/command_new_workcell.py.
                'NewXacroObject',  # Defined in ./ui/command_new_xacro_object.py.
                'SetCROSSPlacement',  # Defined in ./ui/command_set_placement.py.
                'BoxFromBoundingBox',  # Defined in ./ui/command_box_from_bounding_box.py.
                'SphereFromBoundingBox',  # Defined in ./ui/command_sphere_from_bounding_box.py.
                'CylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_from_bounding_box.py.
                'KKEdit',  # Defined in ./ui/command_kk_edit.py.
                'DuplicateRobot',  # Defined in ./ui/command_duplicate_robot.py.
                'Separator',
                # Mesh simplification.
                'SimplifyMesh',  # Defined in ./ui/command_simplify_mesh.py.
                'Separator',
                # "Live" debugging.
                'GetPlanningScene',  # Defined in ./ui/command_get_planning_scene.py.
                'UpdatePlanningScene',  # Defined in ./ui/command_update_planning_scene.py.
                'NewAttachedCollisionObject',  # Defined in ./ui/command_new_attached_collision_object.py.
                'NewPose',  # Defined in ./ui/command_new_pose.py.
                'NewTrajectory',  # Defined in ./ui/command_new_trajectory.py.
                'NewObserver',  # Defined in ./ui/command_new_observer.py.
                'SetJoints',  # Defined in ./ui/command_set_joints.py.
                'Separator',
                # Definition of inertial properties.
                'SetMaterial',  # Defined in ./ui/command_set_material.py.
                'CalculateMassAndInertia',  # Defined in ./ui/command_calculate_mass_and_inertia.py.
                'Separator',
                # Import / export.
                'UrdfImport',  # Defined in ./ui/command_robot_from_urdf.py.
                'AssemblyFromUrdf',  # Defined in ./ui/command_assembly_from_urdf.py.
                'UrdfExport',  # Defined in ./ui/command_urdf_export.py.
                # Workbench settings.
                'Separator',
                'WbSettings',  # Defined in ./ui/command_wb_settings.py.
        ]

        self.appendMenu('CROSS', menu_commands)

        fcgui.addIconPath(str(ICON_PATH))
        # fcgui.addLanguagePath(joinDir('Resources/translations'))

    def Activated(self):
        """Code run when a user switches to this workbench."""
        pass

    def Deactivated(self):
        """Code run when this workbench is deactivated."""
        pass


fcgui.addWorkbench(CrossWorkbench())
