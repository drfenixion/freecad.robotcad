
import importlib

import FreeCAD as fc
import FreeCADGui as fcgui

# Command modules are imported resiliently: a module that fails to import
# (typically because a pip dependency such as `xmltodict` is not installed) no
# longer aborts the whole workbench initialization. The missing command is
# simply skipped, a soft warning is printed, and the "Check and install
# dependencies" dialog is offered on workbench activation.
_COMMAND_MODULES = [
    'command_assembly_from_urdf',
    'command_box_from_bounding_box',
    'command_bring_robot_to_pose',
    'command_calculate_mass_and_inertia',
    'command_duplicate_robot',
    'command_get_planning_scene',
    'command_kk_edit',
    'command_new_attached_collision_object',
    'command_new_joint',
    'command_new_joints_filled',
    'command_new_joints_filled_spider_connect',
    'command_new_link',
    'command_new_links_filled',
    'command_new_observer',
    'command_new_pose',
    'command_new_robot',
    'command_explode_links',
    'command_new_trajectory',
    'command_new_controller',
    'command_new_sensor',
    'command_open_models_library',
    'command_new_workcell',
    'command_new_xacro_object',
    'command_manage_link_display',
    'command_new_lcs_at_robot_link_body',
    'command_reload',
    'command_robot_from_urdf',
    'command_set_joints',
    'command_set_placement',
    'command_set_placement_fast',
    'command_set_placement_fast_child_to_parent',
    'command_set_placement_fast_parent_to_child',
    'command_set_placement_fast_sensor',
    'command_set_placement_in_absolute_coordinates',
    'command_set_placement_by_orienteer',
    'command_set_placement_by_orienteer_with_hold_chain',
    'command_rotate_joint_x',
    'command_rotate_joint_y',
    'command_rotate_joint_z',
    'command_simplify_mesh',
    'command_sphere_from_bounding_box',
    'command_cylinder_x_aligned_from_bounding_box',
    'command_cylinder_y_aligned_from_bounding_box',
    'command_cylinder_z_aligned_from_bounding_box',
    'command_create_collision_copy_obj',
    'command_update_planning_scene',
    'command_urdf_export',
    'command_set_material',
    'command_world_generator',
    'command_transfer_project_to_external_code_generator',
    'command_wb_settings',
    'command_generate_robot_by_text',
    'command_mcp_agent',
    'command_about',
    # CROSS sensors.
    'command_new_lidar2d',
    'command_new_rgb_camera',
    'command_new_ultrasound',
    # CROSS vacuum gripper.
    'command_new_vacuum_gripper',
]

from .deferred_messages import add_message

for _module_name in _COMMAND_MODULES:
    try:
        importlib.import_module(f'.ui.{_module_name}', __package__)
    except Exception as _exc:  # noqa: BLE001 - keep the workbench usable.
        # Deferred: reported when the workbench is activated, not at start-up.
        add_message(f'Tool "{_module_name}" could not be loaded: {_exc}')


def _report_deferred_on_activation() -> None:
    """Report load-time issues now that the workbench is being activated."""
    from .deferred_messages import add_message, flush

    try:
        from .dependencies import missing_dependencies
        missing = [d.pip_name for d in missing_dependencies(include_optional=False)]
    except Exception:
        missing = []

    if missing:
        add_message('RobotCAD: missing dependencies: ' + ', '.join(missing))
        add_message(
            'RobotCAD: use "About RobotCAD" -> "Check and install dependencies" '
            '(or the same button in "Workbench settings") to install them.',
        )

    flush()


def reactivate_commands_and_workspace() -> None:
    """Re-enable tools disabled by missing dependencies and rebuild the GUI.

    Called after the dependencies were installed. Re-imports the command
    modules that failed to load or disabled themselves, then rebuilds the
    toolbar and menu so the newly-available commands appear immediately.
    """
    import sys

    from .packages import invalidate_import_caches
    invalidate_import_caches()

    for name in _COMMAND_MODULES:
        full_name = f'{__package__}.ui.{name}'
        module = sys.modules.get(full_name)
        try:
            if module is None:
                importlib.import_module(f'.ui.{name}', __package__)
            elif getattr(module, 'imports_ok', True) is False:
                importlib.reload(module)
        except Exception:
            pass

    from .deferred_messages import flush
    flush()

    # Rebuild the toolbar and menu with the now-available commands.
    workbench = _WORKBENCH_INSTANCE
    for remover in ('removeToolbar', 'removeMenu'):
        try:
            getattr(workbench, remover)('RobotCAD')
        except Exception:
            pass
    workbench.appendToolbar(
        'RobotCAD', _registered_commands(workbench._toolbar_commands),
    )
    workbench.appendMenu(
        'RobotCAD', _registered_commands(workbench._menu_commands),
    )
    try:
        fcgui.updateGui()
    except Exception:
        pass


from .wb_utils import ICON_PATH
from . import wb_constants


def _registered_commands(commands: list[str]) -> list[str]:
    """Drop command names that are not registered (e.g. failed modules).

    Separators are always kept.
    """
    try:
        registered = set(fcgui.listCommands())
    except Exception:
        return commands
    return [c for c in commands if c == 'Separator' or c in registered]


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
            'NewJointsFilledSpider',  # Defined in ./ui/command_new_joints_filled_spider_connect.py.
            'NewController',  # Defined in ./ui/command_new_controller.py.
            'NewSensor',  # Defined in ./ui/command_new_sensor.py.
            'NewVacuumGripper',  # Defined in ./ui/command_new_vacuum_gripper.py.
            'GenerateRobotByText',  # Defined in ./ui/command_generate_robot_by_text.py.
            'OpenModelsLibrary',  # Defined in ./ui/command_open_models_library.py.
            'NewWorkcell',  # Defined in ./ui/command_new_workcell.py.
            'NewXacroObject',  # Defined in ./ui/command_new_xacro_object.py.
            'ManageLinkDisplay',  # Defined in ./ui/command_manage_link_display.py.
            'NewLCSAtRobotLinkBody',  # Defined in ./ui/command_new_lcs_at_robot_link_body.py.
            'SetCROSSPlacementFast',  # Defined in ./ui/command_set_placement_fast.py.
            'SetCROSSPlacementFastChildToParent',  # Defined in ./ui/command_set_placement_fast_child_to_parent.py.
            'SetCROSSPlacementFastParentToChild',  # Defined in ./ui/command_set_placement_fast_parent_to_child.py.
            'SetCROSSPlacementInAbsoluteCoordinates',  # Defined in ./ui/command_set_placement_in_absolute_coordinates.py.
            'SetCROSSPlacementByOrienteer',  # Defined in ./ui/command_set_placement_by_orienteer.py.
            'SetCROSSPlacementByOrienteerWithHoldChain',  # Defined in ./ui/command_set_placement_by_orienteer_with_hold_chain.py.
            'SetCROSSPlacementFastSensor',  # Defined in ./ui/command_set_placement_fast_sensor.py.
            # 'SetCROSSPlacement',  # Defined in ./ui/command_set_placement.py.
            'RotateJointX',  # Defined in ./ui/command_rotate_joint_x.py.
            'RotateJointY',  # Defined in ./ui/command_rotate_joint_y.py.
            'RotateJointZ',  # Defined in ./ui/command_rotate_joint_z.py.
            'BoxFromBoundingBox',  # Defined in ./ui/command_box_from_bounding_box.py.
            'SphereFromBoundingBox',  # Defined in ./ui/command_sphere_from_bounding_box.py.
            'ZAlignedCylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_z_aligned_from_bounding_box.py.
            'XAlignedCylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_x_aligned_from_bounding_box.py.
            'YAlignedCylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_y_aligned_from_bounding_box.py.
            'CreateCollisionCopyObj',  # Defined in ./ui/command_create_collision_copy_obj.py.
            # 'SimplifyMesh',  # Defined in ./ui/command_simplify_mesh.py.
            'GetPlanningScene',  # Defined in ./ui/command_get_planning_scene.py.
            'UpdatePlanningScene',  # Defined in ./ui/command_update_planning_scene.py.
            # 'IKTool',  # Defined in ./ui/command_ik_tool.py.
            'NewAttachedCollisionObject',  # Defined in ./ui/command_new_attached_collision_object.py.
            'NewPose',  # Defined in ./ui/command_new_pose.py.
            'NewTrajectory',  # Defined in ./ui/command_new_trajectory.py.
            'KKEdit',  # Defined in ./ui/command_kk_edit.py.
            'SetJoints',  # Defined in ./ui/command_set_joints.py.
            'SetMaterial',  # Defined in ./ui/command_set_material.py.
            'CalculateMassAndInertia',  # Defined in ./ui/command_calculate_mass_and_inertia.py.
            'WorldGenerator',  # Defined in ./ui/command_world_generator.py.
            'UrdfImport',  # Defined in ./ui/command_robot_from_urdf.py.
            'AssemblyFromUrdf',  # Defined in ./ui/command_assembly_from_urdf.py.
            'UrdfExport',  # Defined in ./ui/command_urdf_export.py.
            'TransferProjectToExternalCodeGenerator',  # Defined in ./ui/command_transfer_project_to_external_code_generator.py.
            'WbSettings',  # Defined in ./ui/command_wb_settings.py.
            'MCPAgent',  # Defined in ./ui/command_mcp_agent.py.
            'AboutRobotCAD',  # Defined in ./ui/command_about.py.
            # 'Reload',  # Developer tool, hidden from toolbar.
        ]
        self.appendToolbar('RobotCAD', _registered_commands(toolbar_commands))

        # Same as commands but with NewObserver and without Reload.
        menu_commands = [
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
            'NewVacuumGripper',  # Defined in ./ui/command_new_vacuum_gripper.py.
            'GenerateRobotByText',  # Defined in ./ui/command_generate_robot_by_text.py.
            'OpenModelsLibrary',  # Defined in ./ui/command_open_models_library.py.
            'NewWorkcell',  # Defined in ./ui/command_new_workcell.py.
            'NewXacroObject',  # Defined in ./ui/command_new_xacro_object.py.
            'KKEdit',  # Defined in ./ui/command_kk_edit.py.
            'DuplicateRobot',  # Defined in ./ui/command_duplicate_robot.py.
            'Separator',
            # # CROSS sensors
            # 'NewRgbCamera',  # Defined in ./ui/command_new_rgb_camera.py.
            # 'NewLidar2d',  # Defined in ./ui/command_new_lidar2d.py.
            # 'NewUltrasound',  # Defined in ./ui/command_new_ultrasound.py.
            'Separator',
            # Placement
            'ManageLinkDisplay',  # Defined in ./ui/command_manage_link_display.py.
            'NewLCSAtRobotLinkBody',  # Defined in ./ui/command_new_lcs_at_robot_link_body.py.
            'SetCROSSPlacementFast',  # Defined in ./ui/command_set_placement_fast.py.
            'SetCROSSPlacementFastChildToParent',  # Defined in ./ui/command_set_placement_fast_child_to_parent.py.
            'SetCROSSPlacementFastParentToChild',  # Defined in ./ui/command_set_placement_fast_parent_to_child.py.
            'SetCROSSPlacementInAbsoluteCoordinates',  # Defined in ./ui/command_set_placement_in_absolute_coordinates.py.
            'SetCROSSPlacementByOrienteer',  # Defined in ./ui/command_set_placement_by_orienteer.py.
            'SetCROSSPlacementByOrienteerWithHoldChain',  # Defined in ./ui/command_set_placement_by_orienteer_with_hold_chain.py.
            'SetCROSSPlacementFastSensor',  # Defined in ./ui/command_set_placement_fast_sensor.py.
            'SetCROSSPlacement',  # Defined in ./ui/command_set_placement.py.
            'RotateJointX',  # Defined in ./ui/command_rotate_joint_x.py.
            'RotateJointY',  # Defined in ./ui/command_rotate_joint_y.py.
            'RotateJointZ',  # Defined in ./ui/command_rotate_joint_z.py.
            'Separator',
            # Collisions
            'BoxFromBoundingBox',  # Defined in ./ui/command_box_from_bounding_box.py.
            'SphereFromBoundingBox',  # Defined in ./ui/command_sphere_from_bounding_box.py.
            'ZAlignedCylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_z_aligned_from_bounding_box.py.
            'XAlignedCylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_x_aligned_from_bounding_box.py.
            'YAlignedCylinderFromBoundingBox',  # Defined in ./ui/command_cylinder_y_aligned_from_bounding_box.py.
            'CreateCollisionCopyObj',  # Defined in ./ui/command_create_collision_copy_obj.py.
            # Mesh simplification.
            'SimplifyMesh',  # Defined in ./ui/command_simplify_mesh.py.
            'Separator',
            # "Live" debugging.
            'GetPlanningScene',  # Defined in ./ui/command_get_planning_scene.py.
            'UpdatePlanningScene',  # Defined in ./ui/command_update_planning_scene.py.
            # 'IKTool',  # Defined in ./ui/command_ik_tool.py.
            'NewAttachedCollisionObject',  # Defined in ./ui/command_new_attached_collision_object.py.
            'NewPose',  # Defined in ./ui/command_new_pose.py.
            'BringRobotToPose',  # Defined in ./ui/command_bring_robot_to_pose.py.
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
            'WorldGenerator',  # Defined in ./ui/command_world_generator.py.
            'TransferProjectToExternalCodeGenerator',  # Defined in ./ui/command_transfer_project_to_external_code_generator.py.
            'Separator',
            # Workbench settings.
            'WbSettings',  # Defined in ./ui/command_wb_settings.py.
            'Separator',
            # MCP agent.
            'MCPAgent',  # Defined in ./ui/command_mcp_agent.py.
            'Separator',
            # About.
            'AboutRobotCAD',  # Defined in ./ui/command_about.py.
        ]

        self.appendMenu('RobotCAD', _registered_commands(menu_commands))

        # Kept so the toolbar/menu can be rebuilt after the dependencies are
        # installed (see reactivate_commands_and_workspace).
        self._toolbar_commands = toolbar_commands
        self._menu_commands = menu_commands

        fcgui.addIconPath(str(ICON_PATH))
        # fcgui.addLanguagePath(joinDir('Resources/translations'))

    def Activated(self):
        """Code run when a user switches to this workbench.

        Reports the messages collected during load (disabled tools / missing
        dependencies), then checks the dependencies and, when some required
        package is missing, opens the interactive install dialog (which can be
        closed by the user).
        """
        try:
            _report_deferred_on_activation()
        except Exception:
            pass

        try:
            from .ui.dependencies_dialog import show_dependencies_if_missing
            show_dependencies_if_missing()
        except Exception:
            # Never block switching to the workbench on a dependency check.
            pass

    def Deactivated(self):
        """Code run when this workbench is deactivated."""
        pass


_WORKBENCH_INSTANCE = CrossWorkbench()
fcgui.addWorkbench(_WORKBENCH_INSTANCE)
