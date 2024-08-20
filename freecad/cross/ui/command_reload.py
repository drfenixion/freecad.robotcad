# Developer tool to reload the workbench.

from importlib import import_module, reload

import FreeCADGui as fcgui


class _ReloadCommand:
    """The command definition to create a new Robot object."""

    def GetResources(self):
        return {
            'Pixmap': 'bulb',
            'MenuText': 'Reload the ROS workbench',
            'Accel': 'W, R',
            'ToolTip': 'Reload the ROS workbench',
        }

    def IsActive(self):
        return True

    def Activated(self):
        def _reload_module(module_name: str) -> None:
            try:
                module = import_module(module_name)
            except ImportError:
                return
            reload(module)
        _reload_module('freecad.cross.assembly4_utils')
        _reload_module('freecad.cross.assembly_from_urdf')
        _reload_module('freecad.cross.coin_utils')
        _reload_module('freecad.cross.deep_copy')
        _reload_module('freecad.cross.freecad_utils')
        _reload_module('freecad.cross.freecadgui_utils')
        _reload_module('freecad.cross.gui_utils')
        _reload_module('freecad.cross.import_dae')
        _reload_module('freecad.cross.joint')
        _reload_module('freecad.cross.joint_proxy')
        _reload_module('freecad.cross.link')
        _reload_module('freecad.cross.link_proxy')
        _reload_module('freecad.cross.mesh_utils')
        _reload_module('freecad.cross.placement_utils')
        _reload_module('freecad.cross.planning_scene_proxy')
        _reload_module('freecad.cross.planning_scene_utils')
        _reload_module('freecad.cross.pose_proxy')
        _reload_module('freecad.cross.robot')
        _reload_module('freecad.cross.robot_from_urdf')
        _reload_module('freecad.cross.robot_proxy')
        _reload_module('freecad.cross.ros.node')
        _reload_module('freecad.cross.ros.planning_scene')
        _reload_module('freecad.cross.ros.utils')
        _reload_module('freecad.cross.trajectory')
        _reload_module('freecad.cross.trajectory_proxy')
        _reload_module('freecad.cross.ui.command_assembly_from_urdf')
        _reload_module('freecad.cross.ui.command_box_from_bounding_box')
        _reload_module('freecad.cross.ui.command_get_planning_scene')
        _reload_module('freecad.cross.ui.command_kk_edit')
        _reload_module('freecad.cross.ui.command_new_joint')
        _reload_module('freecad.cross.ui.command_new_link')
        _reload_module('freecad.cross.ui.command_new_pose')
        _reload_module('freecad.cross.ui.command_new_robot')
        _reload_module('freecad.cross.ui.command_new_trajectory')
        _reload_module('freecad.cross.ui.command_new_workcell')
        _reload_module('freecad.cross.ui.command_new_xacro_object')
        _reload_module('freecad.cross.ui.command_reload')
        _reload_module('freecad.cross.ui.command_robot_from_urdf')
        _reload_module('freecad.cross.ui.command_set_cross_placement')
        _reload_module('freecad.cross.ui.command_set_joints')
        _reload_module('freecad.cross.ui.command_sphere_from_bounding_box')
        _reload_module('freecad.cross.ui.command_urdf_export')
        _reload_module('freecad.cross.ui.command_wb_settings')
        _reload_module('freecad.cross.ui.file_overwrite_confirmation_dialog')
        _reload_module('freecad.cross.ui.set_joints_dialog')
        _reload_module('freecad.cross.ui.set_joints_manual_input_table')
        _reload_module('freecad.cross.urdf_loader')
        _reload_module('freecad.cross.urdf_parser_utils')
        _reload_module('freecad.cross.urdf_utils')
        _reload_module('freecad.cross.utils')
        _reload_module('freecad.cross.version')
        _reload_module('freecad.cross.wb_utils')
        _reload_module('freecad.cross.workcell')
        _reload_module('freecad.cross.workcell_proxy')
        _reload_module('freecad.cross.xacro_loader')
        _reload_module('freecad.cross.xacro_object')
        _reload_module('freecad.cross.xacro_object_proxy')


fcgui.addCommand('Reload', _ReloadCommand())
