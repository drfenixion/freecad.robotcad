"""Global workbench constants. Excluded from wb_globals.py with reason eliminate circular dependency"""

import FreeCAD as fc


def str_to_bool(s: str) -> bool:
    return s.lower() == "true"


# Constants.
PREFS_CATEGORY = 'RobotCAD'  # Category in the preferences dialog.
PREF_VHACD_PATH = 'vhacd_path'  # Path to the V-HACD executable.
PREF_OVERCROSS_TOKEN = 'overcross_token'  # Auth token for external code generator
WORKBENCH_NAME = 'RobotCAD - ROS2'

lcs_wrapper_prefix = "LCS wrapper "

XMLTODICT_ATTR_PREFIX_ORIGIN = '@'
XMLTODICT_ATTR_PREFIX_FIXED_FOR_PROP_NAME = 'attr_'

### ROS2_CONTROLLERS
ROS2_CONTROLLERS_CURRENT_ROS_VERSION_PARAM_NAME = 'controllers_current_ros_version'
ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE = '___'
ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE_YAML = '.'
ROS2_CONTROLLERS_PARAM_MAP_MARKER = '__map_'
ROS2_CONTROLLERS_ROS_VERSIONS = [
    {'ros_version': 'jazzy', 'controllers_branch': 'master'},
    {'ros_version': 'iron', 'controllers_branch': 'iron'},
]

ROS2_CONTROLLERS_PARAMS_TO_FRECAD_PROP_MAP = {
    'bool': 'App::PropertyBool',
    'bool_array': 'App::PropertyBoolList',
    'int': 'App::PropertyInteger',
    # 'int_': 'App::PropertyIntegerConstraint',
    'int_array': 'App::PropertyIntegerList',
    # 'enumeration': 'App::PropertyEnumeration',
    'double': 'App::PropertyFloat',
    # 'double_': 'App::PropertyFloatConstraint',
    'double_array': 'App::PropertyFloatList',
    'string': 'App::PropertyString',
    'string_array': 'App::PropertyStringList',
}

TYPE_CONVERT_FUNCTIONS = {
    'App::PropertyBool': lambda string: str_to_bool(string),
    'App::PropertyInteger': lambda string: int(string),
    'App::PropertyFloat': lambda string: float(string),
}

ROS2_CONTROLLERS_INTERFACES = [
    'position',
    'velocity',
    'acceleration',
    'effort',
]

# some types required to be replaced to more suited because can be more convenient to use
ROS2_CONTROLLERS_PARAMS_TYPES_REPLACEMENTS = {
    'joints' : {
        'replace': 'App::PropertyLinkListGlobal',
        'origin': 'App::PropertyStringList',
         # 'is_controller', 'is_broadcaster' used for chain mode.
         # Check is success if any of check functions return True
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': [],
    },
    'joint' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': None,
    },
    'front_wheels_names' : {
        'replace': 'App::PropertyLinkListGlobal',
        'origin': 'App::PropertyStringList',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': [],
    },
    'rear_wheels_names' :{
        'replace': 'App::PropertyLinkListGlobal',
        'origin': 'App::PropertyStringList',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': [],
    },
    'front_wheels_state_names' : {
        'replace': 'App::PropertyLinkListGlobal',
        'origin': 'App::PropertyStringList',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': [],
    },
    'rear_wheels_state_names' : {
        'replace': 'App::PropertyLinkListGlobal',
        'origin': 'App::PropertyStringList',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': [],
    },
    'dof_names' : {
        'replace': 'App::PropertyLinkListGlobal',
        'origin': 'App::PropertyStringList',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': [],
    },
    'reference_and_state_dof_names' : {
        'replace': 'App::PropertyLinkListGlobal',
        'origin': 'App::PropertyStringList',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': [],
    },
    'command_joints' : {
        'replace': 'App::PropertyLinkListGlobal',
        'origin': 'App::PropertyStringList',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': [],
    },
    'traction_joint_name' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': None,
    },
    'steering_joint_name' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': None,
    },
    'left_wheel_names' : {
        'replace': 'App::PropertyLinkListGlobal',
        'origin': 'App::PropertyStringList',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': [],
    },
    'right_wheel_names' : {
        'replace': 'App::PropertyLinkListGlobal',
        'origin': 'App::PropertyStringList',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': [],
    },
    'front_left_wheel_command_joint_name' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': None,
    },
    'front_right_wheel_command_joint_name' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': None,
    },
    'rear_right_wheel_command_joint_name' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': None,
    },
    'rear_left_wheel_command_joint_name' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': None,
    },
    'front_left_wheel_state_joint_name' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': None,
    },
    'front_right_wheel_state_joint_name' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': None,
    },
    'rear_right_wheel_state_joint_name' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': None,
    },
    'rear_left_wheel_state_joint_name' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_joint', 'is_controller', 'is_broadcaster'],
        'default_value_replace': None,
    },

    'base_frame_id' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_link'],
        'default_value_replace': None,
    },
    'odom_frame_id' : {
        'replace': 'App::PropertyString',
        'origin': 'App::PropertyString',
        'check_functions': ['return_true'],
        'default_value_replace': 'odom',
    },
    'control__frame__id' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_link'],
        'default_value_replace': None,
    },
    'fixed_world_frame__frame__id' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_link'],
        'default_value_replace': None,
    },
    'ft_sensor__frame__id' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_link'],
        'default_value_replace': None,
    },
    'gravity_compensation__frame__id' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_link'],
        'default_value_replace': None,
    },
    'gravity_compensation__CoG__pos' : {
        'replace': 'App::PropertyPosition',
        'origin': 'App::PropertyFloatList',
        'check_functions': ['return_true'],
        'default_value_replace': fc.Vector(),
    },
    'frame_id' : {
        'replace': 'App::PropertyLinkGlobal',
        'origin': 'App::PropertyString',
        'check_functions': ['is_link'],
        'default_value_replace': None,
    },
}

ROS2_CONTROLLERS_VALIDATION_RULES_DESCRIPTIONS = {
    ## **Value validators**
    'bounds': 'Bounds checking (inclusive)',
    'lt': 'parameter < value',
    'gt': 'parameter > value',
    'lt_eq': 'parameter <= value',
    'gt_eq': 'parameter >= value',
    'one_of': 'Value is one of the specified values',
    ## **String validators**
    'fixed_size': 'Length string is specified length',
    'size_gt': 'Length string is greater than specified length',
    'size_lt': 'Length string is less than specified length',
    'not_empty': 'String parameter is not empty',
    'one_of': 'String is one of the specified values',
    ## **Array validators**
    'unique': 'Contains no duplicates',
    'subset_of': 'Every element is one of the list',
    'fixed_size': 'Number of elements is specified length',
    'size_gt': 'Number of elements is greater than specified length',
    'size_lt': 'Number of elements is less than specified length',
    'not_empty': 'Has at-least one element',
    'element_bounds': 'Bounds checking each element (inclusive)',
    'lower_element_bounds': 'Lower bound for each element (inclusive)',
    'upper_element_bounds': 'Upper bound for each element (inclusive)',
}

ROS2_CONTROLLERS_EXCLUDED_PARAMS = {
    'joint_group_velocity_controller': {
        'params_based_on_controller_dir': 'forward_command_controller',
        'params_based_on_controller_name': 'forward_command_controller',
        'excluded_params': ['interface_name'],
    },
    'joint_group_effort_controller': {
        'params_based_on_controller_dir': 'forward_command_controller',
        'params_based_on_controller_name': 'forward_command_controller',
        'excluded_params': ['interface_name'],
    },
    'joint_group_position_controller': {
        'params_based_on_controller_dir': 'forward_command_controller',
        'params_based_on_controller_name': 'forward_command_controller',
        'excluded_params': ['interface_name'],
    },
}

ASSEMBLY_WB_JOINTS_MATCHING = {
    'Fixed': {
        'type': 'fixed', 
        'limits': None
    },
    'Revolute': {
        'type': 'revolute', 
        'limits': [
            {'assembly_enable_param': 'EnableAngleMin', 'assembly_value_param': 'AngleMin', 'robotcad_value_param': 'LowerLimit'},
            {'assembly_enable_param': 'EnableAngleMax', 'assembly_value_param': 'AngleMax', 'robotcad_value_param': 'UpperLimit'}
        ]
    },
    'Revolute_unlimited': {
        'type': 'continuous',
        'limits': None
    },
    'Cylindrical': {
        'type': 'undefined', 
        'limits': None
    },
    'Slider': {
        'type': 'prismatic', 
        'limits': [
            {'assembly_enable_param': 'EnableLengthMin', 'assembly_value_param': 'LengthMin', 'robotcad_value_param': 'LowerLimit'},
            {'assembly_enable_param': 'EnableLengthMax', 'assembly_value_param': 'LengthMax', 'robotcad_value_param': 'UpperLimit'}
        ]
    },
    'Ball': {
        'type': 'floating', 
        'limits': None
    },
    'Distance': {
        'type': 'fixed', 
        'limits': None
    },
    'Parallel': {
        'type': 'fixed', 
        'limits': None
    },
    'Perpendicular': {
        'type': 'fixed', 
        'limits': None
    },
    'Angle': {
        'type': 'fixed', 
        'limits': None
    },
    'RackPinion': {
        'type': 'undefined', 
        'limits': None
    },
    'Screw': {
        'type': 'undefined', 
        'limits': None
    },
    'Gears': {
        'type': 'undefined', 
        'limits': None
    },
    'Belt': {
        'type': 'undefined', 
        'limits': None
    },
}

JOINT_NAME_STR_BETWEEN_LINKS = '__to__'
