"""Global workbench constants. Excluded from wb_globals.py with reason eliminate circular dependency"""

import FreeCAD as fc

# Constants.
PREFS_CATEGORY = 'RobotCAD'  # Category in the preferences dialog.
PREF_VHACD_PATH = 'vhacd_path'  # Path to the V-HACD executable.
PREF_OVERCROSS_TOKEN = 'overcross_token'  # Auth token for external code generator
WORKBENCH_NAME = 'RobotCAD - ROS2'

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

ROS2_CONTROLLERS_INTERFACES = [
    'position',
    'velocity',
    'acceleration',
    'effort',
]

# some types required to be replaced to more suited because can be more convenient to use
ROS2_CONTROLLERS_PARAMS_TYPES_REPLACEMENTS = {
    'joints' : {
        'replace': 'App::PropertyLinkList', 
        'origin': 'App::PropertyStringList', 
        'check_func': 'is_joint',
        'default_value_replace': [],
        },
    'joint' : {
        'replace': 'App::PropertyLink', 
        'origin': 'App::PropertyString', 
        'check_func': 'is_joint', 
        'default_value_replace': None,
        },
    'front_wheels_names' : {
        'replace': 'App::PropertyLinkList', 
        'origin': 'App::PropertyStringList',
        'check_func': 'is_joint',
        'default_value_replace': [],
        },
    'rear_wheels_names' :{
        'replace': 'App::PropertyLinkList', 
        'origin': 'App::PropertyStringList',
        'check_func': 'is_joint',
        'default_value_replace': [],
        },
    'front_wheels_state_names' : {
        'replace': 'App::PropertyLinkList', 
        'origin': 'App::PropertyStringList',
        'check_func': 'is_joint',
        'default_value_replace': [],
        },
    'rear_wheels_state_names' : {
        'replace': 'App::PropertyLinkList', 
        'origin': 'App::PropertyStringList',
        'check_func': 'is_joint',
        'default_value_replace': [],
        },
    'dof_names' : {
        'replace': 'App::PropertyLinkList', 
        'origin': 'App::PropertyStringList',
        'check_func': 'is_joint',
        'default_value_replace': [],
        },
    'reference_and_state_dof_names' : {
        'replace': 'App::PropertyLinkList', 
        'origin': 'App::PropertyStringList',
        'check_func': 'is_joint',
        'default_value_replace': [],
        },
    'command_joints' : {
        'replace': 'App::PropertyLinkList', 
        'origin': 'App::PropertyStringList',
        'check_func': 'is_joint',
        'default_value_replace': [],
        },
    'traction_joint_name' : {
        'replace': 'App::PropertyLink', 
        'origin': 'App::PropertyString', 
        'check_func': 'is_joint', 
        'default_value_replace': None,
        },        
    'steering_joint_name' : {
        'replace': 'App::PropertyLink', 
        'origin': 'App::PropertyString', 
        'check_func': 'is_joint', 
        'default_value_replace': None,
        },      
    'left_wheel_names' : {
        'replace': 'App::PropertyLinkList', 
        'origin': 'App::PropertyStringList', 
        'check_func': 'is_joint', 
        'default_value_replace': [],
        }, 
    'right_wheel_names' : {
        'replace': 'App::PropertyLinkList', 
        'origin': 'App::PropertyStringList', 
        'check_func': 'is_joint', 
        'default_value_replace': [],
        }, 
        

    'base_frame_id' : {
        'replace': 'App::PropertyLink', 
        'origin': 'App::PropertyString',
        'check_func': 'is_link',
        'default_value_replace': None,
        },
    'odom_frame_id' : {
        'replace': 'App::PropertyLink', 
        'origin': 'App::PropertyString',
        'check_func': 'is_link',
        'default_value_replace': None,
        },
    'control__frame__id' : {
        'replace': 'App::PropertyLink', 
        'origin': 'App::PropertyString',
        'check_func': 'is_link',
        'default_value_replace': None,
        },
    'fixed_world_frame__frame__id' : {
        'replace': 'App::PropertyLink', 
        'origin': 'App::PropertyString',
        'check_func': 'is_link',
        'default_value_replace': None,
        },
    'ft_sensor__frame__id' : {
        'replace': 'App::PropertyLink', 
        'origin': 'App::PropertyString',
        'check_func': 'is_link',
        'default_value_replace': None,
        },
    'gravity_compensation__frame__id' : {
        'replace': 'App::PropertyLink', 
        'origin': 'App::PropertyString',
        'check_func': 'is_link',
        'default_value_replace': None,
        },
    'gravity_compensation__CoG__pos' : {
        'replace': 'App::PropertyPosition', 
        'origin': 'App::PropertyFloatList',
        'check_func': 'return_true',
        'default_value_replace': fc.Vector(),
        },
    'frame_id' : {
        'replace': 'App::PropertyLink', 
        'origin': 'App::PropertyString',
        'check_func': 'is_link',
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