"""Proxy for Cross::Controller and Cross::Broadcaster FreeCAD objects

A controllers and broadcasters are representation of controllers and broadcasters from ros2_controllers.

"""

from __future__ import annotations

from typing import ForwardRef, List, Optional, Union, cast
from typing import Iterable
from copy import deepcopy
from pathlib import Path
import os
import yaml
import xml.etree.ElementTree as ET
import re

import FreeCAD as fc

from PySide.QtWidgets import QMenu  # FreeCAD's PySide

from .freecad_utils import ProxyBase
from .freecad_utils import add_property
from .freecad_utils import error
from .freecad_utils import get_valid_property_name
from .freecad_utils import warn
from .wb_utils import ICON_PATH
from .wb_utils import ROS2_CONTROLLERS_PATH
from .wb_utils import is_joint
from .wb_utils import is_link
from .wb_utils import is_controller
from .wb_utils import is_broadcaster
from .wb_utils import return_true
from .wb_utils import is_controllers_template_for_param_mapping
from .wb_utils import ros_name
from .wb_utils import get_valid_urdf_name
from .utils import deepmerge
from . import wb_constants

# Stubs and type hints.
from .controller import Controller as CrossController  # A Cross::Controller, i.e. a DocumentObject with Proxy "Controller". # noqa: E501
DO = fc.DocumentObject
DOList = List[DO]
VPDO = ForwardRef('FreeCADGui.ViewProviderDocumentObject')  # Don't want to import FreeCADGui here. # noqa: E501
AppLink = DO  # TypeId == 'App::Link'.
check_functions_required = [is_joint, is_link, return_true, is_controller, is_broadcaster] # dont remove used by check_functions()

class ControllerProxy(ProxyBase):
    """The proxy for Controller objects."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    ControllerType = 'Cross::Controller'
    BroadcasterType = 'Cross::Broadcaster'
    dynamicTypes = [ControllerType, BroadcasterType]

    Type = ControllerType

    # dinamic type at creating time.
    # type can be 'Cross::Controller' and 'Cross::Broadcaster'
    def __init__(self, obj: CrossController, dynamicType:str | None = None):
        super().__init__(
            'controller', [
            '_Type',
            ],
        )

        if dynamicType:
            if dynamicType in self.dynamicTypes:
                self.Type = dynamicType
            else:
                raise TypeError("Not supported dynamic type. Supporting only " + str(self.dynamicTypes) + " types.")

        if obj.Proxy is not self:
            obj.Proxy = self
        self.controller = obj

        self._init_properties(obj)


    def _init_properties(self, obj: CrossController):
        add_property(
            obj, 'App::PropertyString', '_Type', 'Internal',
            'The type',
        )
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type


    def execute(self, obj: CrossController) -> None:
        pass


    def onChanged(self, obj: CrossController, prop: str) -> None:
        pass


    def onDocumentRestored(self, obj):
        """Restore attributes because __init__ is not called on restore."""
        self.__init__(obj)


    def dumps(self):
        return None


    def loads(self, state) -> None:
        pass


class _ViewProviderController(ProxyBase):
    """A view provider for the Controller container object """

    def __init__(self, vobj: VPDO):
        super().__init__(
            'view_object',
            [
                'Visibility',
            ],
        )
        vobj.Proxy = self

    def getIcon(self):
        # Implementation note: "return 'controller.svg'" works only after
        # workbench activation in GUI.
        return str(ICON_PATH / 'controller.svg')

    def attach(self, vobj: VPDO):
        self.view_object = vobj
        self.controller = vobj.Object

    def updateData(self, obj: CrossController, prop: str):

        def get_param_to_replace(
            prop: str,
            param_map_marker: str = wb_constants.ROS2_CONTROLLERS_PARAM_MAP_MARKER,
        ):
            """Return param_to_replace.

            param_to_replace is string that should replaced with f.e. joint name
            from map source attr f.e. dof_names in mapped_params_templates
            example of mapping: 'gains_____map_dof_names___p' -> 'gains___joint_name___p'
            examples of param_to_replace: '__map_dof_names', '__map_joints'
            """

            return param_map_marker + prop


        def map_param(
            prop: str,
            mapped_params_templates: dict,
            obj: CrossController,
        ):
            """Map param by map param template. Add mapped param to obj

            mapped params templates - params that is templates for mapping other params (ex: gains_____map_joints___p)
            params for mapping - params that should be mapped by teplate (ex: joints)

            after mapping param name will be like - gains___joint_name___p
            """

            param_to_replace = get_param_to_replace(prop)

            mapped_params_templates_filtered = {}
            for template in mapped_params_templates:
                if param_to_replace in template:
                    mapped_params_templates_filtered[template] = template


            attr = getattr(obj, prop)

            controllers = get_controllers_data()
            controllers_merged = {}
            controllers_merged.update(controllers['controllers'])
            controllers_merged.update(controllers['broadcasters'])

            parameters_flatten_filtered = {}
            for param_flatten in controllers_merged[ros_name(obj)]['parameters_flatten']:
                if param_flatten in mapped_params_templates_filtered.keys():
                    parameters_flatten_filtered[param_flatten] = controllers_merged[ros_name(obj)]['parameters_flatten'][param_flatten]


            for el in attr:
                # exclude controllers and broadcaster from mapping because they can be presented in mapped param but used for chain purposes
                if not is_controller(el) and not is_broadcaster(el):

                    if isinstance(el, DO):
                        el = ros_name(el)

                    for template in mapped_params_templates_filtered:
                        mapped_prop_name = template.replace(param_to_replace, el)

                        try:
                            attr_not_exist_trigger = getattr(obj, mapped_prop_name)
                        except AttributeError:
                            parameters = unflatten_params(deepcopy(parameters_flatten_filtered), param_to_replace, el)
                            controllers_merged[ros_name(obj)]['parameters'] = parameters
                            obj = add_controller_properties_block(obj, controllers_merged[ros_name(obj)])
                            t = obj


        def remove_redundant_mapped_attrs(
            prop: str,
            obj: CrossController,
            parameter_full_name_glue: str = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE,
        ) -> CrossController:
            """ Remove redundant mapped attrebutes after thier source attr elements was deleted """

            params_to_map = getattr(obj, 'params_to_map', [])
            param_to_replace = get_param_to_replace(prop)

            if prop in params_to_map:

                # prepare param_to_map_value_as_str_list
                param_to_map_value = getattr(obj, prop)
                param_to_map_value_as_dict = {}
                for el in param_to_map_value:
                    if isinstance(el, DO):
                        el = ros_name(el)
                    param_to_map_value_as_dict[el] = el
                param_to_map_value_as_str_list = list(param_to_map_value_as_dict.keys())

                # check diff old mapped param and new
                mapped_params_prop_name = 'mapped_params' + parameter_full_name_glue + prop
                mapped_params_prop_value_old = getattr(obj, mapped_params_prop_name, [])
                if param_to_map_value_as_str_list != mapped_params_prop_value_old:
                    mapped_params_to_remove = list(set(mapped_params_prop_value_old) - set(param_to_map_value_as_str_list))

                    # remove reduntdant params
                    for mapped_param_to_remove in mapped_params_to_remove:
                        mapped_params_templates = getattr(obj, 'mapped_params_templates')
                        for mapped_params_template in mapped_params_templates:
                            param_to_remove_name = mapped_params_template.replace(param_to_replace, mapped_param_to_remove)
                            obj.removeProperty(param_to_remove_name)

                # save mapped params to check diff in future
                try:
                    attr = getattr(obj, mapped_params_prop_name)
                    if attr != param_to_map_value_as_str_list:
                        setattr(obj, mapped_params_prop_name, param_to_map_value_as_str_list)
                except AttributeError:
                    # add meta property
                    obj, used_property_name = add_property(
                        obj,
                        'App::PropertyStringList',
                        mapped_params_prop_name,
                        'Internal',
                        'Mapped params',
                        param_to_map_value_as_str_list,
                    )
                    obj.setPropertyStatus(mapped_params_prop_name, ['Hidden', 'ReadOnly'])

            return obj


        def custom_type_checks(prop: str, obj: CrossController) -> CrossController:
            """ Check type of linked element(s) by custom type """


            def notice_not_suited_object_selected(element, replacements: dict, prop: str):
                error(
                    'Selected not suited object (' + ros_name(element) + ') for '
                    + prop + ' property. Object was excluded. Verification functions - '
                    + ', '.join(replacements[prop]['check_functions'])
                    + '. Use filter of this type.', gui=True,
                )

            # custom type checks
            replacements = wb_constants.ROS2_CONTROLLERS_PARAMS_TYPES_REPLACEMENTS
            if prop in replacements:
                filtered_elements = deepcopy(replacements[prop]['default_value_replace'])
                check_functions = replacements[prop]['check_functions']
                attr = getattr(obj, prop)
                if type(attr) is list:
                    for element in attr:
                        if element:
                            suited_object_flag = False
                            for check_function in check_functions:
                                if globals()[check_function](element):
                                    suited_object_flag = True
                                    filtered_elements.append(element)
                                    break
                            if not suited_object_flag:
                                notice_not_suited_object_selected(element, replacements, prop)


                else:
                    if attr:
                        suited_object_flag = False
                        for check_function in check_functions:
                            if globals()[check_function](attr):
                                suited_object_flag = True
                                filtered_elements = attr
                                break
                        if not suited_object_flag:
                            notice_not_suited_object_selected(attr, replacements, prop)

                if attr != filtered_elements:
                    setattr(obj, prop, filtered_elements)

            return obj


        def map_mapable_params(prop: str, obj: CrossController):
            """ Do mapping for every mapable element (f.e. joint) by source attr (f.e. dof_names or joints) of mapping
            and map templates (present params with included map marker).

            See description of functions get_mapped_params(), map_param()
            """

            # map mapable params
            mapped_params_templates, params_to_map = get_mapped_params(obj)
            if len(params_to_map):
                for param_to_map in params_to_map:
                    if prop == param_to_map:
                        map_param(param_to_map, mapped_params_templates, obj)

            return obj


        obj = custom_type_checks(prop, obj)

        # remove mapped attr after source joints was removed from source of mapping attr
        obj = remove_redundant_mapped_attrs(prop, obj)


        obj = map_mapable_params(prop, obj)


    def onChanged(self, vobj: VPDO, prop: str):
        controller: CrossController = vobj.Object

    def setupContextMenu(self, vobj: VPDO, menu: QMenu) -> None:
        return

    def doubleClicked(self, vobj: VPDO):
        gui_doc = vobj.Document
        if not gui_doc.getInEdit():
            gui_doc.setEdit(vobj.Object.Name)
        else:
            error('Task dialog already active')
        return True

    def setEdit(self, vobj: VPDO, mode):
        return False

    def unsetEdit(self, vobj: VPDO, mode):
        import FreeCADGui as fcgui
        fcgui.Control.closeDialog()

    def dumps(self):
        return None

    def loads(self, state) -> None:
        pass

    def on_context_menu(self, vobj: VPDO) -> None:
        pass


def make_broadcaster(controller_data: dict, doc: Optional[fc.Document] = None) -> CrossController | None:
    dynamicType = ControllerProxy.BroadcasterType

    return make_controller(controller_data, dynamicType, doc)


def make_controller(controller_data: dict, dynamicType = None, doc: Optional[fc.Document] = None) -> CrossController | None:
    """Add a Cross::Controller to the current document."""

    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return

    controller: CrossController = doc.addObject('App::FeaturePython', controller_data['name'])
    ControllerProxy(controller, dynamicType)

    add_controller_properties_block(controller, controller_data)

    # add meta property
    prop_name = 'plugin_class_name'
    controller, used_property_name = add_property(
        controller,
        'App::PropertyString',
        prop_name,
        'Internal',
        'Plugin class full name',
        controller_data['controller_plugin_class_name'],
    )
    controller.setPropertyStatus(prop_name, ['Hidden', 'ReadOnly'])

    get_mapped_params(controller) # fill meta propeties about mapped params

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        _ViewProviderController(controller.ViewObject)
    else:
        error('Parameters of ' + controller_data['name'] + ' not received, interrupt.', True)
        doc.recompute()
        return None

    doc.recompute()
    return controller


def add_controller_properties_block(controller: CrossController, controller_data: dict) -> CrossController:

    controller = add_controller_properties(
        controller,
        {controller_data['name']: controller_data['parameters']},
        controller_data['name'],
    )

    adding_flatten_params = flatten_params(controller_data['parameters'], flat_params = {})
    parameters_flatten = deepcopy(controller_data['parameters_flatten'])
    controller_data['parameters_flatten'] = {
        **parameters_flatten,
        **adding_flatten_params,
    }
    parameters_flatten_full_names = controller_data['parameters_flatten'].keys()
    prop_name = 'controller_parameters_fullnames_list'
    # add meta property
    # there are only list of full names of controller parameters (gotten from controller YAML config)
    if hasattr(controller, prop_name):
        setattr(controller, prop_name, parameters_flatten_full_names)
    else:
        controller, used_property_name = add_property(
            controller,
            'App::PropertyStringList',
            prop_name,
            'Internal',
            'List of full names of parameters',
            parameters_flatten_full_names,
        )
        controller.setPropertyStatus(prop_name, ['Hidden', 'ReadOnly'])


    return controller


def add_controller_properties(
    controller: CrossController,
    parameters: dict,
    parameter_name: str,
    parameter_full_name_glue: str = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE,
) -> CrossController:
    """Adding properties to controller."""

    for param_name, param in parameters[parameter_name].items():

        try:
            # type param present only in leaf element
            # and exception used for recursion call.
            # If throw exception then go recursion
            prop_type = param['type_fc']

            default_value = None
            if 'default_value' in param:
                default_value = param['default_value']

            try:
                var_name = param['full_name']
            except KeyError:
                # avoiding except of outer try block by using other type error
                raise RuntimeError('param full_name - KeyError')

            # for recursive props make category for grouping them from names of each recursion dive
            # example linear__x__has_velosity_limits
            full_name_splited = param['full_name'].split(parameter_full_name_glue)
            if len(full_name_splited) > 1:
                category = parameter_full_name_glue.join(full_name_splited[:-1])
            else:
                # root categories
                category = 'Mandatory Root'
                if default_value is None \
                or (not default_value and default_value is not False):

                    if 'description' in param:
                        if '(Optional)' in param['description'] \
                        or '(optional)' in param['description']:
                            category = 'Root'

                    if parameter_name == 'range_sensor_broadcaster' \
                    and param['full_name'] in ['radiation_type', 'variance']:
                        category = 'Root'
                    elif parameter_name == 'admittance_controller' \
                    and param['full_name'] in ['robot_description']:
                        category = 'Root'
                    elif parameter_name == 'joint_state_broadcaster' \
                    and param['full_name'] in ['extra_joints', 'interfaces', 'joints']:
                        category = 'Root'
                    elif parameter_name == 'joint_trajectory_controller' \
                    and param['full_name'] in ['cmd_timeout', 'command_joints']:
                        category = 'Root'
                    elif parameter_name in [
                        'bicycle_steering_controller',
                        'tricycle_steering_controller',
                        'ackermann_steering_controller',
                        'diff_drive_controller',
                        'tricycle_controller',
                    ] \
                    and param['full_name'] in ['base_frame_id', 'odom_frame_id']:
                        category = 'Root'
                else:
                    category = 'Root'

            # make description
            help_txt = ''
            if 'description' in param:
                help_txt = param['description']

            if 'validation_str' in param:
                help_txt += '\n\n' + param['validation_str']

            # add property
            controller, used_property_name = add_property(
                controller,
                prop_type,
                var_name,
                category,
                help_txt,
                default_value,
            )

            if is_controllers_template_for_param_mapping(var_name):
                controller.setPropertyStatus(var_name, ['Hidden', 'ReadOnly'])

        except KeyError:
            # not type finded at this level and should dive deeper
            controller = add_controller_properties(
                controller,
                parameters[parameter_name],
                param_name,
            )

    return controller


def get_controllers_data(ROS2_CONTROLLERS_PATH: Path = ROS2_CONTROLLERS_PATH) -> dict :
    ''' Get controllers data. '''

    def collect_controllers_parameters(controllers: dict) -> dict :
        ''' Adding to controllers their collected parameters. '''

        for controller_name in controllers:
            controller = controllers[controller_name]

            for path in list(controller['parameters_path'].values()):
                with open(path) as stream:
                    if 'parameters' in controllers[controller_name]:
                        controllers[controller_name]['parameters'].update(yaml.safe_load(stream))
                    else:
                        controllers[controller_name]['parameters'] = yaml.safe_load(stream)

        return controllers


    def collect_controllers_plugin_data(controllers: dict) -> dict :
        ''' Adding to controllers their plugin data. '''

        for dir_name in controllers:
            controller = controllers[dir_name]

            for path in list(controller['plugin_path'].values()):
                root = ET.parse(path).getroot()

                for type_tag in root.findall('class'):
                    full_class_name = type_tag.get('name')
                    type = type_tag.get('type')
                    base_class_type = type_tag.get('base_class_type')
                    class_name = full_class_name.split('/')[-1]

                    # special cases for some of same controller names in xml
                    if 'GripperActionController' in full_class_name:
                        gripper_interface = re.search(r'(.+)_', full_class_name).group(1) # effor, position
                        class_name = class_name + gripper_interface.capitalize()
                        controller['parameters']['gripper_action_controller_' + gripper_interface] = deepcopy(controller['parameters']['gripper_action_controller'])

                    camelCaseToList = re.findall(r'[A-Za-z](?:[a-z]+|[A-Z]*(?=[A-Z]|$))', class_name)
                    controller_name = '_'.join(camelCaseToList).lower()
                    plugin = {
                        full_class_name:
                          {
                            'name': controller_name,
                            'full_name': full_class_name,
                            'type': type,
                            'base_class_type': base_class_type,
                            'description': type_tag.find('description').text.strip(),
                          },
                    }

                    if 'plugins_data' in controllers[dir_name]:
                        controllers[dir_name]['plugins_data'].update(plugin)
                    else:
                        controllers[dir_name]['plugins_data'] = plugin

        # special cases
        # there are 2 gripper_action_controller (effor, position) in dir and only 1 yaml config
        # above split controller and now delete origin gripper_action_controller
        try:
            del controllers['gripper_controllers']['parameters']['gripper_action_controller']
        except KeyError:
            pass

        return controllers

    
    def add_custom_controllers_parameters(controllers_dirs: dict) -> dict:
        '''Add custom parameters to controller. 
        It needed for addition custom logics to controller on extended code generator side.'''
        
        try:
            imitate_mecanum_by_friction = {
                'type': 'bool',
                'default_value': True,
                'description': 'If option is "true" diagonal friction will be added to collisions of wheels in generated SDF code.\n \
In that case wheel collisions should be spheres (radius same as wheel radius) and must be set base_frame_id.\n \
base_frame_id must be first frame of mobile platform.\n \
base_frame orientation must be directed: front to Y-positive, right side to X-positive.\n \
You can check orientation by FreeCAD global axis.',
                'read_only': False
            }
            controllers_dirs['mecanum_drive_controller']['parameters']['mecanum_drive_controller']['imitate_mecanum_by_friction'] = imitate_mecanum_by_friction
        except KeyError:
            pass

        return controllers_dirs


    controllers = get_controllers_root_dirs(ROS2_CONTROLLERS_PATH)
    controllers['controllers_dirs'] = filter_controllers_dirs(controllers['controllers_dirs'])

    controllers['controllers_dirs'] = collect_controllers_config_files(controllers['controllers_dirs'])
    controllers['broadcasters_dirs'] = collect_controllers_config_files(controllers['broadcasters_dirs'])
    controllers['controllers_dirs'] = collect_controllers_parameters(controllers['controllers_dirs'])
    controllers['broadcasters_dirs'] = collect_controllers_parameters(controllers['broadcasters_dirs'])
    
    controllers['controllers_dirs'] = add_custom_controllers_parameters(controllers['controllers_dirs'])

    controllers['controllers_dirs'] = collect_controllers_plugin_data(controllers['controllers_dirs'])
    controllers['broadcasters_dirs'] = collect_controllers_plugin_data(controllers['broadcasters_dirs'])

    controllers['controllers'] = separate_controllers_from_dirs(controllers['controllers_dirs'])
    controllers['broadcasters'] = separate_controllers_from_dirs(controllers['broadcasters_dirs'])

    return controllers


def filter_controllers_dirs(controllers: dict):
    """Filter controllers directories for that not coded parsing yet"""

    # filted does not adapted controller (need some code to adapt)
    filter_controllers_dirs = ['parallel_gripper_controller', 'gpio_controllers']

    for controller in list(controllers):
        if controller in filter_controllers_dirs:
            del controllers[controller]

    return controllers


def add_full_name_to_params(
    params: dict,
    param_name_prefix: list = [],
    parameter_full_name_glue: str = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE,
) -> dict:
    ''' Add full name with parent prefixes to every param.

    Params can be at various level of nested deep.
    full_param_name means all parents prefixes + param_name joined with parameter_full_name_glue
    '''

    for param_name, param in params.items():
        try:
            # param['type'] - KeyError trigger to recursion because type present only in leaf element
            type = param['type']

            full_param_name = parameter_full_name_glue.join(param_name_prefix + [param_name])
            params[param_name]['full_name'] = full_param_name
        except KeyError:
            param_name_prefix.append(param_name)
            params[param_name] = add_full_name_to_params(param, param_name_prefix)
            param_name_prefix.pop()

    return params


def flatten_params(params: dict, flat_params: dict) -> dict:
    ''' flattens parameters dict.
    Set full (with parent prefixes) param name as root dict attribute and this way make 1 level parameters
    '''

    for param_name, param in params.items():
        try:
            # param['type'] - KeyError trigger to recursion because type present only in leaf element
            type = params['type']

            flat_params[params['full_name']] = params
        except KeyError:
            flat_params = flatten_params(param, flat_params)

    return flat_params


def unflatten_params(
    flatten_params: dict,
    param_to_replace: str | None = None,
    replace: str | None = None,
    parameter_full_name_glue: str = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE,
) -> dict:
    ''' Unflattens parameters dict.

    Split flatten params name by parameter_full_name_glue and make nested structure with list
    '''

    def unflatten_params_recursion(params, param_name: str, leaf_param_data: dict = {}, params_nest_level_name: str = ''):
        params_nest = param_name.split(parameter_full_name_glue)
        params_nest_level_name = params_nest.pop(0)

        if param_to_replace is not None and replace is not None:
            if params_nest_level_name == param_to_replace:
                params_nest_level_name = replace
                leaf_param_data['full_name'] = leaf_param_data['full_name'].replace(param_to_replace, replace)

        result_params = {}

        if len(params_nest) > 0:
            params = unflatten_params_recursion(params, parameter_full_name_glue.join(params_nest), leaf_param_data, params_nest_level_name)
            result_params = {params_nest_level_name: params}
        else:
            result_params = {params_nest_level_name: leaf_param_data}

        return result_params


    params = {}
    for param_name, leaf_param_data in flatten_params.items():
        params = deepmerge(unflatten_params_recursion(params, param_name, leaf_param_data), params)

    return params


def separate_controllers_from_dirs(controllers_dirs: dict) -> dict :
    ''' Separate controllers from controllers directories dictionaries.

    Directory can contains several controllers.
    This make controllers as first level properties of dict.
    '''

    def add_fc_types_based_on_params_types(params: dict, param_name_prefix: list = []) -> dict:
        ''' Return params with FreeCAD types in addition to ros2_controllers parameters types
        '''

        # some types required to be replaced to more suited because can be more convenient to use
        replacement = wb_constants.ROS2_CONTROLLERS_PARAMS_TYPES_REPLACEMENTS
        for param_name, param in params.items():
            try:
                # param['type'] - KeyError trigger to recursion because type present only in leaf element
                params[param_name]['type_fc'] = wb_constants.ROS2_CONTROLLERS_PARAMS_TO_FRECAD_PROP_MAP[param['type']]
                params[param_name]['type_fc_origin'] = param['type_fc']
                if 'default_value' in param:
                    params[param_name]['default_value_origin'] = param['default_value']

                # full_param_name means param_name + all parents prefixes
                full_param_name = params[param_name]['full_name']
                if full_param_name in replacement:
                    params[param_name]['type_fc'] = replacement[full_param_name]['replace']
                    # some types of replaced params must have other default value
                    if 'default_value_replace' in replacement[full_param_name]:
                        params[param_name]['default_value'] = replacement[full_param_name]['default_value_replace']
            except KeyError:
                param_name_prefix.append(param_name)
                params[param_name] = add_fc_types_based_on_params_types(param, param_name_prefix)
                param_name_prefix.pop()

        return params


    def exclude_params(params: dict, excluded_params: list) -> dict:
        ''' Return params without exluded params
        '''

        params_without_exluded = {}
        for param_name, param in params.items():
            if param_name not in excluded_params:
                params_without_exluded[param_name] = param

        return params_without_exluded


    def add_validation_rules_str_to_params(params: dict) -> dict:
        ''' Add validation rules string to every param with validation.

        It convert rules to string and to every param where present validation.
        '''

        for param_name, param in params.items():
            validation_str = ''
            if 'validation' in param:
                for validation_rule, validation_value in param['validation'].items():

                    rule_desc = 'Validation rule: '
                    try:
                        rule_desc += wb_constants.ROS2_CONTROLLERS_VALIDATION_RULES_DESCRIPTIONS[validation_rule.rstrip("<>")]
                    except KeyError:
                        rule_desc += validation_rule.rstrip("<>")

                    validation_value_str = ''
                    if validation_value:
                        validation_value_str += '. Validation value: '

                        try:
                            validation_value_str += ', '.join(str(el) for el in validation_value)
                        except:
                            validation_value_str += str(validation_value)

                    validation_str += rule_desc + validation_value_str + '\n'

                params[param_name]['validation_str'] = validation_str

        return params


    excluded_params = wb_constants.ROS2_CONTROLLERS_EXCLUDED_PARAMS

    controllers = {}
    for controller_dir_name, controller_dir in controllers_dirs.items():
        if 'plugins_data' in controller_dir:
            for plugin_class_name, plugin_data in controller_dir['plugins_data'].items():

                plugin_data_name = plugin_data['name']

                # prepare controller parameters
                parameters = {}
                try:
                    parameters = controller_dir['parameters'][plugin_data_name]
                except KeyError:
                    # special cases for controllers that does not have their own params and uses params from other controllers
                    params_based_on_controller_dir = excluded_params[plugin_data_name]['params_based_on_controller_dir']
                    params_based_on_controller_name = excluded_params[plugin_data_name]['params_based_on_controller_name']
                    params_to_exlude = excluded_params[plugin_data_name]['excluded_params']
                    parameters = exclude_params(
                        controllers_dirs[params_based_on_controller_dir]['parameters'][params_based_on_controller_name],
                        params_to_exlude,
                    )
                if plugin_data_name in ['ackermann_steering_controller', 'bicycle_steering_controller', 'tricycle_steering_controller']:
                    # must add steering_controllers_library params to steering controllers
                    parameters.update(controllers_dirs['steering_controllers_library']['parameters']['steering_controllers_library'])

                parameters = add_full_name_to_params(parameters)
                parameters = add_fc_types_based_on_params_types(parameters)
                parameters = add_validation_rules_str_to_params(parameters)
                parameters_flatten = flatten_params(parameters, flat_params = {})


                # assembly controller dict
                controllers[plugin_data_name] = {
                    'name': plugin_data_name,
                    'description': plugin_data['description'],
                    'controller_path': controller_dir['dir'],
                    'plugin_path': controller_dir['plugin_path'],
                    'parameters_path': controller_dir['parameters_path'],
                    'parameters': parameters,
                    'parameters_flatten': parameters_flatten,
                    'controller_dir_name': controller_dir_name,
                    'controller_plugin_class_name': plugin_class_name,
                }

    return controllers


def get_controllers_root_dirs(ROS2_CONTROLLERS_PATH: Path) -> dict :
    ''' Get controllers root dirs. '''
    controllers_and_broadcusters = {}

    controller_key_words = ['controller']
    controller_key_words_blacklist = ['test', 'ros2_controllers', 'rqt']
    controllers = get_files_or_dirs_by_filter(
        ROS2_CONTROLLERS_PATH,
        files_or_dirs = 'dirs',
        key_words = controller_key_words,
        key_words_blacklist = controller_key_words_blacklist,
        attr_name_for_found_result = 'dir',
    )

    controller_key_words = ['broadcaster']
    broadcasters = get_files_or_dirs_by_filter(
        ROS2_CONTROLLERS_PATH,
        files_or_dirs = 'dirs',
        key_words = controller_key_words,
        key_words_blacklist = controller_key_words_blacklist,
        attr_name_for_found_result = 'dir',
    )

    controllers_and_broadcusters['controllers_dirs'] = controllers
    controllers_and_broadcusters['broadcasters_dirs'] = broadcasters

    return controllers_and_broadcusters


def collect_controllers_config_files(controllers: dict) -> dict :
    ''' Get controllers plugin xml file and parameters yaml file. '''

    for controller_folder_name, controller_value in controllers.items():

        plugin = get_files_or_dirs_by_filter(
            controller_value['dir'],
            key_words = ['.xml'],
            key_words_blacklist = ['package.xml'],
        )
        controllers[controller_folder_name].update({'plugin_path': plugin})

        parameters = get_files_or_dirs_by_filter(
            controller_value['dir'] / 'src',
            key_words = ['.yaml'],
        )
        controllers[controller_folder_name].update({'parameters_path': parameters})

    return controllers


def get_files_or_dirs_by_filter(
    dir: Path,
    files_or_dirs:str = 'files',
    key_words:list = [],
    key_words_blacklist:list = [],
    attr_name_for_found_result:str | None = None,
) -> dict :
    ''' Get files or dirs in path by included keywords in file name. '''

    files_or_dirs_result = {}
    for root, dirs, files in os.walk(dir):
        for name in eval(files_or_dirs):
            # check key words
            for key_word in key_words:
                if key_word in name:

                    # check blacklist
                    name_in_blacklist = False
                    for key_word_bl in key_words_blacklist:
                        if key_word_bl in name:
                            name_in_blacklist = True
                            break

                    # forming result
                    if not name_in_blacklist:
                        if name not in files_or_dirs_result:
                            if not attr_name_for_found_result:
                                files_or_dirs_result[name] = {Path(root) / name}
                            else:
                                files_or_dirs_result[name] = {}

                        if not attr_name_for_found_result:
                            files_or_dirs_result.update({name: Path(root) / name})
                        else:
                            files_or_dirs_result.update({name: {attr_name_for_found_result: Path(root) / name}})

        break

    return files_or_dirs_result


def get_mapped_params(
    obj: CrossController,
    parameter_full_name_glue: str = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE,
) -> tuple[dict, dict]:
    """Get mapped params templates and params for mapping.

    mapped params templates - params that is templates for mapping other params (ex: gains_____map_joints___p)
    params for mapping - params that should be mapped by teplate (ex: joints)

    after mapping param name will be like - gains___joint_name___p
    """

    prop_name_mapped_params_templates = 'mapped_params_templates'
    prop_name_params_to_map = 'params_to_map'

    try:
        mapped_params_templates = getattr(obj, prop_name_mapped_params_templates)
        params_to_map = getattr(obj, prop_name_params_to_map)
    except AttributeError:
        # calculate and fill attrs

        param_map_marker = wb_constants.ROS2_CONTROLLERS_PARAM_MAP_MARKER
        mapped_params_templates = {}
        for param_full_name in obj.controller_parameters_fullnames_list:
            if param_map_marker in param_full_name:
                mapped_params_templates[param_full_name] = param_full_name

        params_to_map = {}
        for mapped_params_template in mapped_params_templates:
            param_name_to_map = re.search(param_map_marker + '(.+)' + parameter_full_name_glue, mapped_params_template).group(1)
            params_to_map[param_name_to_map] = param_name_to_map


        # add meta property
        obj, used_property_name = add_property(
            obj,
            'App::PropertyStringList',
            prop_name_mapped_params_templates,
            'Internal',
            'Mapped params templates (ex: gains_____map_joints___p)',
            mapped_params_templates.keys(),
        )
        obj.setPropertyStatus(prop_name_mapped_params_templates, ['Hidden', 'ReadOnly'])

        # add meta property
        obj, used_property_name = add_property(
            obj,
            'App::PropertyStringList',
            prop_name_params_to_map,
            'Internal',
            'Params to map (ex: joints) when present mapped params templates (ex: gains_____map_joints___p). \
                Ex: after map gains___joint_name___p.',
            params_to_map.keys(),
        )
        obj.setPropertyStatus(prop_name_params_to_map, ['Hidden', 'ReadOnly'])

    return mapped_params_templates, params_to_map
