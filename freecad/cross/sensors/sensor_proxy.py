"""Proxy for Cross::Sensor FreeCAD objects

A sensors are representation of Gazebo sensors https://gazebosim.org/docs/latest/sensors/ gotten from handly maked sdf files (resources/sensors)
with added meta data (type, descriptions of fields, etc) from sdformat package (modules/sdformat) schema files via sdf_tree().
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
import xmltodict

import FreeCAD as fc

from PySide.QtWidgets import QMenu  # FreeCAD's PySide

from ..freecad_utils import ProxyBase
from ..freecad_utils import add_property
from ..freecad_utils import error
from ..freecad_utils import get_valid_property_name
from ..freecad_utils import warn
from ..wb_utils import ICON_PATH
from ..wb_utils import SENSORS_DATA_PATH
from ..wb_utils import SDFORMAT_PATH
from ..wb_utils import MODULES_PATH
from ..wb_utils import is_joint
from ..wb_utils import is_link
from ..wb_utils import is_sensor
from ..wb_utils import return_true
from ..wb_utils import ros_name
from ..wb_utils import get_valid_urdf_name
from ..utils import deepmerge, replace_substring_in_keys
from .. import wb_constants
from ..exceptions import CallRecursion
from ..sdf.sdf_parser.sdf_schema_parser import sdf_schema_parser
from ..sdf.sdf_parser.sdf_tree import sdf_tree

# Stubs and type hints.
from .sensor import Sensor as CrossSensor  # A Cross::Sensor, i.e. a DocumentObject with Proxy "Sensor". # noqa: E501
DO = fc.DocumentObject
DOList = List[DO]
VPDO = ForwardRef('FreeCADGui.ViewProviderDocumentObject')  # Don't want to import FreeCADGui here. # noqa: E501
AppLink = DO  # TypeId == 'App::Link'.
check_functions_required = [is_joint, is_link, return_true, is_sensor] # dont remove used by check_functions()

class SensorProxy(ProxyBase):
    """The proxy for Sensor objects."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Cross::Sensor'

    def __init__(self, obj: CrossSensor):
        super().__init__(
            'sensor', [
            '_Type',
            ],
        )

        if obj.Proxy is not self:
            obj.Proxy = self
        self.sensor = obj

        self._init_properties(obj)


    def _init_properties(self, obj: CrossSensor):
        add_property(
            obj, 'App::PropertyString', '_Type', 'Internal',
            'The type',
        )
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type


    def execute(self, obj: CrossSensor) -> None:
        pass


    def onChanged(self, obj: CrossSensor, prop: str) -> None:
        pass


    def onDocumentRestored(self, obj):
        """Restore attributes because __init__ is not called on restore."""
        self.__init__(obj)


    def dumps(self):
        return None


    def loads(self, state) -> None:
        pass


class _ViewProviderSensor(ProxyBase):
    """A view provider for the Sensor container object """

    def __init__(self, vobj: VPDO):
        super().__init__(
            'view_object',
            [
                'Visibility',
            ],
        )
        vobj.Proxy = self

    def getIcon(self):
        # Implementation note: "return 'sensor.svg'" works only after
        # workbench activation in GUI.
        return str(ICON_PATH / 'sensor.svg')

    def attach(self, vobj: VPDO):
        self.view_object = vobj
        self.sensor = vobj.Object

    def updateData(self, obj: CrossSensor, prop: str):
        return

    def onChanged(self, vobj: VPDO, prop: str):
        sensor: CrossSensor = vobj.Object

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


def add_sensor_properties_block(sensor: CrossSensor, sensor_data: dict) -> CrossSensor:

    sensor = add_sensor_properties(
        sensor,
        {sensor_data['name']: sensor_data['parameters']},
        sensor_data['name'],
    )

    adding_flatten_params = flatten_params(sensor_data['parameters'], flat_params = {})
    parameters_flatten = deepcopy(sensor_data['parameters_flatten'])
    sensor_data['parameters_flatten'] = {
        **parameters_flatten,
        **adding_flatten_params,
    }
    parameters_flatten_full_names = sensor_data['parameters_flatten'].keys()
    prop_name = 'sensor_parameters_fullnames_list'
    # add meta property
    # there are only list of full names of sensor parameters (gotten from sensor YAML config)
    if hasattr(sensor, prop_name):
        setattr(sensor, prop_name, parameters_flatten_full_names)
    else:
        sensor, used_property_name = add_property(
            sensor,
            'App::PropertyStringList',
            prop_name,
            'Internal',
            'List of full names of parameters',
            parameters_flatten_full_names,
        )
        sensor.setPropertyStatus(prop_name, ['Hidden', 'ReadOnly'])

    return sensor


def add_sensor_properties(
    sensor: CrossSensor,
    parameters: dict,
    parameter_name: str,
    parameter_full_name_glue: str = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE,
) -> CrossSensor:
    """Adding properties to sensor."""

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
                else:
                    category = 'Root'

            # make description
            help_txt = ''
            if 'description' in param:
                help_txt = param['description']

            if prop_type in wb_constants.TYPE_CONVERT_FUNCTIONS:
                default_value = wb_constants.TYPE_CONVERT_FUNCTIONS[prop_type](default_value)

            # add property
            sensor, used_property_name = add_property(
                sensor,
                prop_type,
                var_name,
                category,
                help_txt,
                default_value,
            )

        except KeyError:
            # the type is not found at this level and should dive deeper
            sensor = add_sensor_properties(
                sensor,
                parameters[parameter_name],
                param_name,
            )

    return sensor


def get_sensors_data(SENSORS_PATH: Path = SENSORS_DATA_PATH) -> dict :
    ''' Get sensors data. '''

    def collect_sensors_parameters(sensors_dirs: dict) -> dict :
        ''' Adding to sensors their collected parameters. '''

        for sensor_attached_to in sensors_dirs:
            sensors_dir_data = sensors_dirs[sensor_attached_to]
            for sensor in list(sensors_dir_data['sensors'].values()):
                with open(sensor['path']) as stream:
                    try:
                        data_dict = xmltodict.parse(stream.read())
                    except:
                        pass

                    if 'file_data' in sensor:
                        sensor['file_data'].update(data_dict)
                    else:
                        sensor['file_data'] = data_dict

                    sensor_data = data_dict['sdf']['world'][sensor_attached_to]['sensor']
                    # remove @name because we will use sensor name from FC instead of sensor name from sdf
                    sensor_parameters = {key: value for key, value in sensor_data.items() if not key.startswith('@name')}

                    # replace attr prefix (@) with 'attr_' because @ cant be save in prop name
                    sensor_parameters = replace_substring_in_keys(
                        sensor_parameters,
                        wb_constants.XMLTODICT_ATTR_PREFIX_ORIGIN,
                        wb_constants.XMLTODICT_ATTR_PREFIX_FIXED_FOR_PROP_NAME,
                    )

                    if 'parameters' in sensor:
                        sensor['parameters'].update(sensor_parameters)
                    else:
                        sensor['parameters'] = sensor_parameters

        return sensors_dirs


    def add_schema_data(sensors, sensor_schema_as_dict) -> dict :

        def add_param_data(parameter: dict, parameter_name: str, sensor_schema_as_dict: dict) -> dict:
            """Add data (like data, description) to paramater from sdf schema"""
            for key in list(parameter):
                elem = parameter[key]
                try:
                    if isinstance(elem, dict) and '#text' not in elem:
                        raise CallRecursion('go recursion to leaf element of dict')

                    if not isinstance(elem, dict) and not isinstance(elem, list) and '#text' not in key:
                        parameter[key] = {'#text': elem}

                    index_type = sdf_schema_parser.get_technical_attr_prefix_with_attr_symbol() + 'type'
                    index_description = sdf_schema_parser.get_technical_attr_prefix_with_attr_symbol() + 'description'
                    if '#text' != key:
                        try:
                            parameter[key][index_type] = sensor_schema_as_dict[parameter_name][key][index_type]
                            parameter[key][index_description] = sensor_schema_as_dict[parameter_name][key][index_description]
                        except (KeyError, TypeError):
                            pass
                    else:
                        try:
                            parameter[index_type] = sensor_schema_as_dict[parameter_name][index_type]
                            parameter[index_description] = sensor_schema_as_dict[parameter_name][index_description]
                        except (KeyError, TypeError):
                            pass

                except CallRecursion:
                    parameter[key] = add_param_data(elem, key, sensor_schema_as_dict[parameter_name])

            return parameter


        for attached_group in sensors['sensors_dirs']:
            sensors_of_attached_group = sensors['sensors_dirs'][attached_group]['sensors']

            for sensor_name in sensors_of_attached_group:

                for parameter_name, parameter in sensors_of_attached_group[sensor_name]['parameters'].items():
                    if not isinstance(parameter, dict):
                        parameter_value = parameter
                        parameter = {'#text': parameter_value}
                    sensors_of_attached_group[sensor_name]['parameters'][parameter_name] = add_param_data(parameter, parameter_name, sensor_schema_as_dict)

            sensors['sensors_dirs'][attached_group]['sensors'] = sensors_of_attached_group

        return sensors


    tree=sdf_tree("sensor.sdf")
    sensor_schema_as_dict=tree.get_element_as_dict['sensor']

    sensors = get_sensors_root_dirs(SENSORS_PATH)
    sensors['sensors_dirs'] = collect_sensors_files_grouped_by_dirs(sensors['sensors_dirs'])
    sensors['sensors_dirs'] = collect_sensors_parameters(sensors['sensors_dirs'])
    sensors = add_schema_data(sensors, sensor_schema_as_dict)
    sensors = separate_sensors_from_dirs(sensors['sensors_dirs'])

    return sensors


def add_full_name_to_params(
    params: dict,
    param_name_prefix: list = [],
    parameter_full_name_glue: str = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE,
) -> dict:
    ''' Add full name with parent prefixes to every param.

    Params can be at various levels of nested deep.
    full_param_name means all parents prefixes + param_name joined with parameter_full_name_glue
    '''

    for param_name, param in params.items():
        try:
            if isinstance(param, dict) and '#text' not in param:
                raise CallRecursion('go recursion to leaf element of dict')

            full_param_name = parameter_full_name_glue.join(param_name_prefix + [param_name])

            if not isinstance(param, dict):
                params[param_name] = {}
                params[param_name]['#text'] = param

            params[param_name]['full_name'] = full_param_name
            params[param_name]['default_value'] = params[param_name].get('#text', '')
        except CallRecursion:
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
            if isinstance(param, dict) and '#text' not in param:
                raise CallRecursion('go recursion to leaf element of dict')

            flat_params[param['full_name']] = param
        except CallRecursion:
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


def separate_sensors_from_dirs(sensors_dirs: dict) -> dict :
    ''' Separate sensors from sensors directories dictionaries.

    Directory can contains several sensors.
    This make sensors as first level properties of dict.
    '''

    def add_fc_types_based_on_params_types(params: dict, param_name_prefix: list = []) -> dict:
        ''' Return params with FreeCAD types in addition to ros2_sensors parameters types
        '''

        # some types required to be replaced to more suited because can be more convenient to use
        replacement = wb_constants.ROS2_CONTROLLERS_PARAMS_TYPES_REPLACEMENTS
        for param_name, param in params.items():
            try:
                if isinstance(param, dict) and '#text' not in param:
                    raise CallRecursion('go recursion to leaf element of dict')

                try:
                    type = param[sdf_schema_parser.get_technical_attr_prefix_with_attr_symbol() + 'type']

                    try:
                        type_value = type['#text']
                    except (KeyError, TypeError):
                        type_value = type

                    type_fc = wb_constants.ROS2_CONTROLLERS_PARAMS_TO_FRECAD_PROP_MAP[type_value]
                except KeyError:
                    type = 'string'
                    type_fc = wb_constants.ROS2_CONTROLLERS_PARAMS_TO_FRECAD_PROP_MAP[type]

                params[param_name]['type_fc'] = type_fc
                params[param_name]['type_fc_origin'] = param['type_fc']

            except CallRecursion:
                param_name_prefix.append(param_name)
                params[param_name] = add_fc_types_based_on_params_types(param, param_name_prefix)
                param_name_prefix.pop()

        return params


    sensors = {}
    for sensor_dir_name, sensor_dir in sensors_dirs.items():
        for sensor_name, sensor in sensor_dir['sensors'].items():

            # prepare sensor parameters
            parameters = sensor['parameters']

            parameters = add_full_name_to_params(parameters)
            parameters = add_fc_types_based_on_params_types(parameters)
            parameters_flatten = flatten_params(parameters, flat_params = {})


            if sensor_dir_name not in sensors:
                sensors[sensor_dir_name] = {}

            # assembly sensor dict
            sensors[sensor_dir_name][sensor_name] = {
                'name': sensor_name,
                'type': sensor.get('type', ''),
                'description': sensor.get('description', ''),
                'sensor_path': sensor.get('path', ''),
                'parameters': parameters,
                'parameters_flatten': parameters_flatten,
                'sensor_dir_name': sensor_dir_name,
            }

    return sensors


def get_sensors_root_dirs(SENSORS_PATH: Path = SENSORS_DATA_PATH) -> dict :
    ''' Get sensors root dirs. '''
    sensors = {}

    sensor_key_words = ['joint', 'link'] # , 'model'
    sensor_key_words_blacklist = []
    sensors_dirs = get_files_or_dirs_by_filter(
        SENSORS_PATH,
        files_or_dirs = 'dirs',
        key_words = sensor_key_words,
        key_words_blacklist = sensor_key_words_blacklist,
        attr_name_for_found_result = 'dir',
    )

    sensors['sensors_dirs'] = sensors_dirs

    return sensors


def collect_sensors_files_grouped_by_dirs(sensors_dirs: dict) -> dict :
    ''' Get sensors files. '''

    for sensor_folder_name, sensor_value in sensors_dirs.items():

        sensors_files = get_files_or_dirs_by_filter(
            sensor_value['dir'],
            key_words = ['.sdf'],
            key_words_blacklist = [],
            attr_name_for_found_result = 'path',
        )
        sensors_dirs[sensor_folder_name].update({'sensors': sensors_files})

    return sensors_dirs


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

                    # cut extention
                    name_without_file_extention = name.split('.')[0]

                    # forming result
                    if not name_in_blacklist:
                        if name not in files_or_dirs_result:
                            if not attr_name_for_found_result:
                                files_or_dirs_result[name_without_file_extention] = {Path(root) / name}
                            else:
                                files_or_dirs_result[name_without_file_extention] = {attr_name_for_found_result: Path(root) / name}

                        if not attr_name_for_found_result:
                            files_or_dirs_result.update({name_without_file_extention: Path(root) / name})
                        else:
                            files_or_dirs_result.update({name_without_file_extention: {attr_name_for_found_result: Path(root) / name}})

        break

    return files_or_dirs_result
