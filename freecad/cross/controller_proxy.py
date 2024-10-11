"""Proxy for Cross::Controller and Cross::Broadcaster FreeCAD objects

A controllers and broadcasters are representation of controllers and broadcasters from ros2_controllers.

"""

from __future__ import annotations

from typing import ForwardRef, List, Optional, Union, cast
from copy import deepcopy

import FreeCAD as fc

from PySide.QtWidgets import QMenu  # FreeCAD's PySide

from .freecad_utils import ProxyBase
from .freecad_utils import add_property
from .freecad_utils import error
from .freecad_utils import get_valid_property_name
from .freecad_utils import warn
from .wb_utils import ICON_PATH
from .wb_utils import is_joint # dont remove used by check_func()
from .wb_utils import is_link # dont remove used by check_func()
from .wb_utils import return_true # dont remove used by check_func()
from .wb_utils import ros_name
from . import wb_constants

# Stubs and type hints.
from .controller import Controller as CrossController  # A Cross::Controller, i.e. a DocumentObject with Proxy "Controller". # noqa: E501
DO = fc.DocumentObject
DOList = List[DO]
VPDO = ForwardRef('FreeCADGui.ViewProviderDocumentObject')  # Don't want to import FreeCADGui here. # noqa: E501
AppLink = DO  # TypeId == 'App::Link'.
check_functions = [is_joint, is_link, return_true]

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
        super().__init__('controller', [
            '_Type',
            ])
        
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

        def notice_not_suited_object_selected(element, replacements: dict, prop: str):
            error('Selected not suited object (' + ros_name(element) + ') for ' 
                + prop + ' property. Verification function - ' 
                + replacements[prop]['check_func'] 
                + '. Use filter of this type.', gui=True)
            
        replacements = wb_constants.ROS2_CONTROLLERS_PARAMS_TYPES_REPLACEMENTS
        if prop in replacements:
            filtered_elements = deepcopy(replacements[prop]['default_value_replace'])
            check_func = replacements[prop]['check_func']
            attr = getattr(obj, prop)
            if type(attr) is list:
                for element in attr:
                    if element:
                        if not globals()[check_func](element):
                            notice_not_suited_object_selected(element, replacements, prop)
                        else:
                            filtered_elements.append(element)
            else:
                if attr:
                    if not globals()[check_func](attr):
                        notice_not_suited_object_selected(attr, replacements, prop)
                    else:
                        filtered_elements = attr
                        
            if attr != filtered_elements:
                setattr(obj, prop, filtered_elements)


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

    controller = add_controller_properties(
        controller, 
        {controller_data['name']: controller_data['parameters']},
        controller_data['name'],
        )

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        _ViewProviderController(controller.ViewObject)
    else:
        error('Parameters of ' + controller_data['name'] + ' not received, interrupt.', True)
        doc.recompute()
        return None

    doc.recompute()
    return controller


def add_controller_properties(controller: CrossController,
                            parameters: dict, 
                            parameter_name: str, 
                            parameter_name_prefix: list = [], 
                            recursion_deep: int = 0) -> CrossController | None:
    """Adding properties to controller."""


    def modify_name_prefix(parameter_name_prefix: list, parameter_name: str, param_name: str) -> list:
        """Modify parameter name prefix.
         
            Modify parameter name prefix by adding current param_name to prefix
            when going dive in recursion or remove last prefix when rise from recursion.
            
            At zero recursion level it just add root param_name to prefix.
            
            parameter_name_prefix example: ['linear', 'x', 'has_velosity_limits' ]"""


        if not parameter_name_prefix or parameter_name_prefix[-1] == parameter_name:
            parameter_name_prefix.append(param_name)
        else:
            parameter_name_prefix_reversed = reversed(parameter_name_prefix)
            for prefix in parameter_name_prefix_reversed:
                if prefix != parameter_name:
                    parameter_name_prefix.pop()
                else:
                    parameter_name_prefix.append(param_name)
                    break

        return parameter_name_prefix


    # parameters = deepcopy(parameters)

    recursion_deep += 1
    if recursion_deep > 100:
        error('Max recursion deep is reached in add_controller_property', True)
        return None

    for param_name, param in parameters[parameter_name].items():

        parameter_name_prefix = modify_name_prefix(parameter_name_prefix, parameter_name, param_name)

        var_name = get_valid_property_name('__'.join(parameter_name_prefix))

        try:
            # type param present only in leaf element
            # and exception used for recursion call.
            # If throw exception then go recursion
            prop_type = param['type_fc']


            default_value = None
            if 'default_value' in param:
                default_value = param['default_value']

            # for recursive props make category for grouping them from names of each recursion dive
            # example linear__x__has_velosity_limits 
            if len(parameter_name_prefix) > 1:
                category = '__'.join(parameter_name_prefix[:-1:])
            else:
                if default_value is None:
                    category = 'Root'
                else:
                    category = 'Root with default value'
                

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

            # remove last recursion name prefix after add property
            if parameter_name_prefix:
                parameter_name_prefix.pop()

        except KeyError:
            # not type finded at this level and should dive deeper
            controller = add_controller_properties(
                controller, 
                parameters[parameter_name],
                param_name, 
                parameter_name_prefix, 
                recursion_deep,
                )
            
            # remove last recursion name prefix after rise from recursion
            if parameter_name_prefix:
                parameter_name_prefix.pop()

    return controller
