from PySide2.QtCore import QObject, Qt, QPropertyAnimation, QEasingCurve,QSize
import FreeCADGui as fcgui
from PySide2.QtGui import QColor
from PySide2.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QPushButton,
    QSizePolicy,
    QHBoxLayout,
    QDoubleSpinBox,
    QLabel,
    QLineEdit,
    QSpinBox,
    QCheckBox,
    QGroupBox,
    QScrollArea,
    QDockWidget,
    QTabWidget,QDialogButtonBox,QSpacerItem
)
import xml.etree.ElementTree as ET
from typing import Union

from .sdf_parser import sdf_tree, sdf_schema_parser

import re
import os
from ..wb_utils import UI_PATH

debug = False
if debug:
    import pdb
"""
File will  initialize all properties that need to be defined for sdf object e.g world , links and joints
"""

# ********
# xml access
# ********
re_pattern = re.compile(
    (
        r"((\s*(-|\+)?(\d+(\.\d*)?|\.\d+|\d+\.\d+[eE][-\+]?[0-9]+)\s+){2}((-|\+)?(\d+(\.\d*)?|\.\d+|\d+\.\d+[eE][-\+]?[0-9]+))\s*)|"
        r"((\s*(-|\+)?(\d+(\.\d*)?|\.\d+|\d+\.\d+[eE][-\+]?[0-9]+)\s+){3}((-|\+)?(\d+(\.\d*)?|\.\d+|\d+\.\d+[eE][-\+]?[0-9]+))\s*)|"
        r"((\s*(-|\+)?(\d+(\.\d*)?|\.\d+|\d+\.\d+[eE][-\+]?[0-9]+)\s+)((-|\+)?(\d+(\.\d*)?|\.\d+|\d+\.\d+[eE][-\+]?[0-9]+))\s*)|"
        r"(\s*(-|\+)?\d+\s+(-|\+)?\d+\s*)|"
        r"((\s*(-|\+)?(\d+(\.\d*)?|\.\d+|\d+\.\d+[eE][-\+]?[0-9]+)\s+){5}((-|\+)?(\d+(\.\d*)?|\.\d+|\d+\.\d+[eE][-\+]?[0-9]+))\s*)|"
        r"(\d+ \d+)|"
        r"((\s*\+?(\d+(\.\d*)?|\.\d+|\d+\.\d+[eE][-\+]?[0-9]+)\s+){3}\+?(\d+(\.\d*)?|\.\d+|\d+\.\d+[eE][-\+]?[0-9]+)\s*)"
    )
)


# this will check if a string is a vector of numbers
# will be used bu get_xml_content to validate objects of type vector3,pose ... e.t.c
def assert_vect(s):
    # original
    # pattern = r'(?<![\d.-])\-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?(?:\s+\-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)+(?![\d.-])'
    # update
    return bool(re_pattern.match(s))


def extract_vector_n(input_string):
    """this extracts a vector from a string of  numbers"""
    # input_string.strip().split(' ') could aslo work here
    # well lets go with some regex
    # because this might also be used  for comma separated values

    numbers = re.findall(r"-?(?:\d+\.\d+|\.\d+|\d+)(?:e-?\d+)?", input_string)

    return [float(num) for num in numbers]


def get_xml_data(
    element: ET.Element, tag: Union[str, list], Is_Attribute: bool = False
) -> Union[list, dict, str]:
    """
    see set_xml_data()
    The functions have  similar parameters
    except for the value parameter which is not included and tag \n
    for attributes a list is used in place of tag with the parent tag at index 0 and attribute name at index 1  \n
    This e.g ['world','name'] will return the name attribute of the world element\n
    ->for none string values the caller  is responsible for converting to appropriate types"""

    def get_value(elem_data):
        if assert_vect(elem_data):
            # equivalent string
            vect_eq = extract_vector_n(elem_data)
            return vect_eq
        else:
            # try converting to int or float date type
            try:
                try:
                    return int(elem_data)
                except Exception:
                    return float(elem_data)
            except Exception:
                return elem_data

    if Is_Attribute is not True:
        elem_iter = element.iter(tag)
    else:
        elem_iter = element.iter(tag[0])
    # only a single element exists no need to use a for loop
    try:
        elem = elem_iter.__next__()
    except Exception:
        return None
    if Is_Attribute is False:
        txt = elem.text
        if txt is None:
            txt = ""
        return get_value(txt)
    else:
        # return the  the attribute dictionary
        try:
            try:
                return int(elem.attrib[tag[1]])
            except Exception:
                return float(elem.attrib[tag[1]])
        except Exception:
            return elem.attrib[tag[1]]


def set_xml_data(
    element: ET.Element,
    tag: str,
    Is_Attribute: bool,
    value: Union[dict, float, int, list, str],
) -> ET.Element:
    """
    tag is the tag name of the element to be edited \n
    Element is
    Is Attribute  can either be true or false , True if th value is an attribute \n
    if Is_Attribute is True value has to be a dictionary with one or more key value pairs \n
    The value parameter will contain the actual value to be updated
    """
    # get the affected element
    elem_iter = element.iter(tag)
    # no need for a loop
    try:
        elem = elem_iter.__next__()
    except Exception:
        return None
        # ensure no dictionaries are sent for non attributes
    if Is_Attribute is False and isinstance(value, dict) is False:
        if isinstance(value, list):
            # equivalent string
            elem.text = " ".join(map(str, value))
        else:
            elem.text = str(value)
    else:
        # add/edit  attributes
        for key in value.keys():
            elem.set(key, str(value[key]))
    return element


# *******
# xml access
# *******


# *******
# start of logic responsible for generating parameter names
# *******
# *******
# *******

reserved_names=set()
def generate_parameter_name(element_name, parent_names=None, reserved_names=None):
    """
    Generates a unique, short parameter name based on the element name and its hierarchy.

    Args:
        element_name (str): Name of the current XML element.
        parent_names (list): List of parent element names (for hierarchical uniqueness).
        reserved_names (set): Set of already-used names to avoid collisions.

    Returns:
        str: A unique parameter name (e.g., "joint_axis_xyz").
    """
    if parent_names is None:
        parent_names = []
    else:
        # ensure parent names is a list
        if not isinstance(parent_names, list):
            parent_names = [parent_names]
    if reserved_names is None:
        reserved_names = set()

    # Step 1: Shorten the element name (abbreviate if needed)
    shortened_name = _abbreviate_name(element_name)

    # Step 2: Combine with parent names for hierarchy (if provided)
    if parent_names:
        # Shorten parent names and join with underscores
        parent_prefix = "_".join(_abbreviate_name(p) for p in parent_names)
        candidate_name = f"{parent_prefix}_{shortened_name}"
    else:
        candidate_name = shortened_name

    # Step 3: Ensure the name is unique
    final_name = _ensure_unique_name(candidate_name, reserved_names)

    return final_name


def _abbreviate_name(name):
    # do nothing for now 
    return name


def _ensure_unique_name(name, reserved_names=None):
    """
    Ensures the generated name is unique by appending a counter if needed.
    Example: If "joint_ax" exists, return "joint_ax_2".
    """
    # to solve this issue a list of all used names will be maintained for now 
    #  no errors have been observed in realation to repeated names 
    if name not in reserved_names:
        reserved_names.add(name)
        return name
    else:
        counter = 2
        while f"{name}_{counter}" in reserved_names:
            counter += 1
        new_name = f"{name}_{counter}"
        reserved_names.add(new_name)
        return new_name


# ******
# *****
#  end of logic  responsible for unique name generation

# ******
# ******
# collapsible sections

class CollapsibleSection(QWidget):
    def __init__(self, title="", parent=None):
        super().__init__(parent)
        
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(0)
        
        # Header with arrow indicator
        self.header = QWidget()
        header_layout = QHBoxLayout(self.header)
        header_layout.setContentsMargins(0, 0, 0, 0)
        
        # Arrow label (using Unicode characters)
        self.arrow_label = QLabel("▶")  # Right arrow when collapsed
        self.arrow_label.setStyleSheet("""
            QLabel {
                color: #8c92ac;
                font-size: 12px;
                width: 15px;
                padding-left: 3px;
            }
        """)
        self.arrow_label.setFixedWidth(20)
        
        # Title button
        self.toggle_btn = QPushButton(title)
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.setStyleSheet("""
            QPushButton {
                text-align: left;
                border-radius: 9px
                padding: 5px;
                border: none;
                background: transparent;
                font-weight: bold;
            }
            QPushButton:hover {
            background-color: rgba(255, 255, 255, 0.15);
            border: 2px solid #aaa;
            }

        """)
        self.toggle_btn.toggled.connect(self.toggle)
        header_layout.addWidget(self.arrow_label)
        header_layout.addWidget(self.toggle_btn)
        header_layout.addStretch()
        
        # enable and disable checkbox  this will  enabled on 
        # optional elements i.e and there state will be store and used 
        # to generate sdf disabled elements wont be included 
        self.enable_check = QCheckBox()
        self.enable_check.setToolTip("activates and deactivates optional items \n only enabled items will be exported as sdf")
        self.enable_check.setChecked(True)
        self.enable_check.setStyleSheet("""
            QCheckBox {
                spacing: 0;
                padding: 0;
                margin-right: 5px;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
                border: 1px solid #999;
                border-radius: 3px;
            }
            QCheckBox::indicator:unchecked {
                background: #e0e0e0;
            }
            QCheckBox::indicator:checked {
                background: #4CAF50;
                border: 1px solid #3e8e41;
                image: none;  /* This removes the tick mark */
            }
            QCheckBox::indicator:disabled {
                background: #f0f0f0;
                border: 1px solid #ddd;
            }
        """)
        self.enable_check.stateChanged.connect(self.toggle_enabled)
        header_layout.addWidget(self.enable_check)
        
        
        # Content area
        self.content = QWidget()
        self.content.setVisible(False)
        self.content_layout = QVBoxLayout(self.content)
        self.content_layout.setContentsMargins(15, 5, 5, 5)
        
        self.layout.addWidget(self.header)
        self.layout.addWidget(self.content)
    
    def toggle(self, checked):
        # Update arrow
        self.arrow_label.setText("▼" if checked else "▶")
        
        # Toggle content
        self.content.setVisible(checked)
        
        # Update parent layout if nested
        parent = self.parent()
        while parent:
            if isinstance(parent, CollapsibleSection):
                parent.update_layout()
                break
            parent = parent.parent()
    # callback for checkbox 
    def toggle_enabled(self, state):
        """Enable/disable based on checkbox state"""
        enabled = state == Qt.Checked
        self.toggle_btn.setEnabled(enabled)
        self.content.setEnabled(enabled)
        
        # Grey out arrow when disabled
        self.arrow_label.setStyleSheet(f"color: {'#555' if enabled else '#aaa'}")
        
        # Collapse if being disabled
        if not enabled and self.toggle_btn.isChecked():
            self.toggle_btn.setChecked(False)
            self.toggle(False)
    
    def update_layout(self):
        """Force layout update for proper sizing"""
        self.content.updateGeometry()
        self.updateGeometry()
    
    def add_widget(self, widget):
        """Add any widget including nested sections"""
        self.content_layout.addWidget(widget)
    
    def add_section(self, title):
        """Convenience method to add nested section"""
        section = CollapsibleSection(title)
        self.add_widget(section)
        return section
# ******
# ******
# end of collapsible section
ignore_list = [
    "audio",
    "include",
    "pose",
    "meta",
    "visibility_flags",
    "transparency"
    ,"type",
    "axis",
    "axis2",
    "pose",
    "sensor",
    "parent",
    "child"
    
]

# this dictionary holds all names  that are already defined in FreeCAD
# this can be stored as a .json file but for now a dictionary will work 
# items are stored in the format 
# {parent_element:{name_in_sdf_template:name_in_freecad}}
# where parent element id the element where the item is defined in the sdf hierachy
defined_names:dict={}
defined_names["link"]={"name":["Label","Label2"]}
defined_names["inertia"]={"ixx":"Ixx","ixy":"Ixy","ixz":"Ixz","iyy":"Iyy","iyz":"Iyz","izz":"Izz",}
defined_names["inertial"]={"mass":"Mass"}
defined_names["joint"]={"name":["Label","Label2"]}
# {name:str,{parents:list,alias,default value, type}}
# algorithm
# i. add parameters to object
# ii. update ui and display it
#
# initialization will take place in the link_proxy , the ui file will be a class  parameter
# that will be accessed by the edit command and populated in the link_proxy
# edit Parameters will only display the dock widget
class properties_base(QDockWidget):
    def __init__(self,parent):
        super().__init__(parent)
    
    def initializeButtons(self,layout:QHBoxLayout|QVBoxLayout):
        self.buttons = QDialogButtonBox()
        self.buttons.setObjectName("buttonbox")
        self.buttons.setStandardButtons(
            QDialogButtonBox.Ok | QDialogButtonBox.RestoreDefaults
        )
        
        self.buttonsLayout = QHBoxLayout()
        self.buttonsLayout.addItem(
            QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
        )
        self.buttonsLayout.addWidget(self.buttons)
        layout.addLayout(self.buttonsLayout)
        
    def connectButtonSignals(self):
        """Connect button signals to their respective slots"""
        self.buttons.accepted.connect(self.onOkClicked)  # OK button
        self.buttons.rejected.connect(self.onRestoreDefaults)  # Note: RestoreDefaults is actually a 'reject' role
        
    def onOkClicked(self):
        """Handler for OK button click - closes the dock widget"""
        self.close()
        self.__class__.active=False
        
    def onRestoreDefaults(self):
        """Placeholder for RestoreDefaults functionality"""
        print("Restore Defaults clicked")  # Add your restore logic here
        # all properties in properties class variable will be called , and the alias will have its
        # and default values will be used to reset values using setattr 
        # the dockwidget will be closed
        # link_properties will be called with property_only set to false
        # the link_properties dockwidget will then be displayed again using ...
        #  main_window.addDockWidget(PySide2.QtCore.Qt.RightDockWidgetArea,dockwidget)
        
        
    def closeEvent(self, event):
        """Override of closeEvent to handle custom close behavior"""
        # You can add custom close handling here if needed
        # For example:
        # print("DockWidget is closing")
        # if not self.shouldClose():
        #     event.ignore()
        #     return
        reserved_names.clear()
        self.__class__.active=False
        super().closeEvent(event)  # Call parent class implementation
        
    def sizeHint(self):
        return QSize(732, 876)
    
class link_properties(properties_base):
    properties:list=[]#{name:str,{parents:list,alias,default value, type}}
    active=False
    def __init__(self,obj,type,property_only,parent=None):
        if self.__class__.active is True:
            return
        super().__init__(parent)
        self.setObjectName("linkEditor")
        self.obj=obj
        self.type=type
        # Main container widget and layout
        self.container = QWidget()
        self.contents = QVBoxLayout(self.container)
        self.contents.setContentsMargins(5, 5, 5, 5)
        self.contents.setSpacing(5)
        self.setWidget(self.container)
        
        self.initializeTabs()
        super().initializeButtons(layout=self.contents)
        
        # Connect OK button signal
        super().connectButtonSignals()
        self.prop_N_uiSetup=initialize(self,self.obj,self.type,property_only)
    def initializeTabs(self):
        self.tab = QTabWidget()
        self.tab.setObjectName("linkTabs")
        self.tab.setTabsClosable(False)
        self.tab.setMovable(False)
        self.tab.setTabBarAutoHide(False)
        
        self.link_tab = QWidget()
        self.link_tab.setObjectName("link")
        self.tab.addTab(self.link_tab, "Link")
        
        self.visual_tab = QWidget()
        self.visual_tab.setObjectName("visual")
        self.tab.addTab(self.visual_tab, "Visual")
        
        self.collision_tab = QWidget()
        self.collision_tab.setObjectName("collision")
        self.tab.addTab(self.collision_tab, "Collision")
        
        self.tab.setCurrentIndex(0)
        self.contents.addWidget(self.tab)
        
    
    
        
class joint_properties(properties_base):
    properties:list=[]
    active=False
    def __init__(self,obj,type,property_only,parent=None):
        if self.__class__.active is True:
            return
        super().__init__(parent)
        self.obj=obj
        self.type=type
        self.jointWidget=QWidget()
        self.container = QWidget()
        self.contents = QVBoxLayout(self.container)
        self.contents.setContentsMargins(5, 5, 5, 5)
        self.contents.setSpacing(5)
        self.setWidget(self.container)
        self.contents.addWidget(self.jointWidget)
        super().initializeButtons(layout=self.contents)
        super().connectButtonSignals()
        self.prop_N_uiSetup=initialize(self,self.obj,self.type,property_only)
    
    

class initialize:
    def __init__(self,parent, object, type,property_only=True):
        # type can either be link or joint
        self.type = type  # either a link or a joint
        self.obj = object
        self.property_only=property_only
        self.parent=parent
        self.UI()
        
    def generate_ui(self,tab,root_file:str,include_files:list):
        '''
        Args:
            tab(ui): the tab to add widgets to
            root_file(str):first file that all others are children of 
            include_files(list): strictly a list  children of the root file 
        Returns:
            None
            
        '''
        structure: dict = sdf_schema_parser.sdf_schema_parser(
            file=root_file, recurse=False, includeMetaData=False
        ).data_structure
        top = CollapsibleSection(structure["tag"])
        self.add_dynamic_widgets(top, structure, structure["children"])
        # now loop through all other files and add them
        for f in include_files:
            s: dict = sdf_schema_parser.sdf_schema_parser(
            file=f, recurse=False, includeMetaData=False
                ).data_structure
            self.add_dynamic_widgets(top,s,s["children"])
        
        # set layout has to be used or else Qt might ignore it
        # 2. Create a scroll area and set its widget
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)  # Important for proper resizing
        container = QWidget()
        container.setLayout(QVBoxLayout())  # Or QHBoxLayout if needed
        container.layout().addWidget(top)
            
        scroll.setWidget(container)
        if tab.layout():
            QWidget().setLayout(tab.layout())  # Clear old layout
        tab.setLayout(QVBoxLayout())
        tab.layout().addWidget(scroll)
        
    def UI(self):
        if self.type == "link":
            link_tab=self.parent.link_tab
            self.generate_ui(link_tab,"link.sdf",["inertial.sdf"])
            # Todo
            # code related to collision 
            collision_tab=self.parent.collision_tab
            self.generate_ui(collision_tab,"collision.sdf",["surface.sdf"])
            visual_tab=self.parent.visual_tab
            self.generate_ui(visual_tab,"visual.sdf",[])
            #  avoid similar names on the same object only since reserved names is global clear it
            # so that the next object will have an empty list 
            reserved_names.clear()
            # visual_tab=self.linkd.__class__.ui.visual
        elif self.type == "joint":
            
            jointwidget=self.parent.jointWidget
            self.generate_ui(jointwidget,"joint.sdf",[])
            reserved_names.clear()
        else:
            pass

    def property_n_widget_setup(
        self,
        widget: CollapsibleSection,
        name: str,
        alias: str,
        default,
        data_type: str,
        parent_tag: str = None,
        min:float|int=0.0 , max:float|int=1000.0,decimals:int=3,
        defined:bool=False
    ):
        if parent_tag is None:
            parent_tag = "root"
        if data_type == "string":
            # pick from here April 9 ,2025 14:30
            # Todo
            #   1.add property to object
            if self.property_only:
                if defined is False:
                    if isinstance(alias,list):
                        self.obj.addProperty(
                    "App::PropertyString", alias[0], parent_tag, hidden=True
                )  # Todo
                    else:
                        if  hasattr(self.obj,alias) is False:
                            self.obj.addProperty(
                        "App::PropertyString", alias, parent_tag, hidden=True
                            )  # Todo
            else:
                widget.add_widget(
                    self.create_labeled_lineedit(name, alias, default_text=default)
                )
        elif data_type == "bool":
            if self.property_only:   
                if defined is False and hasattr(self.obj,alias) is False:
                    self.obj.addProperty(
                    "App::PropertyBool", alias, parent_tag, hidden=True
                    )  # Todo
                    setattr(self.obj,alias,True if default == "true" else False)
            else:
                widget.add_widget(self.create_labeled_checkbox(
                    name, alias, default_state=True if default == "true" else False
                ))
        elif data_type == "double":
            if self.property_only:
                if defined is False and hasattr(self.obj,alias) is False: 
                    self.obj.addProperty     (
                    "App::PropertyFloat", alias, parent_tag, hidden=True
                    )  # Todo
                    setattr(self.obj,alias,float(default))
            # Todo
            #  1.  how to determin min and max value
            #     options
            #       i. read current default value and set max to twice the value
            #           issue arises for default values of  0
            else:
                widget.add_widget(
                    self.create_labeled_double_spinbox(name, alias, default_val=default)
                )
        elif data_type == "unsigned int":
            if self.property_only:
                if defined is False and hasattr(self.obj,alias) is False:
                    self.obj.addProperty(
                    "App::PropertyInteger", alias, parent_tag, hidden=True
                )  # Todo
                    setattr(self.obj,alias,int(default))
            else:
                widget.add_widget(
                    self.create_labeled_spinbox(name, alias, default_val=default)
                )
        elif data_type == "vector3":
            default_list = extract_vector_n(default)
            if self.property_only:   
                if defined is False and hasattr(self.obj,alias) is False:
                    self.obj.addProperty("App::PropertyVector", alias, parent_tag, hidden=True)
                    setattr(self.obj,alias,tuple(default_list))
            else:
                
                widget.add_widget(
                    self.create_vector3_group(
                        name,
                        alias,
                        default_values=[default_list[0], default_list[1], default_list[2]],
                    )
                )
        elif data_type == "pose":
            pass
        elif data_type=="color":
            pass  
        # Todo
        else:
            pass

    def add_dynamic_widgets(
        self,
        widget: CollapsibleSection,
        dc: dict,
        children: list,
        parent_tag: str | None = None,
    ):
        """
        parameters :
            widget(QWidget): top most widget
            dc: a dictionary has  hierachial info about sdf file
            children: a list of dictionaries that are similar to dc
            parent_tag:str
                name of  the parent
        returns:
            qwidget or None
        """
        # check all attribute and add a cresponding widget based
        # on the data type
        tag: str = dc["tag"]
        defined:bool=False

        if dc["attributes"] is not None:
            for attribute in dc["attributes"]:
                defined:bool=False
                # attributes are of type Element_Attributes see sdf_schema_parser
                name, default, data_type = attribute.get_complete().values()
                # check if the tag anme is part of the ones whose properties have already defined in FreeCAD
                # if its available  check if the name is already defined see defined_names
                # might be remove in fuature after synchronization 
                if tag in defined_names and name in defined_names[tag].keys():
                    # take alias as the property name in FreeCAD
                    alias=defined_names[tag][name]
                    # if isinstance(alias,list):
                    #     alias=alias[0]
                    defined=True
                    # if the name is not available generate its alias 
                else:
                    alias = generate_parameter_name(name, parent_names=parent_tag,reserved_names=reserved_names)
                    # dont add properties more than once
                    
                if self.property_only :
                    prop_data = {
                    "name": name,
                    "alias": alias[0] if isinstance(alias, list) else alias,
                    "parent": tag,
                    "default": default,
                        }
                    self.parent.__class__.properties.append(prop_data)
                    
                self.property_n_widget_setup(
                    widget, name, alias, default, data_type, parent_tag=parent_tag,defined=defined
                )
        
        if dc["value"] is not None:
            defined=False
            alias = None
            if parent_tag is not None:
                if parent_tag in defined_names and tag in defined_names[parent_tag].keys():# and and isinstance(defined_names[parent_tag], dict)
                    alias=defined_names[parent_tag][tag]
                    # if isinstance(alias,list):
                    #     alias=alias[0]
                    defined=True
                else:  
                    alias = generate_parameter_name(tag, parent_names=parent_tag,reserved_names=reserved_names)
        
            else:
                # probably unreachable
                print("reached\n")
                if tag in defined_names and tag in defined_names[tag].keys():
                    alias=defined_names[tag][name]
                    # if isinstance(alias,list):
                    #     alias=alias[0]
                    defined=True
                # reachable
                alias = generate_parameter_name(tag,reserved_names=reserved_names)
            # append list only once during initialization 
            if self.property_only:
                prop_alias = alias[0] if isinstance(alias, list) else alias
                self.parent.__class__.properties.append(
                    {
                "name": tag,
                "alias": prop_alias,
                "parent": parent_tag,
                "default": dc["value"],
              }
                )
            self.property_n_widget_setup(
                widget,
                tag,
                alias,
                default=dc["value"],
                data_type=dc["data_type"],
                parent_tag=parent_tag,defined=defined
            )
        # loop through allchildren , and perform neccesary steps
        # recursion might be requiered
        if len(children) > 0:
            # create a new container only for nested items not the root item which does not 
            # have a parent 
            # Determine the container widget (CollapsibleSection if parent exists)
            container = CollapsibleSection(tag) if parent_tag is not None else widget
            if parent_tag is not None:
                widget.add_widget(container)
            # loop through each child recursively
            for child in children:
                if child["tag"] not in ignore_list:
                    self.add_dynamic_widgets(container, child, child["children"], tag)

    def create_labeled_double_spinbox(
        self,
        label_text,
        alias,
        parent=None,
        min_val=0.0,
        max_val=100.0,
        default_val=0.0,
        step=1.0,
        decimals=2,
    ):
        """
            Creates a horizontal layout with a label and double spinbox.

            Args:
            label_text (str): Text for the label
            alias: unique name of the components(generated)
            parent (QWidget): Parent widget (optional)
            min_val (float): Minimum value for spinbox
            max_val (float): Maximum value for spinbox
            default_val (float): Default/initial value
            step (float): Step increment
            decimals (int): Number of decimal places

        Returns:
            QWidget: The container widget with horizontal layout
            QDoubleSpinBox: The created spinbox for signal connections
        """
        # Create container widget
        container = QWidget(parent)
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)

        # Create label
        label = QLabel(label_text)
        label.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Preferred)
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        # Create spinbox
        spinbox = QDoubleSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setValue(getattr(self.obj,alias))
        spinbox.setSingleStep(step)
        spinbox.setDecimals(decimals)
        spinbox.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        # val is emitted by the valueChanged signal
        # dont worry about capturing obj it wont change during the life time of this widget
        spinbox.valueChanged.connect(lambda val, obj=self.obj: setattr(obj, alias, val))
        # Add widgets to layout
        layout.addWidget(label)
        layout.addWidget(spinbox)
        return container

    # labeled line edit
    def create_labeled_lineedit(
        self,
        label_text,
        alias,
        parent=None,
        default_text="",
        label_width=None,
        lineedit_width=None,
        placeholder="",
    ):
        """
        Creates a labeled QLineEdit in a horizontal layout.

        Args:
            label_text (str): Text for the label
            alias: unique name of the components(generated)
            parent (QWidget): Parent widget
            default_text (str): Initial text content
            label_width (int): Optional fixed width for the label
            lineedit_width (int): Optional width for the line edit
            placeholder (str): Placeholder text
            callback (function): Optional function to call when text changes

        Returns:
            QWidget: The container widget with horizontal layout
            QLineEdit: The line edit widget for direct access
        """
        # Create container widget
        container = QWidget(parent)
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        label = QLabel(label_text)
        label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        if label_width:
            label.setFixedWidth(label_width)

        # Create line edit
        line_edit = QLineEdit()
        if isinstance(alias,list):
            line_edit.setText(getattr(self.obj,alias[0]))
        else:
            line_edit.setText(getattr(self.obj,alias))
        line_edit.setPlaceholderText(placeholder)
        line_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        # only a line edit has 2 names defined for asingle item e.g link had Label1 and Label2
        if isinstance(alias,list):
            line_edit.textChanged.connect(
            lambda txt, obj=self.obj: (setattr(obj, alias[0], txt), setattr(obj,alias[1],txt)))
        else:
             line_edit.textChanged.connect(
            lambda txt, obj=self.obj: setattr(obj, alias, txt))
        if lineedit_width:
            line_edit.setFixedWidth(lineedit_width)

        layout.addWidget(label)
        layout.addWidget(line_edit)

        return container

    def create_labeled_spinbox(
        self,
        label_text,
        alias,
        parent=None,
        min_val=0,
        max_val=100,
        default_val=0,
        step=1,
        label_width=None,
        spinbox_width=None,
        callback=None,
    ):
        """
        Creates a labeled spinbox in a horizontal layout.

        Args:
            label_text (str): Text for the label
            alias: unique name of the components(generated)
            parent (QWidget): Parent widget
            min_val (int): Minimum value
            max_val (int): Maximum value
            default_val (int): Default value
            step (int): Step increment
            label_width (int): Optional fixed width for label
            spinbox_width (int): Optional width for spinbox
            callback (function): Optional function to call when value changes

        Returns:
            QWidget: The container widget with horizontal layout
            QSpinBox: The spinbox widget for direct access
        """
        container = QWidget(parent)
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # Label
        label = QLabel(label_text)
        label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        if label_width:
            label.setFixedWidth(label_width)

        # SpinBox
        spinbox = QSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setValue(getattr(self.obj,alias))
        spinbox.setSingleStep(step)
        spinbox.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        if spinbox_width:
            spinbox.setFixedWidth(spinbox_width)
        spinbox.valueChanged.connect(lambda val, obj=self.obj: setattr(obj, alias, val))
        layout.addWidget(label)
        layout.addWidget(spinbox)
        return container

    # checkbox
    def create_labeled_checkbox(
        self,
        label_text,
        alias,
        parent=None,
        default_state=False,
        label_width=None,
        callback=None,
    ):
        """
        Creates a labeled checkbox in a horizontal layout.

        Args:
            label_text (str): Text for the label
            alias: unique name of the components(generated)
            parent (QWidget): Parent widget
            default_state (bool): Initial checked state
            label_width (int): Optional fixed width for the label
            callback (function): Optional function to call when state changes

        Returns:
            QWidget: The container widget with horizontal layout
            QCheckBox: The checkbox widget for direct access
        """
        container = QWidget(parent)
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        label = QLabel(label_text)
        label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        if label_width:
            label.setFixedWidth(label_width)

        # Checkbox
        checkbox = QCheckBox()
        checkbox.setChecked(getattr(self.obj,alias))
        checkbox.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        # callback
        checkbox.stateChanged.connect(
            lambda state, obj=self.obj: setattr(obj, alias, state)
        )
        layout.addWidget(label)
        layout.addWidget(checkbox)
        return container

    # vector3
    def create_vector3_group(
        self,
        title,
        alias,
        parent=None,
        default_values=(0.0, 0.0, 0.0),
        min_val=-9999.0,
        max_val=9999.0,
        decimals=3,
        step=0.1,
    ):
        """
        Creates a Vector3 control group with X, Y, Z components.

        Args:
            title (str): Groupbox title
            parent (QWidget): Parent widget
            default_values (tuple): Default (x, y, z) values
            min_val (float): Minimum value for all components
            max_val (float): Maximum value for all components
            decimals (int): Number of decimal places
            step (float): Step increment

        Returns:
            QGroupBox: The group container
            tuple: The three spinbox widgets (x_spinbox, y_spinbox, z_spinbox)
            pyqtSignal: Signal emitted when any value changes (emits x, y, z)
        """
        # Create group box
        group = QGroupBox(title, parent)
        group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.MinimumExpanding)

        # Main layout
        layout = QVBoxLayout(group)
        layout.setContentsMargins(5, 15, 5, 5)
        layout.setSpacing(5)

        # Create a signal to emit vector changes

        # Row for X component
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel("X:"))
        x_spinbox = QDoubleSpinBox()
        x_spinbox.setRange(min_val, max_val)
        x_spinbox.setValue(getattr(self.obj,alias)[0])
        x_spinbox.setDecimals(decimals)
        x_spinbox.setSingleStep(step)
        x_layout.addWidget(x_spinbox)
        x_layout.addStretch()

        # Row for Y component
        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel("Y:"))
        y_spinbox = QDoubleSpinBox()
        y_spinbox.setRange(min_val, max_val)
        y_spinbox.setValue(getattr(self.obj,alias)[1])
        y_spinbox.setDecimals(decimals)
        y_spinbox.setSingleStep(step)
        y_layout.addWidget(y_spinbox)
        y_layout.addStretch()

        # Row for Z component
        z_layout = QHBoxLayout()
        z_layout.addWidget(QLabel("Z:"))
        z_spinbox = QDoubleSpinBox()
        z_spinbox.setRange(min_val, max_val)
        z_spinbox.setValue(getattr(self.obj,alias)[2])
        z_spinbox.setDecimals(decimals)
        z_spinbox.setSingleStep(step)
        z_layout.addWidget(z_spinbox)
        z_layout.addStretch()

        # Add rows to main layout
        layout.addLayout(x_layout)
        layout.addLayout(y_layout)
        layout.addLayout(z_layout)

        # callbacks
        x_spinbox.valueChanged.connect(
            lambda val, obj=self.obj: setattr(
                obj, alias, (val, y_spinbox.value(), z_spinbox.value())
            )
        )
        y_spinbox.valueChanged.connect(
            lambda val, obj=self.obj: setattr(
                obj, alias, (x_spinbox.value(), val, z_spinbox.value())
            )
        )
        z_spinbox.valueChanged.connect(
            lambda val, obj=self.obj: setattr(
                obj, alias, (x_spinbox.value(), y_spinbox.value(), val)
            )
        )
        group.adjustSize()
        group.setMinimumSize(200, 120)

        return group


