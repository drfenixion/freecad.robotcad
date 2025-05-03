from .sdf_parser import sdf_tree
import xml.etree.ElementTree as ET
from typing import List, Optional
from . import setup
import copy,os
from typing import List, TypedDict
from .. import urdf_utils
# a list of elements must have children that will be includeded whether selected or not 
override_list=["inertia","axis","limit",]

class elements:
    """A class to manage and cache common SDF link elements with cleanup functionality."""
    
    # Class-level element caches
    link: Optional[ET.Element] = None
    surface: Optional[ET.Element] = None
    collision: Optional[ET.Element] = None
    inertial: Optional[ET.Element] = None
    material: Optional[ET.Element] = None
    visual: Optional[ET.Element] = None
    joint:Optional[ET.Element]=None
    def __init__(self):
        """Initialize the link elements by loading and caching the required SDF elements."""
        self._initialize_elements()
        
    def _initialize_elements(self) -> None:
        """Initialize and cache all required elements with their respective cleanups."""
        element_configs = [
            ('link', "link.sdf", None),
            ('surface', "surface.sdf", None),
            ('collision', "collision.sdf", None),  # Fixed typo in filename
            ('inertial', "inertial.sdf", ["pose"]),
            ('material', "material.sdf", ["script", "shader", "render_order",
                                        "double_sided", "pbr", "lighting"]),
            ('visual', "visual.sdf", ["meta", "visibility_flags"]),
             ("joint","joint.sdf",None)
        ]
        
        for element_name, filename, remove_list in element_configs:
            if getattr(self.__class__, element_name) is None:
                element = self.get_element(filename)
                if remove_list:
                    self.clean_element(element, remove_list)
                setattr(self.__class__, element_name, element)
    
    def clean_element(self, element: ET.Element, remove_list: List[str]) -> None:
        """
        Recursively remove elements whose tag names are in the remove_list.
        
        Args:
            element: The root element to clean
            remove_list: List of element tag names to remove
        """
        to_remove:List[ET.Element]=[]
        if element is None:
            return
        for child in element:
            if len(child)>0:
                if child.tag in remove_list:
                    # prefer using a to_remove instead of element.remove(child)
                    # the later causes issues 
                    to_remove.append(child) 
                else:
                    self.clean_element(child,remove_list)
            elif child.tag in remove_list:
                to_remove.append(child)
            else:
                pass
        for e in to_remove:
            element.remove(e)
            


         
    def get_element(self, file: str) -> ET.Element:
        """
        Get an SDF element from a file.
        
        Args:
            file: The SDF file to load
            
        Returns:
            The root element of the SDF tree
        """
        return sdf_tree.sdf_tree(file, metaData=False, recurse=False, minimal=True).get_element
Elements=elements()

class element_property_map(TypedDict):
    name:str
    alias:str
    parent:str
    default:str
    
def get_alias(dlist:List[element_property_map],name:str,parent:str)->str:
    """
    get the alias name of xml properties 
    used to retrieve info from  FreeCAD 

    Args:
        dlist (List[element_property_map]):a list of dictionaries from the properties class variable
        {name,alias,parent,default}
        name (str): tag name of element
        parent (str): parent tag name

    Returns:
        str: alias name to be used to retrieve the propety
    """
    for item in dlist:
        if item["name"]==name and item["parent"]==parent:
            return item["alias"]

def update(obj,el:ET.Element,selected_list:list,property_struct:List[element_property_map])->ET.Element|None:
    """
    update element  text and attributes of elements based on values  in FreeCAD Properties 
    remove unselected elements 

    Args:
        obj (_type_): document object i.e {link,joint,...}
        el (ET.Element):parent xml element 
        selected_list (list): list of elements to be included see selected list in FreeCAD properties 
        property_struct (_type_): object properties and  and there aliases
    Returns:
        ET.Element: updated element
    """
    elements_to_remove:List[ET.Element]=[]
    attributes_to_remove:List[List[ET.Element,str]]=[]
    for element in el:
        # check if element has children 
        if len(element)>0:
             # some elements such as inertial will be included whether they are selected or not 
            if element.tag in selected_list or element.tag in override_list:
                    update(obj,element,selected_list,property_struct)
            else:
                # remove element
                elements_to_remove.append(element)
                # el.remove(element) <-this causes issues i.e some elements are skipped
        else:
            for name,*_ in element.attrib.items():
                # use the tag name here see setup.py  add_dynamic widgets
                # for attributes the name is the attribute name and the parent is the tag
                id:str=get_alias(property_struct,name,element.tag)
                # special treatment for quantities and ...
                # type conversions are reqired
                if id is not None:
                    
                    value=sanitize_for_sdf(getattr(obj,id))
                    element.set(element.tag,value)
                else:
                    # prevent changing of size during iteration 
                    attributes_to_remove.append([element,name])
            if element.text is not None:
                # for text name is the tag and the parent is the parent tag 
                id:str=get_alias(property_struct,element.tag,el.tag)
                # undefined items will return None
                if id is not None:
                    if id.lower() in ["mass"]:
                        value=sanitize_for_sdf(getattr(obj,id).getValueAs("kg"))
                    else:
                        value=sanitize_for_sdf(getattr(obj,id))
                    element.text=value
                else:
                    elements_to_remove.append(element)
    for elem,attr in attributes_to_remove:
        del elem.attrib[attr]
    for e in elements_to_remove:
        el.remove(e)
    # return el

def sanitize_for_sdf(data:str|list|tuple|bool)->str:
    """
    convert  data types to sdf compatible string  e.g True->'true', False->'false'

    Args:
        data (str | list | tuple | bool):item to be converted

    Returns:
        str: sdf compatible string 
    """
    # check if its a type FreeCAD Vector by checking for the x attribute 
    if hasattr(data,'x'):
        data=list(data)
    if isinstance(data, list) | isinstance(data,tuple):
            # equivalent string
        return " ".join(map(str, data))
    elif isinstance(data,bool):
        return 'true' if data is True else 'false'
    else:
        return str(data)


class ref_data:
    root_link:str|None=None
    visual_container:list=[]
    """
    # description
    visual container will contain a pointer or reference  to a 
    ET.Element item for the most recent link visual item  processed
    # why create this?
    create_visual_sdf requires  cross::link object so as to access user defined
    sdf parameters while  sdf_visual object required a partobject to access physical properties 
    of a link such placement , material info i.e colors
    each need to append to the same element
    so to ensure that both functions write to the correct Element list visual_container stores the most 
    recently processed link element by create_visual_sdf ,sdf_visual object will pop the  element and append 
    geometry and pose info ,tranparency ,and colors 
    
    # Assumption:
    all links are proccesed linearly i.e one at a time with no multithreading so that the element stored in ...
    visual_container  will the correct element to which pose geometry and color data is written
     with multi threading a way to track items will be required 
    # pitfalls 
    
    # TODO
     some code refactors later to avoid this 
    
    """
    collison_container:list=[]
    """
        # description
        see visual_container
    """  
    # {link_name,placement}
    placement_info:dict={}
    
    
# sdf elements 
def create_sdf_element(obj,name:str,type:str):
    if type=="link":
        element=copy.deepcopy(Elements.__class__.link)
    elif type=="visual":
        element=copy.deepcopy(Elements.__class__.visual)
    elif type=="collision":
        element=copy.deepcopy(Elements.__class__.collision)
        surface=copy.deepcopy(Elements.__class__.surface)
        element.append(surface)
    elif type=="inertial":
        element=copy.deepcopy(Elements.__class__.inertial)
        element.attrib.pop('auto', None)
    elif type=="joint":
        element=copy.deepcopy(Elements.__class__.joint)
    else:
        pass
    if type in ["link","visual","collision","inertial"]:
        property_struct=setup.link_properties(data_only=True).__class__.properties
    elif type=="joint":
        property_struct=setup.joint_properties(data_only=True).__class__.properties
    # set name
    if type!="inertial":
        element.attrib["name"]=name
    selected_list=getattr(obj,"selected")

    update(obj,element,selected_list,property_struct)
    if type in ["link",]:
        if ref_data.root_link is not None:
            placement=obj.Placement
            # append link-name and placement
            ref_data.placement_info[obj.Label2]=obj.Placement
            if obj.Label2==ref_data.root_link:
                element.append(urdf_utils.urdf_origin_from_placement(placement,format="sdf"))
            else:
                placement_xml=urdf_utils.urdf_origin_from_placement(placement,format="sdf")
                # placement_xml.set("relative_to",f'{ref_data.root_link}')
                element.append(placement_xml)
    if type=="collision":
        #NOTE
        # from link_proxy.py->export_urdf link and visual are processed before collisions
        # so a visual container exists in ref_data
        lnk_name=name.replace("collision_",'')
        lnk_placement=ref_data.placement_info[lnk_name]
        # get collision placement relative to link
        plc=lnk_placement.inverse()*obj.Placement
        placement_xml=urdf_utils.urdf_origin_from_placement(plc,format="sdf")
        element.append(placement_xml)
    if type=="visual":
        ref_data.visual_container.append(element)
    elif type=="collision":
        ref_data.collison_container.append(element)
    return element



def object_as_sdf(obj,name:str,placement,type:str,element_type:str)->ET.Element:
    """
    map object properties to sdf 

    Args:
        obj (_type_): object.
        name (str): label name 
        type (str): can be box,spere , cylinder, mesh ...
        element_type(str):"visual" or "collision" 

    Returns:
        ET.Element: object 
    """
    if element_type=="visual":  
    # geometry will be simple to inlude 
    # type will be used to determine whether its a box, cylinder,mesh ...
    # get reference to element
        root_element:ET.Element=ref_data.visual_container.pop()
        tranparency=root_element.find("transparency")
    # the FreeCAD Transparency flag varies from 0-100 ,sdf from 0-1
    # scale 
        viewobj=getattr(obj,"ViewObject",None)
        
        if viewobj is not None:
            set_material_element(viewobj,root_element)
            tranparency.text=getattr(viewobj,"Transparency",0)/100
            
    elif element_type=="collision":
        root_element=ref_data.collison_container.pop()

    
    if type=="box":
        # geometry
        root_element.append(urdf_utils.urdf_geometry_box(obj.Length.getValueAs('m'),
            obj.Width.getValueAs('m'),
            obj.Height.getValueAs('m'),format='sdf'))
          
    elif type=="sphere":
        root_element.append(urdf_utils.urdf_geometry_sphere(obj.Radius.getValueAs('m'),format="sdf"))
    elif type=="cylinder":
        root_element.append(urdf_utils.urdf_geometry_cylinder(obj.Radius.getValueAs('m'),
                                                        obj.Height.getValueAs('m'),format="sdf"))
    else:
        pass 
   
    return root_element

    
def set_material_element(viewobj,parent_element):
    # round color values to 4 decimal places and set last value to 1 if it's zero
    fix_color = lambda c: [round(i, 3) for i in c[:-1]] + [1.0 if c[-1] == 0 else round(c[-1], 4)]
    material=copy.deepcopy(Elements.__class__.material)
    shapeAppearence=getattr(viewobj,"ShapeAppearance")[0]
    # extract and update material data 
    materialProperties={
            "shininess":sanitize_for_sdf(round(shapeAppearence.Shininess,4)),
            "diffuse":sanitize_for_sdf(fix_color(shapeAppearence.DiffuseColor)),
            "specular":sanitize_for_sdf(fix_color(shapeAppearence.SpecularColor)),
            "emissive":sanitize_for_sdf(fix_color(shapeAppearence.EmissiveColor)),
            "ambient":sanitize_for_sdf(fix_color(shapeAppearence.AmbientColor))}
    for tag,text in materialProperties.items():
        e=material.find(tag)
        e.text=text
    parent_element.append(material)
def sdf_mesh(obj,placement,package_name,mesh_name,element_type:str='visual'):
    
    geometry=ET.fromstring("<geometry/>")
    mesh=ET.fromstring("<mesh/>")
    uri=ET.fromstring("<uri/>")
    # uri.text=f'model://{package_name}/meshes/{mesh_name}'
    # uri.text=f'model://{package_name}/meshes/{mesh_name}'
    uri.text=f'../../meshes/{mesh_name}'
    mesh.append(uri)
    geometry.append(mesh)
    # get collision or visual based on calling function 
    if element_type=="visual":
        elem:ET.Element=ref_data.visual_container.pop()
    # collision
    else:
        elem:ET.Element=ref_data.collison_container.pop()
    # # append mesh to geometry 
    elem.append(geometry)
 
    # visual.append(urdf_utils.urdf_origin_from_placement(placement,format="sdf"))
    # add material info 
    if element_type=="visual":
        if obj is not None:
            viewobj=getattr(obj,"ViewObject",None)
            if viewobj is not None:
                set_material_element(viewobj,elem)
    # fill maunally for now dynamic filling later based on available features
    return elem

# joint ignore list 
# "type",
# "xyz",
# "gearbox_ratio",
# "gearbox_reference_body",
# "thread_pitch",
# "screw_thread_pitch",
# # joint limit
# "lower",
# "upper",
# "effort",
# "velocity"
# "axis2",
# "pose",
# "sensor",
# "parent",
# "child"
from ..wb_utils import SDF_VERSION
def make_model_cfg(package_name, description_file_name):
    model = ET.Element("model")
    name = ET.SubElement(model, "name")
    name.text = package_name  # Fixed f-string usage
    ver = ET.SubElement(model, "version")
    ver.text = "1.0"
    sdf = ET.SubElement(model, "sdf", version=SDF_VERSION)  # Fixed attribute and f-string
    author=ET.SubElement(model,"author")
    author.append(ET.fromstring("<name>RoboCAD</name>"))
    sdf.text = description_file_name
    return model

def make_world(package_name:str,include_models:list):
    sdf=ET.fromstring("<sdf/>")
    sdf.attrib["version"]=SDF_VERSION
    world=ET.fromstring("<world/>")
    world.attrib["name"]="robot_world"
    sdf.append(world)
    
    for n in include_models:
        inc=ET.fromstring("<include/>")
        uri=ET.fromstring("<uri/>")
        # TODO
        # have this refer to the package name instead
        uri.text=f'../models/{n}'
        inc.append(uri)
        world.append(inc)
    return sdf
import shutil
def copy_files(src,dest):
    for file in os.listdir(src):
        src_file=os.path.join(src,file)
        dest_file=os.path.join(dest,file)
        if os.path.isfile(src_file):
            shutil.copyfile(src_file,dest_file)