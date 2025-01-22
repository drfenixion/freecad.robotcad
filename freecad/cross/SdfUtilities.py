from PySide2.QtCore import QObject,Signal,Property,Slot
import FreeCADGui as fcgui
from PySide2.QtGui import QColor
import  xml.etree.ElementTree as ET
from typing import Union

from .sdf.sdf_parser import sdf_tree
import re 
'''
File will  initialize all properties that need to be defined for sdf object e.g world , links and joints
'''
re_pattern=re.compile(
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
    #original
    #pattern = r'(?<![\d.-])\-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?(?:\s+\-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)+(?![\d.-])'
    #update
    return bool(re_pattern.match(s))
def extract_vector_n(input_string):
    '''this extracts a vector from a string of  numbers '''
    #input_string.strip().split(' ') could aslo work here 
    #well lets go with some regex 
    #because this might also be used  for comma separated values 
    
    numbers=re.findall(r'-?(?:\d+\.\d+|\.\d+|\d+)(?:e-?\d+)?',input_string)

    return [float(num) for num in numbers]

def get_xml_data(element:ET.Element,tag:Union[str,list],Is_Attribute:bool=False)->Union[list,dict,str]:
    '''
    see set_xml_data()
    The functions have  similar parameters
    except for the value parameter which is not included and tag \n
    for attributes a list is used in place of tag with the parent tag at index 0 and attribute name at index 1  \n
    This e.g ['world','name'] will return the name attribute of the world element\n
    ->for none string values the caller  is responsible for converting to appropriate types'''
    def get_value(elem_data):
        if assert_vect(elem_data):
            # equivalent string 
            vect_eq=extract_vector_n(elem_data)
            return vect_eq
        else:
            #try converting to int or float date type 
            try:
                try:
                    return int(elem_data)
                except Exception:
                    return float(elem_data)    
            except Exception:
                return elem_data
    
    if Is_Attribute is not True:
        elem_iter=element.iter(tag)
    else:
        elem_iter=element.iter(tag[0])
    #only a single element exists no need to use a for loop
    try:
        elem=elem_iter.__next__()
    except Exception:
        return None
    if Is_Attribute is False:
        txt=elem.text
        if txt is None:
            txt=''
        return get_value(txt)  
    else:
        # return the  the attribute dictionary 
        try:
            try:
                return   int(elem.attrib[tag[1]])
            except Exception:
                return float(elem.attrib[tag[1]])
        except Exception:
            return   elem.attrib[tag[1]]
        
def set_xml_data(element:ET.Element,tag:str,Is_Attribute:bool,value:Union[dict,float,int,list,str])->ET.Element:
    '''
    tag is the tag name of the element to be edited \n
    Element is
    Is Attribute  can either be true or false , True if th value is an attribute \n
    if Is_Attribute is True value has to be a dictionary with one or more key value pairs \n
    The value parameter will contain the actual value to be updated
    '''
    #get the affected element
    elem_iter=element.iter(tag)
    #no need for a loop
    try:
        elem=elem_iter.__next__()
    except Exception:
        return None
        # ensure no dictionaries are sent for non attributes 
    if Is_Attribute is False and isinstance(value,dict) is False:
        if isinstance(value,list):
            # equivalent string 
            elem.text=' '.join(map(str,value))
        else:
            elem.text=str(value)
    else:
        #add/edit  attributes 
        for key in value.keys():
            elem.set(key,str(value[key]))
    return element
#To do read and  set default values during initialization


def resetWorldParameters(obj):
    sdf_etree=sdf_tree.sdf_tree("world.sdf",metaData=False,recurse=False)
    e=sdf_etree.get_element
    name=get_xml_data(e,["world","name"],True)
    obj.name=name
    obj.gravity=tuple(get_xml_data(e,"gravity",False))
    obj.wind_linear_velocity=tuple(get_xml_data(e,"linear_velocity"))
    obj.MagneticField=tuple(get_xml_data(e,"magnetic_field"))
    sdf_etree=sdf_tree.sdf_tree("atmosphere.sdf",metaData=False,recurse=False)
    atm=sdf_etree.get_element
    obj.temperature=get_xml_data(atm,"temperature")
    obj.pressure=get_xml_data(atm,"pressure")
    obj.temperature_grad=get_xml_data(atm,"temperature_gradient")
    
def world_parameters(obj,version="1.7"):
    
    #  default values will be added 
    obj.addProperty("App::PropertyString","version","sdf")
    obj.addProperty("App::PropertyString","name","sdf","unique name of the world",hidden=True)
    obj.version=version
        # sdf world properties 
    obj.addProperty("App::PropertyVector","gravity","sdf","world gravity vector (m/s^2)",hidden=True)
    obj.addProperty("App::PropertyVector","wind_linear_velocity","sdf",
                             "wind velocity vector (m/s)",hidden=True)
        
    obj.addProperty("App::PropertyVector", "MagneticField", "sdf", 
                             "world magnetic field strength in (Teslas)", hidden=True)
        #  atmosphere properties 
    obj.addProperty("App::PropertyEnumeration", "atm_engine", "atmosphere", 
                             "atmosphere engine", hidden=True)
    obj.atm_engine=["adiabatic"]
    obj.addProperty("App::PropertyFloat", "temperature", "atmosphere", 
                             "temperature at sea level (Kelvins)", hidden=True)
    obj.addProperty("App::PropertyFloat", "pressure", "atmosphere", 
                             "pressure at sea level (pascals)", hidden=True)
    obj.addProperty("App::PropertyFloat", "temperature_grad", "atmosphere", 
                             "rate of change of temperature (kelvins/meter)", hidden=True)
    
    
#theme

    
    

class Palette(QObject):
    textBackgroundChanged = Signal()
    textColorChanged = Signal()
    backgroundChanged=Signal()
    def __init__(self, parent = ...):
        super().__init__(parent)
        self.backgroundColor:str=""
        self.colors=self.GetHexColors()
        if self.isDarkTheme():
            self.backgroundColor=self.lighter()
        else:
            self.backgroundColor=self.darker()
            
    def GetHexColors(self)->dict:
        mw=fcgui.getMainWindow()
        bg=mw.palette().background()
        txt=mw.palette().windowText()
    # bcolor=bg.color().getRgb() # get rgb color tuple
    # textcolor=txt.color().getRgb() # get rgb color tuple
        bcolor=bg.color().name()  # get the hex value of the color
        textcolor=txt.color().name() # get hex value of text color
        return {"background":bcolor,"textcolor":textcolor}
    def isDarkTheme(self):
        return QColor(self.colors["background"]).lightness()<128
    
    def darker(self):
         return QColor(self.colors["background"]).darker().name()
    def lighter(self):
        return QColor(self.colors["background"]).lighter().name()
    def getTextBackground(self):
        return QColor(self.colors["background"]).name()
   
    def setTextBackground(self,bgcolor):
        if self.colors["background"]==bgcolor:
            return
        else:
            self.colors["background"]=bgcolor
    
    def getBackground(self):
        return self.backgroundColor
    def setBackground(self,color):
        if self.backgroundColor==color:
            return
        else:
            self.backgroundColor=color
            
    def getTextColor(self):
        return QColor(self.colors["textcolor"]).name()
    def setTextColor(self,textcolor):
        if self.colors["textcolor"]==textcolor:
            return
        else:
            self.colors["textcolor"]=textcolor
    textBackground=Property(str, getTextBackground, setTextBackground, notify=textBackgroundChanged)
    textColor = Property(str, getTextColor,setTextColor, notify=textColorChanged)
    background=Property(str,getBackground,setBackground,notify=backgroundChanged)
            
#end of theme
    
class WorldProperties(QObject):
    gravityChanged=Signal()
    windVelChanged=Signal()
    MagneticFieldChanged=Signal()
    pressureChanged=Signal()
    temperatureChanged=Signal()
    temperatureGradChanged=Signal()
    worldnameChanged=Signal()
    robotnameChanged=Signal()
    def __init__(self,obj,parent =None):
        super().__init__(parent)
        self.obj=obj
        
        
    
    def updateVector(self,obj,val:float,idx:int):
        if idx==0:
            obj.x=val
        elif idx==1:
            obj.y=val
        elif idx==2:
            obj.z=val
        else:
            return
        return
    
    def getworldName(self):
        return self.obj.name
    def setworldName(self,val):
        if val==self.obj.name:
            return
        else:
            self.obj.name=val
    def getrobotName(self): 
        return self.obj.Label
    def setrobotName(self,val):
        if val==self.obj.Label:
            return
        else:
            self.obj.Label=val
            self.obj.Label2=val
            
    
    def getGravity(self):
        return list(self.obj.gravity)
    @Slot(float,int)
    def setGravity(self,val,idx):
        
        if val==self.obj.gravity[idx]:
            return
        self.updateVector(self.obj.gravity,val,idx)
    
    
    def getwind(self):
        return list(self.obj.wind_linear_velocity)
    @Slot(float,int)
    def setWind(self,val,idx):
        
        if val==self.obj.wind_linear_velocity[int(idx)]:
            return
        self.updateVector(self.obj.wind_linear_velocity,val,idx)
        
    
    def getMagneticField(self):
        return list(self.obj.MagneticField)
    @Slot(float,int)
    def setMagneticField(self,val,idx):
        
        if val==self.obj.MagneticField[idx]:
            return
        self.updateVector(self.obj.MagneticField,val,idx)
    
    
    def getTemperature(self):
        return self.obj.temperature
    def setTemperature(self,val):
        if val==self.obj.temperature:
            return
        else:
            self.obj.temperature=val
    
    
    def getPressure(self):
        return self.obj.pressure
    def setPressure(self,val):
        if val==self.obj.pressure:
            return
        else:
            self.obj.pressure=val
            
    def getTemperatureGrad(self):
        return self.obj.temperature_grad
    def setTemperatureGrad(self,val):
        if val==self.obj.temperature_grad:
            return
        else:
            self.obj.temperature_grad=val
    robotname=Property(str,getrobotName,setrobotName,notify=robotnameChanged)     
    worldname=Property(str,getworldName,setworldName,notify=worldnameChanged)
    gravity=Property("QVariantList",getGravity,notify=gravityChanged)
    temperature=Property("QVariantList",getTemperature,setTemperature,notify=temperatureChanged)
    wind_velocity=Property("QVariantList",getwind,notify=windVelChanged)
    magneticField=Property("QVariantList",getMagneticField,notify=MagneticFieldChanged)
    temperature=Property(float,getTemperature,setTemperature,notify=temperatureChanged)
    pressure=Property(float,getPressure,setPressure,notify=pressureChanged)
    temperatureGrad=Property(float,getTemperatureGrad,setTemperatureGrad,notify=temperatureGradChanged)
    
    
def PhysicsParameters(obj):
    obj.addProperty("App::PropertyString","name","physics",'''
                    name of this set of physics parameters''',hidden=True)
    obj.addProperty("App::PropertyBool","default","physics","use default world Physics profile",hidden=True)
    obj.addProperty("App::PropertyEnumeration","dynamicsEngine","physics",hidden=True)
    obj.dynamicsEngine=["ode","bullet","dart"]
    
    obj.addProperty("App::PropertyFloat","max_step_size","physics",
                    '''simulation for simulations in engine''',hidden=True)
    
    obj.addProperty("App::PropertyFloat","real_time_factor","physics",'''
                    target simlation sppedup factor''', hidden=True)
    obj.addProperty("App::PropertyFloat","real_time_update_rate","physics",
                    '''rate at which to update Physics Engine''',hidden=True)
    
    obj.addProperty("App::PropertyInteger","max_contacts","physics",'''
                    maximum number of contacts between two entities''',hidden=True)
    
    obj.addProperty("App::")
    
    
    
    