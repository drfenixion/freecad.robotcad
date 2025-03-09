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
        self.colors=self.GetHexColors()
        if self.isDarkTheme():
            # this is base on the pahoehoe palette
            self.background_0:str="#1d1e23" #Raisin black
            self.background_1:str="#33363f" # onyx
            self.background_2:str="#656772" # dim Gray
            self.background_3:str="#57463a" #Umber
            self.textbackground:str="#2e2b26"   #jet
        
        else:
            # this is based on the calor pallete blueSunshine
            self.background_0:str="#c3c5c4" # silver 
            self.background_1:str="#e5ecf2" # Alice Blue
            self.background_2:str="#bad4d3" #light blue
            self.background_3:str="#f5efe3" #old lace
            self.textbackground:str="#f7e091" #flax
            pass
            
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
    
    def getTextBackground(self):
        return self.textbackground

    def getBackground0(self):
        return self.background_0
    def getBackground1(self):
        return self.background_1
    def getBackground2(self):
        return self.background_2
    def getBackground3(self):
        return self.background_3
            
    def getTextColor(self):
        return QColor(self.colors["textcolor"]).name()
    
    textBackground=Property(str, getTextBackground,  notify=textBackgroundChanged)
    textColor = Property(str, getTextColor, notify=textColorChanged)
    background0=Property(str,getBackground0,notify=backgroundChanged)
    background1=Property(str,getBackground1,notify=backgroundChanged)
    background2=Property(str,getBackground2,notify=backgroundChanged)
    background3=Property(str,getBackground3,notify=backgroundChanged)
            
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
    # obj.addProperty("App::PropertyString","name","physics",'''
    #                 name of this set of physics parameters''',hidden=True)
    # obj.addProperty("App::PropertyBool","default","physics","use default world Physics profile",hidden=True)
    obj.addProperty("App::PropertyEnumeration","dynamicsengine","physics",hidden=True)
    obj.dynamicsengine=["ode","bullet","simbody","dart"]
    for p in ["max_step_size","real_time_factor","real_time_update_rate"]:
        obj.addProperty("App::PropertyFloat",p,"physics",
                    '''simulation for simulations in engine''',hidden=True)
    
    obj.addProperty("App::PropertyInteger","max_contacts","physics",'''
                    maximum number of contacts between two entities''',hidden=True)
    
    obj.addProperty("App::PropertyEnumeration","dartSolver_type","Dart",hidden=True)
    obj.dartSolver_type=["pgs","dantzig"]
    obj.addProperty("App::PropertyEnumeration","dartCollision_detector","Dart",hidden=True)
    obj.dartCollision_detector=["dart","fcl","bullet","ode"]
    
    # Simbody
    obj.addProperty("App::PropertyFloat","simbodyMax_transient_velocity","Simbody",'''
                    Roughly the relative error of the system.
                    -LOG(accuracy) is roughly the number of significant digits''',hidden=True)
    for pname in ["simbodyStiffness","simbodyDissipation","simbodyPlastic_coef_restitution","simbodyPlastic_impact_velocity",
                  "simbodyStatic_friction","simbodyDynamic_friction","simbodyViscous_friction","simbodyOverride_impact_capture_velocity",
                  "simbodyOverride_stiction_transition_velocity","simbodyMin_step_size","simbodyAccuracy"]:
        obj.addProperty("App::PropertyFloat",pname,"Simbody",hidden=True)
    
    # bullet
    obj.addProperty("App::PropertyEnumeration","bulletSolver_type","bullet",hidden=True)
    obj.bulletSolver_type=["sequential_impulse"]
    obj.addProperty("App::PropertyFloat","bulletMin_step_size","bullet",hidden=True)
    obj.addProperty("App::PropertyInteger","bulletIters","bullet",hidden=True)
    obj.addProperty("App::PropertyFloat","bulletSor","bullet","set successive relaxation parameters",hidden=True)
    #  bullet Constraints 
    for pname in ["bulletCfm","bulletErp","bulletContact_surface_layer","bulletSplit_impulse_penetration_threshold"]:
        obj.addProperty("App::PropertyFloat",pname,"bullet",hidden=True)
        
    obj.addProperty("App::PropertyBool","bulletSplit_impulse","bullet",hidden=True)
    
    # ode
        # float
    for pname in ["odeMin_step_size","odeSor"]:
        obj.addProperty("App::PropertyFloat",pname,"ode",hidden=True)
        # int
    for pint in ["odeIsland_threads","odeIters","odePrecon_iters"]:
        obj.addProperty("App::PropertyInteger",pint,"ode",hidden=True)
        # str
    for pstr in ["odeFriction_model"]:
        obj.addProperty("App::PropertyString",pstr,"ode",hidden=True)
        # bool
        obj.addProperty("App::PropertyEnumeration","odeType","ode",hidden=True)
        obj.odeType=["world","quick"]
    for pbool in ["odeThread_position_correction","odeUse_dynamic_moi_rescaling"]:
        obj.addProperty("App::PropertyBool",pbool,"ode",hidden=True)
        
        # constraints 
    for pcon in ["odeCfm","odeErp","odeContact_max_correcting_vel","odeContact_surface_layer"]:
        obj.addProperty("App::PropertyFloat",pcon,"ode",hidden=True)



class ObjectPropertyBridge(QObject):
    def __init__(self, obj,parent =None):
        super().__init__(parent)
        self.obj=obj
    
    @Slot(str,result='QVariant')
    def getter(self,prop_name):
        return getattr(self.obj,prop_name)
        
    @Slot(str,'QVariant')
    def setter(self, name,value):
        if getattr(self.obj, name) != value:
            setattr(self.obj, name, value)
    
        
        
              
def resetPhysicsattributes(obj):
    sdf_etree=sdf_tree.sdf_tree("physics.sdf",metaData=False,recurse=False)
    e=sdf_etree.get_element
    for i in ["max_step_size","real_time_factor","real_time_update_rate","max_contacts"]:
        result=get_xml_data(e,i,False)
        setattr(obj,i,result)
    engine=get_xml_data(e,["physics","type"],True)
    obj.dynamicsengine=engine
    # dart parameters 
    el=e.find("dart")
    for drt in ["dartSolver_type","dartCollision_detector"]:
        s=drt.replace("dart","").lower()
        result=get_xml_data(el,s,False)
        setattr(obj,drt,result)
    # simbody 
    el=e.find("simbody")
    for smb in ["simbodyStiffness","simbodyDissipation","simbodyPlastic_coef_restitution","simbodyPlastic_impact_velocity",
                  "simbodyStatic_friction","simbodyDynamic_friction","simbodyViscous_friction","simbodyOverride_impact_capture_velocity",
                  "simbodyOverride_stiction_transition_velocity","simbodyMin_step_size","simbodyAccuracy",
                 "simbodyMax_transient_velocity" ]:
        s=smb.removeprefix("simbody").lower()
        result=get_xml_data(el,s,False)
       
        setattr(obj,smb,result)
    
    # bullet
    el=e.find("bullet")
    for blt in ["bulletMin_step_size","bulletSor"]+["bulletIters"]+\
                ["bulletCfm","bulletErp","bulletContact_surface_layer","bulletSplit_impulse_penetration_threshold"]+\
                    ["bulletSplit_impulse"]:
        s=blt.removeprefix("bullet").lower()
        result=get_xml_data(el,s,False)
        #  convert strings to boolean if neccesary s
        if result=="true":
            result=True
        if result=="false":
            result=False
        setattr(obj,blt,result)
    setattr(obj,"bulletSolver_type",get_xml_data(el,"type",False))
    
    # ode 
    el=e.find("ode")
    for ode in ["odeMin_step_size","odeSor"]+ ["odeIsland_threads","odeIters","odePrecon_iters"]+\
        ["odeType","odeFriction_model"]+["odeThread_position_correction","odeUse_dynamic_moi_rescaling"]+\
            ["odeCfm","odeErp","odeContact_max_correcting_vel","odeContact_surface_layer"]:
        s=ode.removeprefix("ode").lower()
        result=get_xml_data(el,s,False)
        #  convert strings to boolean if neccesary s
        if result=="true":
            result=True
        if result=="false":
            result=False
        setattr(obj,ode,result)
        
def linkParameters(obj):
     
    for i in ["gravity","enable_wind","self_collide","kinematic","must_be_base_link"]:
        obj.addProperty("App::PropertyBool",i,"linkparameters",hidden=True)
    
    for v in ["velocity_decayLinear","velocity_decayAngular"]:
        obj.addProperty("App::PropertyFloat",v,"linkparameters",hidden=True)
        
   
    obj.addProperty("App::PropertyFloat","laser_retro","collision",hidden=True)
    obj.addProperty("App::PropertyString","name","collision",hidden=True)
#surface 
    #bounce
    for c in ["bounceRestitution_coefficient","bounceThreshold"]:
        obj.addProperty("App::PropertyFloat",c,"surface",hidden=True)
    
    #friction
    for f in ["torsionalCoefficient","torsionalPatch_radius",\
              "torsionalSurface_radius","torsionalOdeSlip"]:
        obj.addProperty("App::PropertyFloat",f,"friction",hidden=True)
    obj.addProperty("App::PropertyBool","torsionalUse_patch_radius","friction",hidden=True)
    
    for f in ["odeMu","odeMu2","odeSlip1","odeSlip2"]:
        obj.addProperty("App::PropertyFloat",f,"friction",hidden=True)
    obj.addProperty("App::PropertyVector","odeFdir1","friction",hidden=True)
    
    for f in ["bulletFriction","bulletFriction2","bulletRolling_friction"]:
        obj.addProperty("App::PropertyFloat",f,"friction",hidden=True)
    obj.addProperty("App::PropertyVector","bulletFdir1","friction",hidden=True)
    
    # contact 
    obj.addProperty("App::PropertyBool","contactCollide_without_contact",hidden=True)
    for c in ["contactCollide_without_contact_bitmask","contactCollide_bitmask","contactCategory_bitmask"]:
        obj.addProperty("App::PropertyInteger",c,"contact",hidden=True)
    for d in ["contactPoissons_ratio","contactElastic_modulus"]:
         obj.addProperty("App::PropertyFloat",d,"contact",hidden=True)
    for c in ["odeSoft_cfm","odeSoft_erp","odeKp","odeKd","odeMax_vel","odeMin_depth"]:
        obj.addProperty("App::PropertyFloat",c,"contact",hidden=True)
        
    for b in ["bulletSoft_cfm","bulletSoft_erp","bulletKp","bulletKd","bulletSplit_impulse_penetration_threshold"]:
        obj.addProperty("App::PropertyFloat",b,"contact",hidden=True)
        
    obj.addProperty("App::PropertyBool","bulletSplit_Impulse","contact",hidden=True)
    
    for sc in ["dartBone_attachment","dartStiffness","dartDamping","dartFlesh_mass_fraction"]:
        obj.addProperty("App::PropertyFloat",sc,"soft Contact",hidden=True)

       
def resetLinkParameters(obj):
    sdf_etree=sdf_tree.sdf_tree("link.sdf",metaData=False,recurse=False)
    # main attributes
    e=sdf_etree.get_element 
    for i in ["gravity","enable_wind","self_collide","kinematic","must_be_base_link"]:
        result=True if get_xml_data(e,i,False) else False
        setattr(obj,i,result)
    # velocity decay
    vel_decay=e.find("velocity_decay")
    for v in ["velocity_decayLinear","velocity_decayAngular"]:
        s=v.replace("velocity_decay","").lower()
        r=get_xml_data(vel_decay,s,False)
        setattr(obj,v,r)
    # collision 
    sdf_etree=sdf_tree.sdf_tree("collision.sdf",metaData=False,recurse=False)
    # main attributes
    e=sdf_etree.get_element   
    r=get_xml_data(e,["collision","name"],True)
    
    setattr(obj,"name",r)
    r2=get_xml_data(e,"laser_retro",False)
    setattr(obj,"laser_retro",r2)
    
    # surface 
    sdf_etree=sdf_tree.sdf_tree("surface.sdf",metaData=False,recurse=False)
    # main attributes
    e=sdf_etree.get_element  
    for b in ["bounceRestitution_coefficient","bounceThreshold"]:
        b2=b.replace("bounce","").lower()
        bnc=e.find("bounce")
        r=get_xml_data(bnc,b2,False)
        setattr(obj,b,r)
    # collision friction 
    fr=e.find("friction")
    for t in ["torsionalCoefficient","torsionalUse_patch_radius","torsionalPatch_radius",\
              "torsionalSurface_radius","torsionalOdeSlip"]:
        t2=t.replace("torsional","").replace("Ode","").lower()
        trs=fr.find("torsional")
        r=get_xml_data(trs,t2,False)
        if r=='true':
            r=True
        elif r=='false':
            r=False
        setattr(obj,t,r)
        
    for o in ["odeMu","odeMu2","odeFdir1","odeSlip1","odeSlip2"]:
        o2=o.replace("ode","").lower()
        ode=fr.find("ode")
        r=get_xml_data(ode,o2,False)
        if isinstance(r,list):
            r=tuple(r)
        setattr(obj,o,r)
    
    
    for f in ["bulletFriction","bulletFriction2","bulletRolling_friction"]:
        f2=f.replace("bullet","").lower()
        ode=fr.find("bullet")
        r=get_xml_data(ode,f2,False)
        if isinstance(r,list):
            r=tuple(r)
        setattr(obj,f,r)
        
    cnt=e.find("contact")
    for c in ["contactCollide_without_contact_bitmask","contactCollide_bitmask","contactCategory_bitmask"]+\
        ["contactPoissons_ratio","contactElastic_modulus"]:
        c2=c.replace("contact","",1).lower()
        r=get_xml_data(cnt,c2,False)
        setattr(obj,c,r)
        
    for c in ["odeSoft_cfm","odeSoft_erp","odeKp","odeKd","odeMax_vel","odeMin_depth"]:
        c2=c.replace("ode","").lower()
        ode=cnt.find("ode")
        r=get_xml_data(ode,c2,False)
        setattr(obj,c,r)
        
    for b in ["bulletSoft_cfm","bulletSoft_erp","bulletKp","bulletKd",\
              "bulletSplit_impulse_penetration_threshold","bulletSplit_Impulse"]:
        b2=b.replace("bullet","").lower()
        bullet=cnt.find("bullet")
        r=get_xml_data(bullet,b2,False)
        if r=="true":
            r=True
        elif r=='false':
            r=False
        setattr(obj,b,r)
    
    for sc in ["dartBone_attachment","dartStiffness","dartDamping","dartFlesh_mass_fraction"]:
        sc2=sc.replace("dart","").lower()
        softc=e.find("soft_contact")
        r=get_xml_data(softc,sc2,False)
        setattr(obj,sc,r)