from PySide2.QtCore import QObject,Signal,Property
import FreeCADGui as fcgui
from PySide2.QtGui import QColor
'''
File will  initialize all properties that need to be defined for sdf object e.g world , links and joints
'''

#To do read and  set default values during initialization
def world_parameters(obj,version="1.7"):
    
    #  default values will be added 
    obj.addProperty("App::PropertyString","version","sdf")
    obj.version=version
        # sdf world properties 
    obj.addProperty("App::PropertyVector","gravity","sdf","world gravity vector (m/s^2)",hidden=True)
    obj.addProperty("App::PropertyVector","wind_linear_velocity","sdf",
                             "wind velocity vector (m/s)",hidden=True)
        
    obj.addProperty("App::PropertyVector", "MagneticField", "sdf", 
                             "world magnetic field strength in (Teslas)", hidden=True)
        #  atmosphere properties 
    obj.addProperty("App::PropertyEnumeration", "type", "atmosphere", 
                             "atmosphere engine", hidden=True)
    obj.type=["adiabatic"]
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
    def __init__(self,obj ,parent = ...):
        super().__init__(parent)
        self.obj=obj
        self.initializeProperties()
        
    def initializeProperties(self):
        self.gravity=self.obj.gravity
    
    
    def gravity(self):
        return self.gravity
    
    
    def updateGravity(self,g):
        if g==list(self.gravity):
            return
        self.gravity.x=g[0]
        self.gravity.y=g[1]
        self.gravity.z=g[2]
        