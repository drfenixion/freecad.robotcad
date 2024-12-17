import FreeCAD as fc 
from . import wb_utils
from . import sdf

import os
from PySide2.QtWidgets import QDialog, QVBoxLayout
from PySide2.QtQuickWidgets import QQuickWidget
from PySide2.QtCore import QUrl,QSize

class world_dialog(QDialog):
    def __init__(self, parent = ..., f = ...):
        super().__init__()
        self.setWindowTitle("world")
        view =QQuickWidget()
        view.setSource(QUrl(os.path.join(wb_utils.UI_PATH,"qml","main.qml")))
        layout = QVBoxLayout(self)
        layout.addWidget(view)
        self.setLayout(layout)
        
    def sizeHint(self):
        return QSize(700,400)
    
        
    
class sdf_world:
    def __init__(self,Robot):
        self.version="1.7"
        self.robot=Robot
        self.obj=fc.ActiveDocument.addObject("App::FeaturePython", "sdf_world_object")
        Robot.addObject(self.obj)
        self.initialize_properties()
        dia=world_dialog()
        
    def initialize_properties(self):
        self.obj.addProperty("App::PropertyString","version","sdf")
        self.obj.version=self.version
        # sdf world properties 
        self.obj.addProperty("App::PropertyVector","gravity","sdf","world gravity vector (m/s^2)",hidden=True)
        self.obj.addProperty("App::PropertyVector","wind_linear_velocity","sdf",
                             "wind velocity vector (m/s)",hidden=True)
        
        self.obj.addProperty("App::PropertyVector", "MagneticField", "sdf", 
                             "world magnetic field strength in (Teslas)", hidden=True)
        #  atmosphere properties 
        self.obj.addProperty("App::PropertyEnumeration", "type", "atmosphere", 
                             "atmosphere engine", hidden=True)
        self.obj.type=["adiabatic"]
        self.obj.addProperty("App::PropertyFloat", "temperature", "atmosphere", 
                             "temperature at sea level (Kelvins)", hidden=True)
        self.obj.addProperty("App::PropertyFloat", "pressure", "atmosphere", 
                             "pressure at sea level (pascals)", hidden=True)
        self.obj.addProperty("App::PropertyFloat", "temperature_grad", "atmosphere", 
                             "rate of change of temperature (kelvins/meter)", hidden=True)
        
    def initialize_ui(self)->None:
        pass
class ViewProviderSdf:
    def __init__(self,view_obj):
        view_obj.Proxy=self
        
    def getIcon(self):
        return str(wb_utils.ICON_PATH / 'sdf.svg')
    def setEdit(self,obj,mode):
        return False
    def doubleClicked(self,obj):
        gui_doc = obj.Document
        if not gui_doc.getInEdit():
            gui_doc.setEdit(obj.Object.Name)
        dia=world_dialog()
        dia.exec_()
        return True
    def attach(self,obj):
        return
    
    def dumps(self):
        return None
    def loads(self,state):
        return None
 
def make_object(robot):
    rs=sdf_world(robot)
    if fc.GuiUp:
        ViewProviderSdf(rs.obj.ViewObject)