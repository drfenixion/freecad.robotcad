import FreeCAD 
import FreeCADGui as fcgui
from .. import wb_utils
import os
from PySide2.QtWidgets import QDialog, QVBoxLayout
from PySide2.QtQuickWidgets import QQuickWidget
from PySide2.QtCore import QUrl,QSize
from ..SdfUtilities import Palette, WorldProperties,ObjectPropertyBridge

class link(QDialog):
    '''
    This will ba called when a link is selected  while command is activated 
    '''
    def __init__(self,obj):
        super().__init__()
        self.obj=obj
        self.init_widget()
        self.initialize_dialog()
        
    def init_widget(self):
        view =QQuickWidget(self)
        plt=Palette(self)
        prop=ObjectPropertyBridge(self.obj,parent=self)
        view.rootContext().setContextProperty("prop",prop)
        view.rootContext().setContextProperty("plt",plt)
        view.setSource(QUrl(os.path.join(wb_utils.UI_PATH,"qml","link.qml")))
        view.setResizeMode(QQuickWidget.SizeRootObjectToView)
        self.qquick=view
    
    def initialize_dialog(self):
        self.setWindowTitle("link properties")
        layout = QVBoxLayout(self)
        layout.addWidget(self.qquick)
        self.setLayout(layout)
    def sizeHint(self):
        return QSize(700,600)
        
class joint(QDialog):
    '''
     called if selected object is a joint during command activation
    '''
    def __init__(self,obj):
        super().__init__()
        self.obj=obj
        self.init_widget()
        self.initialize_dialog()
        
    def init_widget(self):
        view =QQuickWidget(self)
        plt=Palette(self)
        view.rootContext().setContextProperty("plt",plt)
        view.setSource(QUrl(os.path.join(wb_utils.UI_PATH,"qml","joint.qml")))
        view.setResizeMode(QQuickWidget.SizeRootObjectToView)
        self.qquick=view
    
    def initialize_dialog(self):
        self.setWindowTitle("joint properties")
        layout = QVBoxLayout(self)
        layout.addWidget(self.qquick)
        self.setLayout(layout)
    def sizeHint(self):
        return QSize(700,600)
#  world


class intitialize(QDialog):
    '''
    instantiated if selected obj
    parameter 1:dictionary of additional properties 
    
    initialy the qml file will be generated  stored the current FreeCAD instance temp dir 
    its location stored as a string , 
    
    add neccesary properties and  set the properties initiated proprty to true to avoid 
    repetition 
    
    '''
    def __init__(self,obj,properties=dict,rootqml='',):
        super().__init__()
        self.obj=obj #  Robot object 
        self.intialize_qwidget()
        self.initialize_dialog()
        
    def intialize_qwidget(self):
        view =QQuickWidget(self)
        plt=Palette(self)
        wrld=WorldProperties(self.obj,parent=self)
        prop=ObjectPropertyBridge(self.obj,parent=self)
        view.rootContext().setContextProperty("plt",plt)
        view.rootContext().setContextProperty("world",wrld)
        view.rootContext().setContextProperty("prop",prop)
        view.setSource(QUrl(os.path.join(wb_utils.UI_PATH,"qml","main.qml")))
        view.setResizeMode(QQuickWidget.SizeRootObjectToView)
        
        self.qq=view
      
    def initialize_dialog(self):
        self.setWindowTitle("world")
        layout = QVBoxLayout(self)
        layout.addWidget(self.qq)
        self.setLayout(layout)
    
    def sizeHint(self):
        return QSize(700,400)
    def show(self):
        self.exec_()



#  Command class 

class Editor():
    """Editor command"""

    def GetResources(self):
        return {"Pixmap"  :os.path.join(wb_utils.ICON_PATH,"EditParameters.svg"), # the name of a svg file available in the resources
                "Accel"   : "Shift+e", # a default shortcut (optional)
                "MenuText": "EditParameters",
                "ToolTip" : "edit link,joint and world parameters"}

    def Activated(self):
        obj=fcgui.Selection.getSelection()[0]
        if obj.Proxy.Type=='Cross::Robot':
            wrld=intitialize(obj)
            wrld.show()
        elif obj.Proxy.Type=='Cross::Link':
            lnk=link(obj)
            lnk.exec_()
        else:
            jnt=joint(obj)
            jnt.exec_()
            
        return

    def IsActive(self):
        s=fcgui.Selection.getSelection()
        if len(s)>1:
            return False
        else:
            return wb_utils.is_robot_selected() or wb_utils.is_link_selected() or wb_utils.is_joint_selected()
        

fcgui.addCommand("EditParameters", Editor())