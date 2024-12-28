import FreeCAD 
import FreeCADGui as fcgui
from .. import wb_utils
import os
from PySide2.QtWidgets import QDialog, QVBoxLayout
from PySide2.QtQuickWidgets import QQuickWidget
from PySide2.QtCore import QUrl,QSize
from ..SdfUtilities import Palette

class link():
    '''
    This will ba called when a link is selected  while command is activated 
    '''
    def __init__(self):
        pass
class joint():
    '''
     called if selected object is a joint during command activation
    '''
    def __init__(self):
        pass
#  world


class world(QDialog):
    '''
    instantiated if selected object is Robot during link activation 
    '''
    def __init__(self,obj):
        super().__init__()
        self.obj=obj #  Robot object 
        self.intialize_qwidget()
        self.initialize_dialog()
        
    def intialize_qwidget(self):
        view =QQuickWidget(self)
        plt=Palette(self)
        view.rootContext().setContextProperty("plt",plt)
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
    """My new command"""

    def GetResources(self):
        return {"Pixmap"  :os.path.join(wb_utils.ICON_PATH,"EditParameters.svg"), # the name of a svg file available in the resources
                "Accel"   : "Shift+e", # a default shortcut (optional)
                "MenuText": "EditParameters",
                "ToolTip" : "edit link,joint and world parameters"}

    def Activated(self):
        obj=fcgui.Selection.getSelection()[0]
        if obj.Proxy.Type=='Cross::Robot':
            wrld=world(obj)
            wrld.show()
        else:
            pass
        return

    def IsActive(self):
        s=fcgui.Selection.getSelection()
        if len(s)>1:
            return False
        else:
            return wb_utils.is_robot_selected() or wb_utils.is_link_selected() or wb_utils.is_joint_selected()
        

fcgui.addCommand("EditParameters", Editor())