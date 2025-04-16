import FreeCAD 
import FreeCADGui as fcgui
from .. import wb_utils
import os
from PySide2.QtWidgets import QDialog, QVBoxLayout

from PySide2.QtCore import QUrl,QSize
import PySide2
from ..wb_utils import UI_PATH
from ..sdf.sdf_parser import sdf_schema_parser
from ..sdf import setup

#  Command class 
def show(main_window,dockwidget):
    if dockwidget.__class__.active is False:
        dockwidget.__class__.active=True
        main_window.addDockWidget(PySide2.QtCore.Qt.RightDockWidgetArea,dockwidget)
        
class Editor():
    """Editor command"""

    def GetResources(self):
        return {"Pixmap"  :os.path.join(wb_utils.ICON_PATH,"EditParameters.svg"), # the name of a svg file available in the resources
                "Accel"   : "Shift+e", # a default shortcut (optional)
                "MenuText": "EditParameters",
                "ToolTip" : "edit link,joint and world parameters"}

    def Activated(self):
        obj=fcgui.Selection.getSelection()[0]
        mw=fcgui.getMainWindow() # get main freecad window to add dockwidget
        # load UI 
        if obj.Proxy.Type=='Cross::Link':
            lnk=setup.link_properties(obj,"link",False,mw)
            # dock= fcgui.PySideUic.loadUi(os.path.join(UI_PATH,"linkEditor.ui"))
            show(mw,lnk)
        if obj.Proxy.Type=='Cross::Joint':
            jnt=setup.joint_properties(obj,"joint",False,mw)
            show(mw,jnt)
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