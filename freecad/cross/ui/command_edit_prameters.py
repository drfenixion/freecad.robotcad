import FreeCAD 
import FreeCADGui as fcgui
from .. import wb_utils
import os
from PySide2.QtWidgets import QDialog, QVBoxLayout

from PySide2.QtCore import QUrl,QSize
import PySide2
from ..wb_utils import UI_PATH
from ..sdf.sdf_parser import sdf_schema_parser
from ..SdfUtilities import link_data
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
        mw=fcgui.getMainWindow() # get main freecad window to add dockwidget
        # load UI 
        if obj.Proxy.Type=='Cross::Link':
            lnk=link_data()
            # dock= fcgui.PySideUic.loadUi(os.path.join(UI_PATH,"linkEditor.ui"))
            mw.addDockWidget(PySide2.QtCore.Qt.RightDockWidgetArea,lnk.ui)
           
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