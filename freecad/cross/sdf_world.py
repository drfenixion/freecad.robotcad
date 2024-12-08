import FreeCAD as fc 
from . import wb_utils
from . import sdf
class sdf_world:
    Type="Cross::sdf"
    def __init__(self,Robot):
        self.version="1.7"
        self.robot=Robot
        self.obj=fc.ActiveDocument.addObject("App::FeaturePython", "sdf_world_object")
        Robot.addObject(self.obj)
        self.initialize_properties()
        
    def initialize_properties(self):
        self.obj.addProperty("App::PropertyString","version","sdf")
        self.obj.version=self.version
        
    def initialize_ui(self)->None:
        pass
class ViewProviderSdf:
    def __init__(self,sdf_obj):
        sdf_obj.Proxy=self
    
    def getIcon(self):
        return str(wb_utils.ICON_PATH / 'sdf.svg')
    def setEdit(self,obj,mode):
        return False
    def doubleClicked(self,obj):
        gui_doc = obj.Document
        if not gui_doc.getInEdit():
            gui_doc.setEdit(obj.Object.Name)
        print("item clicked\n")
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