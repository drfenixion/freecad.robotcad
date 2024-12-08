
import xml.etree.ElementTree as ET
import os 
# from typing import Union

from . import wb_utils
import FreeCAD 

#_dir=os.path.join(os.path.expanduser("~"),'Documents/RobotDescriptor/robot_descriptor/sdf')

#this class will store element attributes to allow ease of access later 
class Element_Attributes:
    def __init__(self):
        self._name=""
        self._default=""
        self.type=None| str
        
    #name property
    @property
    def name(self):
       return self._name
    @name.setter
    def name(self,value):
        self._name=value
    
    #default property
    @property
    def attr_value(self):
       return self._default
    @attr_value.setter
    def attr_value(self,value):
        self._default=value
    @property
    def type(self):
        return self.type
    @type.setter
    def type(self,val):
        self.type=val
    #description property

    #get a dictionary of all elements 
   
    #get only the name and default value 
    def get_all(self):
        return {"name":self._name,"default":self._default,"type":self.type}


# ToDo 
#  1. add a  logic to detect type and store it this will be used when generating properties in 
#   freecad 

#class to parse the sdf file and generate a dictioanary 
class sdf_parse:
    def __init__(self,file='root.sdf',ignore_list:list=[],version='1.7'):
        '''
        ignore list is a list of properties already available in urdf hence already taken care of in 
        RoboCAD/ cross urdf implementation  and ones that are not preferable 
        '''
        #initialize directory with the root.sdf
        self.root_dir=os.path.join(wb_utils.SDF_PATH,version,file)
        self.version=version
        self.ignore=ignore_list
        #create a dictionary 
        self.Main_ElemDict={}
        #parse tree and store the result in local variable tree
        self.tree=self.parse_tree(self.root_dir)
        #get the root element 
        self.root=self.tree.getroot()
        
        #populate the dictionary with data 
        # call the tree with the parent  root element
        self.Main_ElemDict=self.populate_structure(self.root)
        '''
        main dict structure
        {
            tag: element tag name
            attributes:element attribtes 
            value:element text 
            children:[] children is a list that has dictionaries that folow the same structure 
            }'''
           
    def populate_structure(self,Element:ET.Element):
        #add elements to structure 
        ElemDict={}

        #skip child if name property is not available
        #considering the copydata element in plugin.sdf 
        #that should not be included 
        try:
            ElemDict["tag"]=Element.attrib["name"]
        except:        
            return ElemDict  
        #  do not include elements in the ignore list 
        if Element.tag in self.ignore:
            return  ElemDict
        #find all attributes and store them  in a list
        #this is due to some classes having multiple attributes 
        #store attibute dictionary and descritpion in a tuple 
        #there the first element is a dictionary of  of the attributes and 
        #the second element is a string of description
        #list to store class atrributes 
        self._attr=[]
        for result in Element.findall("attribute"):
            e=Element_Attributes()
            e.name=result.attrib["name"]
            e.attr_value=result.attrib["default"]
            #  get and store the type 
            e.type=result.get("type",default=None)
            self._attr.append(e)
        
        #check to see that attributes are not empty
        if len(self._attr)==0:
            ElemDict["attributes"]=None
        else:
            ElemDict["attributes"]=self._attr
       
        #some elements do not have default value and type so
        #checking if the data exists first before adding it
        # None is used if the following keys are not part of the attributes 
        #check for default
        if "default" in dict(Element.attrib):
                ElemDict["value"]=Element.attrib["default"]    
        else:
             ElemDict["value"]=None
        
        #create a children key
        ElemDict["children"]=[]
        #now   loop  through every other children and store their data in the 
        #data in the children field 
        for child in Element:
            if child.tag =="include":
                #skip include elements for now 
               pass             
            elif child.tag =="element" and child.attrib["name"]!="include":
                _c=self.populate_structure(child)
                # checks to see if an empty dictionary is returned
                if len(_c) !=0:
                    ElemDict["children"].append(_c)
            #add description item
            #  do not include description items for now 
            elif child.tag =="description":
              pass
            else:
               pass 
           
        return ElemDict   
        
# parse and return tree structure
    def parse_tree(self,dir):
        with open(dir) as file:
            tr=ET.parse(file)
        return tr
    #property to be called to get the element dictionary structure 
    @property
    def data_structure(self):
        return self.Main_ElemDict
     
if __name__=="__main__":  
    s=sdf_parse(file="world.sdf")
    d=s.data_structure
    print(d)
    print("done with this stage hopefully")