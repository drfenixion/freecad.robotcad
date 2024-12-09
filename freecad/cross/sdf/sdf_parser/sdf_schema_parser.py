import xml.etree.ElementTree as ET
import os 
import re

from ...wb_utils import SDFORMAT_SDF_TEMPLATES_PATH


#this class will store element attributes to allow ease of access later 
class Element_Attributes:
    def __init__(self):
        self._name=""
        self._default=""
        
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
  
    #description property

    #get a dictionary of all elements 
   
    #get only the name and default value 
    def get_all(self):
        return {"name":self._name,"default":self._default}


    
#class to parse the sdf file and generate a dictioanary 
class sdf_schema_parser:
    def __init__(self,version='1.10',file='root.sdf', sdf_templates_dir = SDFORMAT_SDF_TEMPLATES_PATH):
        #initialize directory with the root.sdf
        self.root_dir=os.path.join(sdf_templates_dir, version, file)
        self.version=version
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
           
    def populate_structure(self, Element:ET.Element):
        #add elements to structure 
        ElemDict={}

        #skip child if name property is not available
        #considering the copydata element in plugin.sdf 
        #that should not be included 
        try:
            ElemDict["tag"]=Element.attrib["name"]
        except:        
            return ElemDict  
        
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
            self._attr.append(e)

        # add description child tag as attribute
        e=Element_Attributes()
        e.name="element_description"
        description = Element.find("description")
        if description is not None and description.text is not None:
            e.attr_value = description.text
        else:
            e.attr_value = ""
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
                struct_class=sdf_schema_parser(file=child.attrib["filename"])
                ElemDict["children"].append(struct_class.data_structure)
            elif child.tag =="element" \
            and 'name' in child.attrib \
            and child.attrib["name"] != "include":
                _c=self.populate_structure(child)
                if len(_c) !=0:
                    ElemDict["children"].append(_c)
            elif child.tag =="element" \
            and 'name' not in child.attrib \
            and 'copy_data' in child.attrib:
                # case of "copy_data" attr. It is technical tag. Ignore it

                # copy_data - This is a special element that should not be specified in an SDFormat file.
                # It automatically copies child elements into the SDFormat element so that a plugin can access the data
                pass
            elif child.tag =="element" \
            and 'name' not in child.attrib:
                # check for any other technical tag cases.
                # There is not other cases but check saved for future
                pass
            #add description item 
            elif child.tag =="description":
                ElemDict["description"] = child.text
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
