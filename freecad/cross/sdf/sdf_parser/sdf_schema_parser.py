import xml.etree.ElementTree as ET
import os
import re

from ...wb_utils import SDFORMAT_SDF_TEMPLATES_PATH


#this class will store element attributes to allow ease of access later
class Element_Attributes:
    def __init__(self):
        self._name=""
        self._default=""
        self._data_type=""

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
        return self._data_type
    @type.setter
    def type(self,value):
        self._data_type=value
    #description property
    def __str__(self):
        return '{name:%s,value:%s,type:%s}'%(self.name,self._default,self._data_type)
    
    def __repr__(self):
        return '{name:%s,value:%s,type:%s}'%(self.name,self._default,self._data_type)
    #get a dictionary of all elements

    #get only the name and default value
    def get_all(self):
        return {"name":self._name,"default":self._default}
    def get_complete(self)->dict:
        return {"name":self._name,"default":self._default,"type":self._data_type}



#class to parse the sdf file and generate a dictioanary
class sdf_schema_parser:
    def __init__(self,version='1.10',file='root.sdf', sdf_templates_dir = SDFORMAT_SDF_TEMPLATES_PATH,recurse:bool=True
                 ,includeMetaData:bool=True):
        #initialize directory with the root.sdf
        self.root_dir=os.path.join(sdf_templates_dir, version, file)
        self.version=version
        #create a dictionary
        self.Main_ElemDict={}
        #parse tree and store the result in local variable tree
        self.tree=self.parse_tree(self.root_dir)
        #get the root element
        self.root=self.tree.getroot()
        self.recurse=recurse
        self.metaData=includeMetaData
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


    def add_technical_attr_as_attr_to_element(self, technical_attr_name: str, technical_attr_value: str):
        """find first level child element by name and add it`s text as attribute to parent"""

        e = Element_Attributes()
        e.name = self.__class__.get_technical_attr_prefix() + technical_attr_name
        e.attr_value = technical_attr_value
        self._attr.append(e)


    def add_child_text_as_attr_to_element(self, Element: ET.Element, child_name: str):
        """find first level child element by name and add it`s text as attribute to parent"""

        e = Element_Attributes()
        e.name = self.__class__.get_technical_attr_prefix() + child_name
        child_element = Element.find(child_name)
        if child_element is not None and child_element.text is not None:
            e.attr_value = child_element.text
        else:
            e.attr_value = ""
        self._attr.append(e)


    def populate_structure( self, Element:ET.Element):
        #add elements to structure
        ElemDict={}

        #skip child if name property is not available
        #considering the copydata element in plugin.sdf
        #that should not be included
        try:
            ElemDict["tag"]=Element.attrib["name"]
        except:
            return ElemDict
        # add required name 
        required = Element.attrib.get("required", "").strip().lower()
        ElemDict["required"] = required in {"*", "1", "true"} 
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
            e.type=result.attrib["type"]
            self._attr.append(e)
        if self.metaData:
            for technical_attr_name, technical_attr_value in Element.attrib.items():
                self.add_technical_attr_as_attr_to_element(technical_attr_name, technical_attr_value)


            self.add_child_text_as_attr_to_element(Element, 'description')
        else:
            pass

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
                ElemDict["data_type"]=Element.attrib["type"]
        else:
             ElemDict["value"]=None

        #create a children key
        ElemDict["children"]=[]
        #now   loop  through every other children and store their data in the
        #data in the children field
        for child in Element:
            if child.tag =="include":
                #  removed include to ensure only one item can be selected at a time 
                if self.recurse:
                    struct_class=sdf_schema_parser(file=child.attrib["filename"])
                    ElemDict["children"].append(struct_class.data_structure)
                pass
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

    @classmethod
    def get_technical_elements_names(cls) -> list:
        return ["description", "name", "type", "required", "default"]
    @classmethod
    def get_technical_attr_prefix(cls) -> list:
        return 'meta_'
    @classmethod
    def get_technical_attr_prefix_with_attr_symbol(cls) -> list:
        return '@' + cls.get_technical_attr_prefix()
    @classmethod
    def get_manually_added_technical_attributes(cls) -> list:
        attr = []
        for el_name in cls.get_technical_elements_names():
            attr.append(cls.get_technical_attr_prefix_with_attr_symbol() + el_name)
        return attr
