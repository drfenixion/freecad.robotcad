from .sdf_schema_parser import sdf_schema_parser

import xml.etree.ElementTree as ET
import xmltodict

from ...utils import dict_to_xml


class sdf_tree:
    def __init__(self, sfile:str,metaData:bool=True,recurse:bool=True,minimal:bool=False):
        '''sfile : file name to read the element properties from  located in ../sdf \n
        use the get_elem  property  to get the initialized element '''
        #initialize class
        self.struct_class=sdf_schema_parser(file=sfile,includeMetaData=metaData,recurse=recurse,minimal=minimal)
        #get the dictionary structure
        self.structured=self.struct_class.data_structure
        #a stack of parent elements
        #this stack is provided to allow for  having a parent  key in the dictionary
        #to help when creating an xnl tree by adding subnode
        #get stuff started
        self.create_root()
        if len(self.structured["children"])==0:
            self._root_elem.text=self.structured["value"]
        else:
            self.construct_tree(self._root_elem, self.structured["children"])

        self.e_tree=ET.ElementTree(self._root_elem)
    #create the root element
    #this does not need other properties as it does not have them ,this I'm sure of
    #so no need to add them
    def create_root(self)->ET.Element:
        self._root_elem=ET.Element(self.structured["tag"])
        if self.structured["attributes"] is not None:
            for attr in self.structured["attributes"]:
                self._root_elem.set(attr.name,attr.attr_value)

    def construct_tree(self, parent_elem: ET.Element, st_lst)->ET.Element:

        for child in st_lst:
            # attr=dict()
            s=ET.SubElement(parent_elem,child["tag"])
            if child["attributes"] is not None:
                for attr in child["attributes"]:
                    #recall  attributes are stored as class Element_attributes defined in RD_parse_sdf.py
                    #create a dictionary with the attributes name as the key and attribute_value as the value
                    #for each attribute in the list
                    # attr[_att.name]=_att.attr_value
                    s.set(attr.name,attr.attr_value)

            if child["value"] is not None:
                s.text=child["value"]
            if len(child["children"]) > 0:
                self.construct_tree(s,child["children"])

    @property
    def get_tree(self)->ET.ElementTree:
        return self.e_tree
    @property
    def get_element(self)->ET.Element:
        return self._root_elem
    @property
    def get_element_as_dict(self) -> dict:
        """Convert root element (ET) to dictionary"""
        return xmltodict.parse(ET.tostring(self._root_elem))


def sdf_dict_to_xml(sdf_dict: dict, full_document: bool = True, pretty: bool = False) -> str:
    """ Convert (sdf_dict -> xml). Also removes meta attributes before converting.
    Meta attributes (ex. @meta_description) were added manually and should be removed"""

    return dict_to_xml(
        sdf_dict,
        keys_to_remove_before_convert = sdf_schema_parser.get_manually_added_technical_attributes(),
        remove_keys_recursively = True,
        full_document = full_document,
        pretty = pretty,
    )
