
#Todo 

class create_ui:
    '''
    this class will receive a dictionary of sdf properties and creates a ui  based on the info 
    e.g the ui will   have a group with the name of the key and all children of the key aka values will be
    placed in a group box , a checkbox will be added depending on wheather the item is requred or not 
    types and appropriate widgets will be deduced based on the type e.g a string will have a line_edit ,
    float will have a double spin box , vector will have 3 double spin boxes ,with values x,y,z ... e.t.c
    obj: parent object 
    element_struct : a dictionary of sdf properties 
    fuction is also responsible for addng properties to obj
    and  creating callbacks for ui elements to update the properties 
    '''
    def __init__(self,obj,element_stucture:dict):
        pass