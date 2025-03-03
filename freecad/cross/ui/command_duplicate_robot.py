# Duplicate a Cross::Robot, its links and joints.
#
# The `Collision`, `Visual`, and `Real` parts of the new links point to the same
# objects as the original links (i.e. the linked objects are not duplicated).

import FreeCADGui as fcgui
import FreeCAD as fc

from PySide import QtGui # FreeCAD's PySide!
from freecad.cross.sensors.sensor_proxy import get_sensors_data

from ..wb_utils import is_sensor_joint, is_sensor_link, ros_name

# Typing hints.
from freecad.cross.controller_proxy import get_controllers_data
from freecad.cross.robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
DO = fc.DocumentObject


class _DuplicateRobotCommand:
    """The command definition to duplicate a Robot object."""

    def GetResources(self):
        return {
            'Pixmap': 'Std_DuplicateSelection',
            'MenuText': 'Duplicate Robot',
            'Accel': 'D, R',
            'ToolTip': 'Duplicate a CROSS::Robot',
        }

    def IsActive(self):
        # Import late to avoid slowing down workbench start-up.
        from ..wb_utils import is_robot

        objs = fcgui.Selection.getSelection()
        if not objs:
            return False
        if is_robot(objs[0]):
            return True
        return False

    def Activated(self):
        # Import late to avoid slowing down workbench start-up.
        from ..freecad_utils import validate_types
        from ..freecad_utils import warn
        from .duplicate_robot_dialog import DuplicateRobotDialog

        sel = fcgui.Selection.getSelection()
        if not validate_types(sel, ['Cross::Robot']):
            warn('Please select a Cross::Robot', True)
            return

        form = DuplicateRobotDialog(sel[0].Label)

        form.exec()
        for _ in range(form.number_of_duplicates):
            duplicate(sel[0], form.base_name)


def copy_scripted_obj_props(
    orig_obj: DO,
    new_obj: DO,
    exclude_attrs: list = ['Label', 'Proxy', '_Part_ShapeCache'],
    exclude_attrs_addition: list = [],
    exclude_attrs_view_obj: list = ['Label', 'Proxy'],
) -> DO:
    """Copy properties of Freecad scripted object like CrossLink or CrossJoint to anothre one.
    
    exlude_attrs - not copied root attributes
    exlude_attrs_addition - addition not copied root attributes
    exlude_attrs_view_obj - not copied view object root attributes
    """

    # select only props
    orig_link_props = [
        a for a in vars(orig_obj) if not a.startswith('__')
            and a not in exclude_attrs
            and a not in exclude_attrs_addition
            and not callable(getattr(orig_obj, a))
    ]
    orig_link_viewobject_props = [
        a for a in vars(orig_obj.ViewObject) if not a.startswith('__')
            and a not in exclude_attrs_view_obj
            and not callable(getattr(orig_obj.ViewObject, a))
    ]
    # set selected props to new link
    for el in orig_link_props:
        if el != 'ViewObject':
            try:
                orig_attr = getattr(orig_obj, el)
                if orig_attr:
                    setattr(new_obj, el, orig_attr)
            except RuntimeError:
                pass
    for el in orig_link_viewobject_props:
        try:
            orig_attr = getattr(orig_obj.ViewObject, el)
            if orig_attr:
                setattr(new_obj.ViewObject, el, orig_attr)
        except RuntimeError:
            pass

    return     

def duplicate(orig_robot: CrossRobot, base_name: str) -> CrossRobot:
    """Duplicate a Cross::Robot, its links and joints."""
    # Import late to avoid slowing down workbench start-up.
    from freecad.cross.joint_proxy import make_joint
    from freecad.cross.link_proxy import make_link
    from freecad.cross.robot_proxy import make_robot
    from freecad.cross.controller_proxy import make_controller
    from freecad.cross.controller_proxy import make_broadcaster
    from freecad.cross.sensors.sensor_factory import make_sensor

    doc = orig_robot.Document
    robot = make_robot(f'{base_name}_000', doc)

    orig_proxy = orig_robot.Proxy
    sensors = get_sensors_data()

    # cloning links
    for orig_link in orig_proxy.get_links():
        link = make_link(orig_link.Label, doc)
        robot.addObject(link)
        copy_scripted_obj_props(orig_link, link)
        
        # remove links to old link sensors
        new_group = []
        for v in link.Group:
            if not is_sensor_link(v):
                new_group.append(v)
        if link.Group != new_group:
            link.Group = new_group

        # cloning sensors
        for orig_sensor in orig_link.Proxy.get_sensors():
            sensor_dir_name = 'link'
            # remove not first same name instance postfix # 001, 002, etc.
            orig_sensor_name_without_postfix = orig_sensor.Name.rstrip('0123456789')
            sensor = make_sensor(sensors[sensor_dir_name][orig_sensor_name_without_postfix], sensor_dir_name)
            link.addObject(sensor)
            copy_scripted_obj_props(orig_sensor, sensor)
    doc.recompute()

    # cloning joints
    for orig_joint in orig_proxy.get_joints():
        joint = make_joint(orig_joint.Label, doc)
        robot.addObject(joint)
        copy_scripted_obj_props(orig_joint, joint)

        # remove links to old joint sensors
        new_group = []
        for v in joint.Group:
            if not is_sensor_joint(v):
                new_group.append(v)
        if joint.Group != new_group:
            joint.Group = new_group

        # cloning sensors
        for orig_sensor in orig_joint.Proxy.get_sensors():
            sensor_dir_name = 'joint'
            # remove not first same name instance postfix # 001, 002, etc.
            orig_sensor_name_without_postfix = orig_sensor.Name.rstrip('0123456789')
            sensor = make_sensor(sensors[sensor_dir_name][orig_sensor_name_without_postfix], sensor_dir_name)
            joint.addObject(sensor)
            copy_scripted_obj_props(orig_sensor, sensor)
    doc.recompute()

    # get data controllers and broadcasters
    controllers = get_controllers_data()
    
    # cloning controllers
    for orig_controller in orig_proxy.get_controllers():
        controller_data = None 
        for key, value in controllers['controllers'].items():
            if value['controller_plugin_class_name'] == orig_controller.plugin_class_name:
                controller_data = value
                break
        
        # fix a bug with old broadcasters that has controllers type
        if controller_data is None:
            for key, value in controllers['broadcasters'].items():
                if value['controller_plugin_class_name'] == orig_controller.plugin_class_name:
                    controller_data = value
                    break

            if controller_data is None:
                raise ValueError("Cant get controller data for " + ros_name(orig_controller) + ' controller.\
 It can be too old verion. Recreate it before cloning.')

            controller = make_broadcaster(controller_data, doc=doc)
        else:
            controller = make_controller(controller_data, doc=doc)
        
        robot.addObject(controller)
        copy_scripted_obj_props(orig_controller, controller, exclude_attrs_addition = ['_Type'])
    doc.recompute()

    # cloning broadcasters
    for orig_broadcaster in orig_proxy.get_broadcasters():
        broadcaster_data = None 
        for key, value in controllers['broadcasters'].items():
            if value['controller_plugin_class_name'] == orig_broadcaster.plugin_class_name:
                broadcaster_data = value
                break
        
        if broadcaster_data is None:
            raise ValueError("Cant get broadcaster data for " + ros_name(orig_broadcaster) + ' broadcaster.\
 It can be too old verion. Recreate it before cloning.')

        broadcaster = make_broadcaster(broadcaster_data, doc=doc)
        robot.addObject(broadcaster)
        copy_scripted_obj_props(orig_broadcaster, broadcaster)
    doc.recompute()

    copy_scripted_obj_props(orig_robot, robot, exclude_attrs_addition = ['Group'])

    for joint_variable_name in orig_proxy.joint_variables.values():
        setattr(robot, joint_variable_name, getattr(orig_robot, joint_variable_name))

    doc.recompute()
    return robot


fcgui.addCommand('DuplicateRobot', _DuplicateRobotCommand())
