import FreeCAD as fc
from typing import ForwardRef, List, Optional, Union, cast

from .sensor_proxy import add_sensor_properties_block
from .sensor_proxy import _ViewProviderSensor
from .sensor_proxy_link import SensorProxyLink
from .sensor_proxy_joint import SensorProxyJoint
from ..freecad_utils import error, warn
# Stubs and type hints.
from .sensor import Sensor as CrossSensor  # A Cross::Sensor, i.e. a DocumentObject with Proxy "Sensor". # noqa: E501


def make_sensor(sensor_data: dict, sensor_dir_name: str, doc: Optional[fc.Document] = None) -> CrossSensor | None:
    """Add a Cross::SensorJoint or Cross::SensorLink to the current document."""

    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return

    sensor: CrossSensor = doc.addObject('App::FeaturePython', sensor_data['name'])
    sensor_proxy_map = {
        'link': lambda sensor: SensorProxyLink(sensor),
        'joint': lambda sensor: SensorProxyJoint(sensor),
    }
    sensor_proxy_map[sensor_dir_name](sensor) # ex. SensorProxyLink(sensor) or SensorProxyJoint(sensor)

    add_sensor_properties_block(sensor, sensor_data)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        _ViewProviderSensor(sensor.ViewObject)
    else:
        error('Parameters of ' + sensor_data['name'] + ' not received, interrupt.', True)
        doc.recompute()
        return None

    doc.recompute()
    return sensor
