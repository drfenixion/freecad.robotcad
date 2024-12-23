from .sensor_proxy import SensorProxy


class SensorProxyJoint(SensorProxy):
    """The proxy for Sensor of joint objects."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Cross::SensorJoint'
