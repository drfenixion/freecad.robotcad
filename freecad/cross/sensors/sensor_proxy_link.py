from .sensor_proxy import SensorProxy


class SensorProxyLink(SensorProxy):
    """The proxy for Sensor of link objects."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Cross::SensorLink'
