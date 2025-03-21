"""Generate a URDF robot from different sources."""

from __future__ import annotations

from pathlib import Path

from urdf_parser_py.urdf import Robot

from xacro import process_file


class UrdfLoader:
    """Generate a urdf_parser_py.urdf.Robot from different sources."""

    def __init__(self):
        pass

    @classmethod
    def load_from_file(cls, filename: [str | Path]) -> Robot:
        """Load from a URDF file."""
        filename = Path(filename)

        # dont use - Robot.from_xml_file(filename),
        # it leads to error - "Unicode strings with encoding declaration are not supported"
        # if URDF has encoding delaration - f.e. <?xml version="1.0" encoding="utf-8"?>
        # process_file () works - ok
        robot = Robot.from_xml_string(
                process_file(filename.expanduser()).toxml(),
        )
        return robot

    @classmethod
    def load_from_string(cls, description: [str | bytes]) -> Robot:
        """Load from an xml string."""
        return Robot.from_xml_string(description)

    @classmethod
    def load_from_parameter_server(
        cls,
        key: str = 'robot_description',
    ) -> Robot:
        """Load from ROS parameter server."""
        return Robot.from_parameter_server(key)
