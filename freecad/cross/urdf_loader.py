"""Generate a URDF robot from different sources."""

from __future__ import annotations

from pathlib import Path
import re

from freecad.cross.utils import get_parent_by_pattern
from urdf_parser_py.urdf import Robot

from xacro import init_stacks, process_doc, XacroException
from . import xacro as xacro_inst
import xml


def custom_xacro_process_file(input_file_name, **kwargs):
    """Custom main processing pipeline of xacro for correct xml
    to avoid errors (f.e. ExpatError: unbound prefix)"""
    # initialize file stack for error-reporting
    init_stacks(input_file_name)
    # parse the document into a xml.dom tree
    doc = custom_xacro_parse(None, input_file_name)

    # perform macro replacement
    process_doc(doc, **kwargs)

    # add xacro auto-generated banner
    banner = [
        xml.dom.minidom.Comment(c) for c in
        [
            " %s " % ('=' * 83),
            " |    This document was autogenerated by xacro from %-30s | " % input_file_name,
            " |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED  %-30s | " % "",
            " %s " % ('=' * 83),
        ]
    ]
    first = doc.firstChild
    for comment in banner:
        doc.insertBefore(comment, first)

    return doc


def custom_xacro_parse(inp, filename=None):
    """
    Parse input or filename into a DOM tree.
    If inp is None, open filename and load from there.
    Otherwise, parse inp, either as string or file object.
    If inp is already a DOM tree, this function is a noop.
    :return:xml.dom.minidom.Document
    :raise: xml.parsers.expat.ExpatError
    """
    f = None
    if inp is None:
        try:
            inp = f = open(filename)
        except IOError as e:
            # do not report currently processed file as "in file ..."
            xacro_inst.filestack.pop()
            raise XacroException(e.strerror + ": " + e.filename, exc=e)


    ### Custom modification of xml before xacro parse it
    # used for avoid errors of parser

    # Adding custom namespaces in case it missed for suppress parser error
    if hasattr(inp, 'read'):
        inp = inp.read()
    if inp.find('xmlns:sensor=') == -1:
        inp = inp.replace('<robot ', '<robot xmlns:sensor="http://example.com/sensor" ', 1)
    if inp.find('xmlns:controller=') == -1:
        inp = inp.replace('<robot ', '<robot xmlns:controller="http://example.com/sensor" ', 1)
    if inp.find('xmlns:interface=') == -1:
        inp = inp.replace('<robot ', '<robot xmlns:interface="http://example.com/sensor" ', 1)
    if inp.find('xmlns:xacro=') == -1:
        inp = inp.replace('<robot ', '<robot xmlns:xacro="http://example.com/sensor" ', 1)

    # Cut problematic urdf fragments (looks like urdf_parser_py can parse it)
    # https://github.com/ros/urdf_parser_py/issues/90
    if inp.find('<transmission') > -1:
        pattern = r'<transmission .*?>(?:(?!<transmission>|</transmission>)[\s\S])*(<rightActuator|<gap_joint|<use_simulated_gripper_joint|<passive_joint|<simulated_actuated_joint|<rightActuator|<flexJoint|<rollJoint|<passive_joint)(?:(?!<transmission>)[\s\S])*?<\/transmission>'
        flags = re.MULTILINE
        inp = re.sub(pattern, '', inp, flags=flags)

    # replace xacro:include filenames with absolute paths
    # in case of not compiled package gotten from robot_description module or as import URDF
    if inp.find('<xacro:include') > -1:
        # f.e. <xacro:include filename="$(find nextage_description)/urdf/materials.urdf.xacro" />
        pattern = r'<xacro:include[\s\S]*?filename="\$\(find (.*?)\)/.*?"[\s\S]*?/>'
        flags = re.MULTILINE
        match = re.search(pattern, inp, flags=flags)
        if match:
            pkg_name = match.group(1)

            if pkg_name:
                if UrdfLoader.robot_from_urdf:
                    pkg_path = UrdfLoader.robot_from_urdf.get_real_pkg_path(pkg_name)
                if not pkg_path:
                    pkg_path, relative_file_path = get_parent_by_pattern(filename, ['package.xml'])
                pattern = rf'(<xacro:include[\s\S]*?filename=")\$\(find {pkg_name}\)(/.*?"[\s\S]*?/>)'
                inp = re.sub(pattern, rf'\1{pkg_path}\2', inp, flags=flags)

        # f.e. <xacro:include filename="package://nextage_description/urdf/materials.urdf.xacro" />
        pattern = r'<xacro:include[\s\S]*?filename="package://(.*?)/.*?"[\s\S]*?/>'
        flags = re.MULTILINE
        match = re.search(pattern, inp, flags=flags)
        if match:
            pkg_name = match.group(1)

            if pkg_name:
                if UrdfLoader.robot_from_urdf:
                    pkg_path = UrdfLoader.robot_from_urdf.get_real_pkg_path(pkg_name)
                if not pkg_path:
                    pkg_path, relative_file_path = get_parent_by_pattern(filename, ['package.xml'])
                pattern = rf'(<xacro:include[\s\S]*?filename=")package://{pkg_name}(/.*?"[\s\S]*?/>)'
                inp = re.sub(pattern, rf'\1{pkg_path}\2', inp, flags=flags)

        if UrdfLoader.robot_from_urdf and not UrdfLoader.robot_from_urdf.package_path:
            UrdfLoader.robot_from_urdf.package_path = pkg_path
    ### END Custom modification of xml before xacro parse it


    try:
        if isinstance(inp, str):
            return xml.dom.minidom.parseString(inp)
        elif hasattr(inp, 'read'):
            return xml.dom.minidom.parse(inp)
        return inp

    finally:
        if f:
            f.close()


class UrdfLoader:
    """Generate a urdf_parser_py.urdf.Robot from different sources."""

    robot_from_urdf=None

    def __init__(self):
        pass

    @classmethod
    def load_from_file(cls, filename: [str | Path], robot_from_urdf = None) -> Robot:
        """Load from a URDF file."""
        filename = Path(filename)
        cls.robot_from_urdf = robot_from_urdf
        # dont use - Robot.from_xml_file(filename),
        # it leads to error - "Unicode strings with encoding declaration are not supported"
        # if URDF has encoding delaration - f.e. <?xml version="1.0" encoding="utf-8"?>
        # process_file () works - ok
        robot = Robot.from_xml_string(
            custom_xacro_process_file(filename.expanduser()).toxml(),
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
