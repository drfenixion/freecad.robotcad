from __future__ import annotations

from collections.abc import Sequence
from enum import Enum
from pathlib import Path

import FreeCAD as fc

from pivy import coin

from .freecad_utils import quantity_as
from .freecad_utils import warn


_Shape = Enum('Shape', ['BOX', 'CONE', 'CUBE', 'CYLINDER', 'SPHERE'])


def transform_from_placement(
        placement: [fc.Placement | fc.Vector | fc.Rotation],
) -> coin.SoTransform:
    """Return the SoTransform equivalent to the placement.

    # Should show a cone entering a cylinder.
    >>> placement = App.Placement(App.Vector(10, 20, 30), App.Rotation(10, 20, 30))
    >>> cyl_to_y = App.Placement(App.Rotation(App.Vector(1, 0, 0), -90).toMatrix())
    >>> cyl = App.ActiveDocument.addObject('Part::Cylinder', 'Cylinder')
    >>> cyl.Placement = placement * cyl_to_y
    >>> cone = coin.SoCone()  # Centered, along y.
    >>> sg = Gui.ActiveDocument.ActiveView.getSceneGraph()
    >>> sep = coin.SoSeparator()
    >>> sep.addChild(transform_from_placement(placement))
    >>> sep.addChild(cone)
    >>> sg.addChild(sep)
    >>> App.ActiveDocument.recompute()
    """
    if (
        not (
            isinstance(placement, fc.Placement)
            or isinstance(placement, fc.Vector)
            or isinstance(placement, fc.Rotation)
        )
    ):
        raise RuntimeError('Argument must be a FreeCAD.{Placement,Vector,Rotation}')
    if isinstance(placement, fc.Vector):
        placement = fc.Placement(placement, fc.Rotation())
    elif isinstance(placement, fc.Rotation):
        placement = fc.Placement(placement.toMatrix())
    transform = coin.SoTransform()
    transform.translation = placement.Base
    transform.rotation = placement.Rotation.Q
    return transform


def cylinder_between_points(
        start_point_mm: Sequence[float],
        end_point_mm: Sequence[float],
        radius_mm: float,
        color: Sequence[float],
):
    """
    Create a cylinder between two points with a given color.

    Parameters:
    - start_point_mm: (x, y, z), the starting point
    - end_point_mm: (x, y, z), the end point
    - radius_mm: float, radius of the cylinder
    - color: (r, g, b) representing the color
             (RGB values between 0 and 1).

    Returns:
    - coin.SoSeparator containing the cylinder with the specified color

    """
    if len(start_point_mm) != 3:
        raise RuntimeError('start_point_mm must have 3 values')
    if len(end_point_mm) != 3:
        raise RuntimeError('end_point_mm must have 3 values')
    if len(color) != 3:
        raise RuntimeError('color must have 3 values')
    if radius_mm < 0:
        raise RuntimeError('radius_mm must be strictly positive')

    # Create an SoSeparator to encapsulate the cylinder and its properties.
    separator = coin.SoSeparator()

    # Create an SoMaterial node to set the color.
    material = coin.SoMaterial()
    material.diffuseColor = coin.SbColor(color[0], color[1], color[2])
    separator.addChild(material)

    height = coin.SbVec3f(
        end_point_mm[0] - start_point_mm[0],
        end_point_mm[1] - start_point_mm[1],
        end_point_mm[2] - start_point_mm[2],
    ).length()

    cylinder = coin.SoCylinder()
    cylinder.radius = radius_mm  # FreeCAD uses mm as unit.
    cylinder.height = height

    # Create an SoTransform node to position and orient the cylinder
    transform = coin.SoTransform()
    transform.translation = coin.SbVec3f(
        start_point_mm[0],
        start_point_mm[1],
        start_point_mm[2],
    )
    transform.pointAt(
        coin.SbVec3f(
            end_point_mm[0],
            end_point_mm[1],
            end_point_mm[2],
        ),
    )

    separator.addChild(transform)
    separator.addChild(cylinder)

    return separator


def arrow_group(
        points: list[fc.Vector],
        color: tuple[float, float, float] = (0.0, 0.0, 1.0),
        scale: float = 1.0,
) -> coin.SoSeparator:
    """Return the SoSeparator of an arrow between two points.

    """
    if len(points) < 2:
        raise RuntimeError('At least 2 points expected')
    if len(color) != 3:
        raise RuntimeError('color must have 3 values')

    # Ratio cone_height / length_between_points.
    head_height_ratio = 0.3

    # Ratio cone_base_radius / cylinder_radius.
    head_radius_ratio = 2.0

    p0, p1 = points[:2]
    if ((not isinstance(p0, fc.Vector))
            or (not isinstance(p1, fc.Vector))):
        raise RuntimeError('The list of points must contains FreeCAD.Vector instances')

    v = p1 - p0
    # Rotation to bring the x-axis to v.
    rotation = fc.Rotation(fc.Vector(1.0, 0.0, 0.0), v)

    # An SoCylinder is centered, along y.
    cylinder = coin.SoCylinder()
    cylinder.height = v.Length * (1.0 - head_height_ratio)
    cylinder.radius = v.Length / 20.0 * scale

    # Transform to bring the cylinder with origin at bottom, along x.
    along_x = coin.SoTransform()
    # Translation after the rotation.
    along_x.translation = cylinder.height.getValue() / 2.0, 0.0, 0.0
    along_x.rotation = fc.Rotation(fc.Vector(0.0, 0.0, 1.0), -90).Q

    cylinder_sep = coin.SoSeparator()
    cylinder_sep.addChild(along_x)
    cylinder_sep.addChild(cylinder)

    # An SoCone is centered, along y+.
    cone = coin.SoCone()
    cone.bottomRadius = cylinder.radius.getValue() * head_radius_ratio
    cone.height = v.Length * head_height_ratio

    # Transform to bring the top of the cone to p1.
    cone_trans = coin.SoTransform()
    # Translation after the rotation.
    cone_trans.translation = v.Length - (cone.height.getValue() / 2.0), 0.0, 0.0
    cone_trans.rotation = fc.Rotation(fc.Vector(0.0, 0.0, 1.0), -90).Q

    cone_sep = coin.SoSeparator()
    cone_sep.addChild(cone_trans)  # Translation will be done first.
    cone_sep.addChild(cone)

    # Transform to the first point (with orientation).
    transform = coin.SoTransform()
    transform.translation = p0
    transform.rotation = rotation.Q

    coin_color = coin.SoBaseColor()
    coin_color.rgb = color

    group = coin.SoSeparator()
    group.addChild(transform)
    group.addChild(coin_color)
    group.addChild(cylinder_sep)
    group.addChild(cone_sep)
    return group


def face_group(
        points: list[fc.Vector],
        color: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> coin.SoSeparator:
    if len(points) < 3:
        raise RuntimeError('At least 3 points expected')
    if len(color) != 3:
        raise RuntimeError('color must have 3 values')

    # Create a material to fill the square.
    material = coin.SoMaterial()
    material.diffuseColor = color

    # Create the vertices of the face.
    vertices = coin.SoVertexProperty()
    n = len(points)
    vertices.vertex.setValues(0, n, points)

    # Create the face.
    indices = coin.SoIndexedFaceSet()
    indices.coordIndex.setValues(0, n + 1, list(range(n)) + [-1])

    # Set the material for the face.
    # indices.addChild(material)

    # Create a group and add the vertices and indices.
    group = coin.SoSeparator()
    group.addChild(material)
    group.addChild(vertices)
    group.addChild(indices)
    return group


def cylinder_between_points(
        start_point_mm: Sequence[float],
        end_point_mm: Sequence[float],
        radius_mm: float,
        color: Sequence[float],
) -> coin.SoSeparator:
    """
    Create a cylinder between two points.

    Parameters:
    - start_point_mm: (x, y, z), the starting point
    - end_point_mm: (x, y, z), the end point
    - radius_mm: float, radius of the cylinder
    - color: (r, g, b) representing the color
             (RGB values between 0 and 1).

    Returns:
    - coin.SoSeparator containing the cylinder

    """
    return _cylinder_or_cone_between_points(
        _Shape.CYLINDER,
        start_point_mm,
        end_point_mm,
        radius_mm,
        color,
    )


def cone_between_points(
        start_point_mm: Sequence[float],
        end_point_mm: Sequence[float],
        radius_mm: float,
        color: Sequence[float],
) -> coin.SoSeparator:
    """
    Create a cone between two points.

    Parameters:
    - start_point_mm: (x, y, z), the starting point
    - end_point_mm: (x, y, z), the end point
    - radius_mm: float, bottom radius of the cone
    - color: (r, g, b) representing the color
             (RGB values between 0 and 1).

    Returns:
    - coin.SoSeparator containing the cone

    """
    return _cylinder_or_cone_between_points(
        _Shape.CONE,
        start_point_mm,
        end_point_mm,
        radius_mm,
        color,
    )


def frame_group(
        length_mm: [float | fc.Units.Quantity] = 200.0,
        diameter_ratio_to_length: float = 0.05,
        axis_start_ratio: float = 0.0,
) -> coin.SoSeparator:
    """
    Create a frame with 3 cylinders.

    Parameters:
    - length_mm: length of the frame (in mm)
    - diameter_ratio_to_length: ratio of the diameter to the length.
          I.e. the diameter is diameter_ratio_to_length * length_mm.
    - axis_start_ratio: ratio of the distance of the start of the axis.
          The cylinder representing the axis will start at
          axis_start_ratio * length_mm.

    Returns:
    - coin.SoSeparator containing the representation of a frame.

    """
    if isinstance(length_mm, fc.Units.Quantity):
        length_mm = quantity_as(length_mm, 'mm')
    sep = coin.SoSeparator()
    if diameter_ratio_to_length < 0:
        raise RuntimeError('diameter_ratio_to_length must be positive')
    radius = diameter_ratio_to_length * length_mm / 2.0
    axis_start = axis_start_ratio * length_mm
    sep.addChild(
        cylinder_between_points(
            start_point_mm=(axis_start, 0.0, 0.0),
            end_point_mm=(length_mm, 0.0, 0.0),
            radius_mm=radius,
            color=(1.0, 0.0, 0.0),
        ),
    )
    sep.addChild(
        cylinder_between_points(
            start_point_mm=(0.0, axis_start, 0.0),
            end_point_mm=(0.0, length_mm, 0.0),
            radius_mm=radius,
            color=(0.0, 1.0, 0.0),
        ),
    )
    sep.addChild(
        cylinder_between_points(
            start_point_mm=(0.0, 0.0, axis_start),
            end_point_mm=(0.0, 0.0, length_mm),
            radius_mm=radius,
            color=(0.0, 0.0, 1.0),
        ),
    )
    return sep


def tcp_group(
        tcp_length_mm: [float | fc.Units.Quantity] = 200.0,
        tcp_diameter_ratio_to_length: float = 0.25,
        tcp_color: Sequence[float] = (0.7, 0.7, 0.7),
        axis_length_mm: [float | fc.Units.Quantity] = 300.0,
        axis_diameter_ratio_to_length: float = 0.05,
) -> coin.SoSeparator:
    """Return the SoSeparator reprenting a TCP.

    Return the SoSeparator representing a Tool Center Point, i.e. 3 cylinder
    representing the frame and a spiky cylinder symbolizing the tool (with the
    spike of the cone at the origin).

    """
    if len(tcp_color) != 3:
        raise RuntimeError('tcp_color must have 3 values')
    if isinstance(tcp_length_mm, fc.Units.Quantity):
        tcp_length_mm = quantity_as(tcp_length_mm, 'mm')
    if isinstance(axis_length_mm, fc.Units.Quantity):
        axis_length_mm = quantity_as(axis_length_mm, 'mm')

    sep = coin.SoSeparator()

    sep.addChild(
        frame_group(
            length_mm=axis_length_mm,
            diameter_ratio_to_length=axis_diameter_ratio_to_length,
            axis_start_ratio=0.2,
        ),
    )

    tcp_radius = tcp_diameter_ratio_to_length * tcp_length_mm / 2.0
    tcp_cone_end = -0.25 * tcp_length_mm
    tcp_cyl_end = -tcp_length_mm - tcp_cone_end
    sep.addChild(
        cone_between_points(
            start_point_mm=(0.0, 0.0, tcp_cone_end),
            end_point_mm=(0.0, 0.0, 0.0),
            radius_mm=tcp_radius,
            color=tcp_color,
        ),
    )
    sep.addChild(
        cylinder_between_points(
            start_point_mm=(0.0, 0.0, tcp_cone_end),
            end_point_mm=(0.0, 0.0, tcp_cyl_end),
            radius_mm=tcp_radius,
            color=tcp_color,
        ),
    )
    return sep


def save_separator_to_file(
        separator: coin.SoSeparator,
        filename: [Path | str],
        file_format: str = 'iv',
) -> None:
    """Save the content of an SoSeparator to a file in Inventor or VRML format.

    Parameters:
    - separator: the content to be saved.
    - filename: the path to the file to save.
    - file_format: either 'iv' for Inventor (ASCII) or 'wrl' for VRML (binary).

    """
    path = str(Path(filename).expanduser().absolute())

    # Create an SoOutput to write to a file
    output = coin.SoOutput()

    # Open the file for writing
    if not output.openFile(path):
        warn(f'Error: Cannot open file {filename} for writing.')
        return

    # Create an SoWriteAction to perform the writing
    write_action = coin.SoWriteAction(output)

    # Set the file format (Inventor or VRML)
    if file_format == 'iv':
        # Use ASCII format for Inventor.
        write_action.getOutput().setBinary(False)
    elif file_format == 'wrl':
        # Use binary format for VRML.
        write_action.getOutput().setBinary(True)
    else:
        warn("Error: Unsupported file format. Use 'iv' for Inventor or 'wrl' for VRML.")
        return

    # Apply the write action to the separator
    write_action.apply(separator)

    # Close the file
    output.closeFile()


def _cylinder_or_cone_between_points(
        shape: _Shape,
        start_point_mm: Sequence[float],
        end_point_mm: Sequence[float],
        radius_mm: float,
        color: Sequence[float],
) -> coin.SoSeparator:
    """
    Create a cylinder or a cone between two points.

    Parameters:
    - shape: _Shape.CYLINDER or _Shape.CONE.
    - start_point_mm: (x, y, z), the starting point
    - end_point_mm: (x, y, z), the end point
    - radius_mm: float, radius of the cylinder
    - color: (r, g, b) representing the color
             (RGB values between 0 and 1).

    Returns:
    - coin.SoSeparator containing the cylinder

    """
    if len(start_point_mm) != 3:
        raise RuntimeError('start_point_mm must have 3 values')
    if len(end_point_mm) != 3:
        raise RuntimeError('end_point_mm must have 3 values')
    if len(color) != 3:
        raise RuntimeError('color must have 3 values')
    if radius_mm < 0:
        raise RuntimeError('radius_mm must be strictly positive')
    if shape not in (_Shape.CYLINDER, _Shape.CONE):
        raise RuntimeError('shape must be _Shape.CYLINDER or _Shape.CONE')

    separator = coin.SoSeparator()

    material = coin.SoMaterial()
    material.diffuseColor = coin.SbColor(color[0], color[1], color[2])
    separator.addChild(material)

    start = coin.SbVec3f(
        start_point_mm[0],
        start_point_mm[1],
        start_point_mm[2],
    )
    end = coin.SbVec3f(
        end_point_mm[0],
        end_point_mm[1],
        end_point_mm[2],
    )
    shape_axis = end - start
    height = shape_axis.length()

    if height == 0.0:
        return separator

    # Both SoCylinder and SoCone have their axis along the y axis.
    # Rotation from y to shape_axis.
    y_axis = coin.SbVec3f(0.0, 1.0, 0.0)
    axis_rot = coin.SbRotation(y_axis, shape_axis)

    # Both SoCylinder and SoCone have their origin at the center of the
    # shape. We need to translate so that the bottom circle of the shape
    # is at the starting point.
    half_shape_axis = coin.SbVec3f(0.0, height / 2.0, 0.0)
    transform = coin.SoTransform()
    transform.rotation = axis_rot
    transform.translation = start + transform.rotation.getValue() * half_shape_axis

    separator.addChild(transform)

    if shape == _Shape.CYLINDER:
        coin_shape = coin.SoCylinder()
        coin_shape.radius = radius_mm  # FreeCAD uses mm as Coin unit.
    else:
        coin_shape = coin.SoCone()
        coin_shape.bottomRadius = radius_mm  # FreeCAD uses mm as Coin unit.
    coin_shape.height = height
    separator.addChild(coin_shape)

    return separator
