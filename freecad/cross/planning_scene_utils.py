from __future__ import annotations

from typing import Callable

import FreeCAD as fc

from geometry_msgs.msg import Pose

from moveit_msgs.msg import PlanningScene
from moveit_msgs.msg import PlanningSceneWorld
from moveit_msgs.msg import CollisionObject

from shape_msgs.msg import Mesh
from shape_msgs.msg import Plane
from shape_msgs.msg import SolidPrimitive

from pivy import coin

from .coin_utils import frame_group


def coin_from_planning_scene_msg(
        scene: PlanningScene,
        plane_sides_mm: float,
        subframe_length_mm: [float | fc.Units.Quantity] = 100.0,
        ) -> coin.SoSeparator:
    """Convert a moveit_msgs.msg.PlanningScene to Coin3D.

    Only the world is considered for now, not the robot state.

    Parameters:
    - scene: moveit_msgs.msg.PlanningScene
    - plane_sides_mm: length of the sides of the planes, in millimeters.

    Returns:
    - coin.SoSeparator containing Coin3D nodes representing the PlanningScene.

    """
    separator = coin.SoSeparator()

    separator.addChild(coin_from_planning_scene_world(scene.world,
        plane_sides_mm=plane_sides_mm,
        subframe_length_mm=subframe_length_mm))

    return separator


def coin_from_planning_scene_world(
        world: PlanningSceneWorld,
        plane_sides_mm: float,
        subframe_length_mm: [float | fc.Units.Quantity] = 100.0,
        ) -> coin.SoSeparator:
    """Convert a moveit_msgs.msg.PlanningSceneWorld to Coin3D.

    Only the collision objects are considered for now, not the octomap.

    Parameters:
    - scene: moveit_msgs.msg.PlanningSceneWorld
    - plane_sides_mm: length of the sides of the planes, in millimeters.

    Returns:
    - coin.SoSeparator containing Coin3D nodes representing the PlanningScene.

    """
    separator = coin.SoSeparator()
    for co in world.collision_objects:
        separator.addChild(coin_from_collision_object(co,
            plane_sides_mm=plane_sides_mm,
            subframe_length_mm=subframe_length_mm))
    return separator


def coin_from_collision_object(
        co: CollisionObject,
        plane_sides_mm: float,
        subframe_length_mm: [float | fc.Units.Quantity] = 100.0,
        ) -> coin.SoSeparator:
    separator = coin.SoSeparator()
    separator.setName(co.id)
    separator.addChild(transform_from_pose(co.pose))

    for (primitive, pose) in zip(co.primitives,
                                 co.primitive_poses):
        separator.addChild(coin_from_primitive(primitive, pose))

    for (mesh, pose) in zip(co.meshes, co.mesh_poses):
        separator.addChild(coin_from_mesh(mesh, pose))

    for (plane, pose) in zip(co.planes, co.plane_poses):
        separator.addChild(coin_from_plane(plane, pose, plane_sides_mm))

    for (subframe_name, pose) in zip(co.subframe_names, co.subframe_poses):
        separator.addChild(coin_from_subframe(subframe_name, pose,
                                              axis_length_mm=subframe_length_mm))
    return separator


def coin_from_primitive(
        primitive: SolidPrimitive,
        pose: Pose,
        ) -> coin.SoSeparator:
    """Convert a shape_msgs.msg.SolidPrimitive to Coin3D.

    Parameters:
    - primitive: Shape to represent.
    - pose: geometry_msgs.msg.Pose

    Returns:
    - SoTransform

    """
    def prism_not_implemented():
        raise NotImplementedError('PRISM not supported')

    fun_map: dict[int, Callable] = {
            SolidPrimitive.BOX: box_from_primitive,
            SolidPrimitive.SPHERE: sphere_from_primitive,
            SolidPrimitive.CYLINDER: cylinder_from_primitive,
            SolidPrimitive.CONE: cone_from_primitive,
            SolidPrimitive.PRISM: prism_not_implemented,
            }
    type_map: dict[int, str] = {
            SolidPrimitive.BOX: 'BOX',
            SolidPrimitive.SPHERE: 'SPHERE',
            SolidPrimitive.CYLINDER: 'CYLINDER',
            SolidPrimitive.CONE: 'CONE',
            SolidPrimitive.PRISM: 'PRISM',
            }
    if primitive.type not in fun_map.keys():
        raise ValueError('Unsupported SolidPrimitive type, got {primitive.type}')
    sep = coin.SoSeparator()
    sep.setName(f'{type_map[primitive.type]} at {str_from_pose(pose)}')
    sep.addChild(transform_from_pose(pose))
    shape = fun_map[primitive.type](primitive)
    sep.addChild(shape)
    return sep


def transform_from_pose(pose: Pose) -> coin.SoTransform:
    """Convert a geometry_msgs.msg.Pose to an SoTransform.

    Parameters:
    - pose: geometry_msgs.msg.Pose

    Returns:
    - SoTransform

    """
    so_transform = coin.SoTransform()

    # Extract position and orientation from the Pose message
    ros_to_coin_scale = 1000.0  # m -> mm.
    translation = (
            pose.position.x * ros_to_coin_scale,
            pose.position.y * ros_to_coin_scale,
            pose.position.z * ros_to_coin_scale,
            )
    r = pose.orientation
    rotation = coin.SbRotation(r.x, r.y, r.z, r.w)

    # Set the translation and rotation in the SoTransform
    so_transform.translation = translation
    so_transform.rotation = rotation

    return so_transform


def str_from_pose(pose: Pose) -> str:
    """Return a string representation of a geometry_msgs.msg.Pose.

    Parameters:
    - pose: geometry_msgs.msg.Pose to represent as a string.

    Returns:
    - str

    """
    return (f'({pose.position.x}, {pose.position.y}, {pose.position.z} m;'
            f'{pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})')


def box_from_primitive(primitive: SolidPrimitive) -> coin.SoSeparator:
    """Convert a shape_msgs.msg.SolidPrimitive with type=BOX to coin.

    Parameters:

        - solid_primitive: shape_msgs.msg.SolidPrimitive

    Returns:

        - coin.SoSeparator

    """
    if primitive.type != SolidPrimitive.BOX:
        raise ValueError('Unsupported SolidPrimitive type, got {primitive.type},'
                         ' expected {SolidPrimitive.BOX} (BOX)')

    sep = coin.SoSeparator()

    # Scaling node (meters -> millimeters, default SoCube = side 2 units).
    ros_to_coin_cube_scale = 1000.0 / 2.0
    scaling = coin.SoScale()
    scaling.scaleFactor = [
            primitive.dimensions[SolidPrimitive.BOX_X] * ros_to_coin_cube_scale,
            primitive.dimensions[SolidPrimitive.BOX_Y] * ros_to_coin_cube_scale,
            primitive.dimensions[SolidPrimitive.BOX_Z] * ros_to_coin_cube_scale,
            ]
    sep.addChild(scaling)

    cube = coin.SoCube()  # Default side length 2 units.
    sep.addChild(cube)

    return sep


def sphere_from_primitive(primitive: SolidPrimitive) -> coin.SoSeparator:
    """Convert a shape_msgs.msg.SolidPrimitive with type=SPHERE to coin.

    Parameters:

        - solid_primitive: shape_msgs.msg.SolidPrimitive

    Returns:

        - coin.SoSeparator

    """
    if primitive.type != SolidPrimitive.SPHERE:
        raise ValueError('Unsupported SolidPrimitive type, got {primitive.type},'
                         ' expected {SolidPrimitive.SPHERE} (SPHERE)')

    sep = coin.SoSeparator()

    # Scaling node (meters -> millimeters, default SoSphere = radius 1 unit).
    ros_to_coin_sphere_scale = 1000.0
    scaling = coin.SoScale()
    scaling.scaleFactor = [primitive.dimensions[SolidPrimitive.SPHERE_RADIUS] * ros_to_coin_sphere_scale] * 3
    sep.addChild(scaling)

    sphere = coin.SoSphere()  # Default radius 1 unit.
    sep.addChild(sphere)

    return sep


def cylinder_from_primitive(primitive: SolidPrimitive) -> coin.SoSeparator:
    """Convert a shape_msgs.msg.SolidPrimitive with type=CYLINDER to coin.

    Parameters:

        - solid_primitive: shape_msgs.msg.SolidPrimitive

    Returns:

        - coin.SoSeparator

    """
    if primitive.type != SolidPrimitive.CYLINDER:
        raise ValueError('Unsupported SolidPrimitive type, got {primitive.type},'
                         ' expected {SolidPrimitive.CYLINDER} (CYLINDER)')

    sep = coin.SoSeparator()

    # Rotation from Coin (along y) to ROS (along z).
    rotation = coin.SbRotation(0.7071067811865475, 0.0, 0.0, 0.7071067811865476)
    sep.addChild(rotation)

    # Scaling node (meters -> millimeters, default SoCylinder = radius 1 unit).
    ros_to_coin_cylinder_scale = 1000.0
    scaling = coin.SoScale()
    scaling.scaleFactor = [
            primitive.dimensions[SolidPrimitive.CYLINDER_RADIUS] * ros_to_coin_cylinder_scale,
            primitive.dimensions[SolidPrimitive.CYLINDER_HEIGHT] * ros_to_coin_cylinder_scale,
            primitive.dimensions[SolidPrimitive.CYLINDER_RADIUS] * ros_to_coin_cylinder_scale,
            ]
    sep.addChild(scaling)

    cylinder = coin.SoCylinder()  # Along y, radius 1, height 1 unit.
    # TODO: cylinder.radius = primitive.dimensions[SolidPrimitive.CYLINDER_RADIUS] * ros_to_coin_cylinder_scale and test
    # TODO: cylinder.height = primitive.dimensions[SolidPrimitive.CYLINDER_HEIGHT] * ros_to_coin_cylinder_scale and test
    sep.addChild(cylinder)

    return sep


def cone_from_primitive(primitive: SolidPrimitive) -> coin.SoSeparator:
    """Convert a shape_msgs.msg.SolidPrimitive with type=CONE to coin.

    Parameters:

        - solid_primitive: shape_msgs.msg.SolidPrimitive

    Returns:

        - coin.SoSeparator

    """
    if primitive.type != SolidPrimitive.CONE:
        raise ValueError('Unsupported SolidPrimitive type, got {primitive.type},'
                         ' expected {SolidPrimitive.CONE} (CONE)')

    sep = coin.SoSeparator()

    # Rotation from Coin (along y) to ROS (along z).
    rotation = coin.SoRotation()
    rotation.rotation = (0.7071067811865475, 0.0, 0.0, 0.7071067811865476)
    sep.addChild(rotation)

    # Scaling node (meters -> millimeters,
    # default SoCone = radius 1 unit, height = 2.0).
    ros_to_coin_cone_radius_scale = 1000.0
    ros_to_coin_cone_height_scale = 1000.0 / 2.0
    scaling = coin.SoScale()
    scaling.scaleFactor = [
            primitive.dimensions[SolidPrimitive.CONE_RADIUS] * ros_to_coin_cone_radius_scale,
            primitive.dimensions[SolidPrimitive.CONE_HEIGHT] * ros_to_coin_cone_height_scale,
            primitive.dimensions[SolidPrimitive.CONE_RADIUS] * ros_to_coin_cone_radius_scale,
            ]
    sep.addChild(scaling)

    cone = coin.SoCone()  # Pointing towards +y, radius 1, height 1 unit.
    sep.addChild(cone)

    return sep


def coin_from_mesh(mesh_msg: Mesh,
                   pose: Pose,
                   ) -> coin.SoSeparator:
    """Convert a shape_msgs.msg.Mesh to Coin3D nodes.

    Parameters:
    - mesh_msg: shape_msgs.msg.Mesh.
    - pose: Mesh pose.

    Returns:
    - coin.SoSeparator containing the Coin3D node representing the mesh

    """
    separator = coin.SoSeparator()
    separator.setName(f'Mesh at {str_from_pose(pose)}')
    separator.addChild(transform_from_pose(pose))
    separator.addChild(coin_mesh_from_shape_mesh(mesh_msg))
    return separator


def coin_mesh_from_shape_mesh(mesh_msg: Mesh) -> coin.SoSeparator:
    ros_to_coin_scale = 1000.0  # m (ROS) to mm (coin,FreeCAD).
    mesh = coin.SoSeparator()
    if mesh_msg.triangles:
        vertices = coin.SoCoordinate3()
        vertices.point.setNum(len(mesh_msg.vertices))
        for i, vertex in enumerate(mesh_msg.vertices):
            vertices.point.set1Value(i,
                                     vertex.x * ros_to_coin_scale,
                                     vertex.y * ros_to_coin_scale,
                                     vertex.z * ros_to_coin_scale)

        face_set = coin.SoIndexedFaceSet()
        for t, triangle in enumerate(mesh_msg.triangles):
            for i, index in enumerate(triangle.vertex_indices):
                face_set.coordIndex.set1Value(t * 4 + i, int(index))
            face_set.coordIndex.set1Value(t * 4 + 3, -1)  # Close the triangle.

        mesh.addChild(vertices)
        mesh.addChild(face_set)
    return mesh


def coin_from_plane(plane_msg: Plane,
                    pose: Pose,
                    side_mm: float,
                    ) -> coin.SoSeparator:
    """Convert a shape_msgs.msg.Plane to Coin3D.

    Parameters:
    - plane_msg: shape_msgs.msg.Plane Plane.
    - pose: geometry_msgs.msg.Pose Plane pose.
    - side_mm: length of the side of the plane, in millimeters.

    Returns:
    - coin.SoSeparator containing Coin3D nodes representing the plane

    """
    separator = coin.SoSeparator()
    separator.addChild(transform_from_pose(pose))

    if len(plane_msg.coef) != 4:
        raise RuntimeError('Malformed Plane message')

    a, b, c, d = plane_msg.coef

    # Calculate the normal vector.
    normal = coin.SbVec3f(a, b, c)
    normal.normalize()

    # Create a translation node to move the plane to its position.
    translation = coin.SoTranslation()
    translation.translation = (normal * -d).getValue()
    separator.addChild(translation)

    # Create a rotation node to orient the plane according to its normal.
    rotation = coin.SoRotation()
    rotation.rotation = coin.SbRotation(coin.SbVec3f(0, 0, 1), normal)  # Align with the z-axis
    separator.addChild(rotation)

    # Create a scaling node to represent the size of the plane.
    scale = coin.SoScale()
    scale.scaleFactor = (side_mm, side_mm, side_mm)
    separator.addChild(scale)

    vertices = coin.SoCoordinate3()
    vertices.point.setNum(4)
    for i, vertex in enumerate((
            (0.5, 0.5, 0.0),
            (-0.5, 0.5, 0.0),
            (0.5, -0.5, 0.0),
            (-0.5, -0.5, 0.0),
            )):
        vertices.point.set1Value(i, vertex[0], vertex[1], vertex[2])

    quad_mesh = coin.SoQuadMesh()
    quad_mesh.vertexProperty = vertices
    quad_mesh.verticesPerColumn = 2
    quad_mesh.verticesPerRow = 2
    separator.addChild(quad_mesh)

    return separator


def coin_from_subframe(subframe_name: str,
                       pose: Pose,
                       axis_length_mm: [float | fc.Units.Quantity] = 100.0,
                       ) -> coin.SoSeparator:
    """Return a SoSeparator represent a frame.

    Parameters:
    - subfname_name: Subframe name.
    - pose: Subframe pose relative to the collision
            object's center.
    - side_mm: length of the axes of the subframe, in millimeters.

    Returns:
    - coin.SoSeparator containing Coin3D nodes representing the subframe.

    """
    separator = coin.SoSeparator()
    separator.setName(subframe_name)
    separator.addChild(transform_from_pose(pose))
    separator.addChild(frame_group(length_mm=axis_length_mm))
    return separator
