#!/usr/bin/env python3
"""Convert MuJoCo MJCF models to URDF for the models library.

The `mujoco` Python package does not ship an `mjcf_to_urdf` export function,
so this module provides a self-contained MJCF -> URDF converter covering the
features used by the `mujoco_menagerie` models: bodies, joints, geoms, meshes,
inertials, default classes, includes, actuators and joint equalities.

The generated URDF is cached in ``~/.cache/robot_descriptions/mjcf_to_urdf/``
and can be processed by the standard URDF import pipeline
(:func:`freecad.cross.robot_from_urdf.robot_from_urdf_path`).
"""

import hashlib
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Rotation / pose helpers
# ---------------------------------------------------------------------------


def _quat_mul(q1, q2):
    """Multiply two quaternions (w, x, y, z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def _quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _mat_to_quat(m):
    tr = np.trace(m)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z])


def _mat_to_rpy(m):
    """Rotation matrix to URDF fixed-axis XYZ rpy (radians)."""
    sy = np.sqrt(m[0, 0] ** 2 + m[1, 0] ** 2)
    if sy > 1e-6:
        roll = np.arctan2(m[2, 1], m[2, 2])
        pitch = np.arctan2(-m[2, 0], sy)
        yaw = np.arctan2(m[1, 0], m[0, 0])
    else:
        roll = np.arctan2(-m[1, 2], m[1, 1])
        pitch = np.arctan2(-m[2, 0], sy)
        yaw = 0.0
    return np.array([roll, pitch, yaw])


def _rot_axis(axis, angle):
    c, s = np.cos(angle), np.sin(angle)
    if axis == 0:
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 1:
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _euler_to_quat(euler, seq='xyz', degrees=False):
    if degrees:
        euler = np.radians(euler)
    axes = {'x': 0, 'y': 1, 'z': 2}
    R = np.eye(3)
    for axis_char, angle in zip(seq, euler):
        R = R @ _rot_axis(axes[axis_char], angle)
    return _mat_to_quat(R)


def _axisangle_to_quat(axis, angle, degrees=False):
    if degrees:
        angle = np.radians(angle)
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / norm
    half = angle / 2.0
    s = np.sin(half)
    return np.array([np.cos(half), axis[0] * s, axis[1] * s, axis[2] * s])


def _zaxis_to_quat(zaxis):
    z = np.asarray(zaxis, dtype=float)
    norm = np.linalg.norm(z)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    z = z / norm
    z0 = np.array([0.0, 0.0, 1.0])
    v = np.cross(z0, z)
    c = np.dot(z0, z)
    if np.linalg.norm(v) < 1e-12:
        if c > 0:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return np.array([0.0, 1.0, 0.0, 0.0])
    q = np.array([1.0 + c, v[0], v[1], v[2]])
    return q / np.linalg.norm(q)


def _pose_compose(p1, p2):
    pos1, quat1 = p1
    pos2, quat2 = p2
    pos = pos1 + _quat_to_mat(quat1) @ pos2
    quat = _quat_mul(quat1, quat2)
    return pos, quat


def _pose_inverse(p):
    pos, quat = p
    inv_quat = _quat_conj(quat)
    inv_pos = -(_quat_to_mat(inv_quat) @ pos)
    return inv_pos, inv_quat


def _parse_vec(text, n):
    if not text:
        return np.zeros(n)
    vals = [float(x) for x in str(text).split()]
    if len(vals) == 1 and n > 1:
        vals = vals * n
    while len(vals) < n:
        vals.append(0.0)
    return np.array(vals[:n])


def _fmt_vec(v):
    return ' '.join(f'{x:.9g}' for x in np.asarray(v, dtype=float).flatten())


# ---------------------------------------------------------------------------
# MJCF parsing
# ---------------------------------------------------------------------------


def _resolve_includes(elem, base_dir):
    """Replace <include> elements with the content of the included files."""
    for inc in list(elem.findall('include')):
        file_attr = inc.get('file')
        if not file_attr:
            elem.remove(inc)
            continue
        inc_path = Path(base_dir) / file_attr
        if not inc_path.exists():
            elem.remove(inc)
            continue
        try:
            inc_root = ET.parse(str(inc_path)).getroot()
        except ET.ParseError:
            elem.remove(inc)
            continue
        _resolve_includes(inc_root, inc_path.parent)
        idx = list(elem).index(inc)
        elem.remove(inc)
        for child in list(inc_root):
            elem.insert(idx, child)
            idx += 1


def _parse_compiler(root):
    compiler = root.find('compiler')
    attrs = dict(compiler.attrib) if compiler is not None else {}
    return {
        'angle': attrs.get('angle', 'degree'),
        'coordinate': attrs.get('coordinate', 'local'),
        'meshdir': attrs.get('meshdir', ''),
        'eulerseq': attrs.get('eulerseq', 'xyz'),
        'autolimits': attrs.get('autolimits', 'false'),
    }


def _parse_defaults(root):
    """Return dict class_name -> {'joint': {}, 'geom': {}, 'general': {}}."""
    classes = {'': {'joint': {}, 'geom': {}, 'general': {}}}

    def walk(elem, inherited):
        cls = elem.get('class', '')
        merged = {
            'joint': dict(inherited['joint']),
            'geom': dict(inherited['geom']),
            'general': dict(inherited['general']),
        }
        for child in elem:
            if child.tag == 'default':
                walk(child, merged)
            elif child.tag in merged:
                merged[child.tag].update(child.attrib)
        if cls:
            classes[cls] = merged
        else:
            for tag in merged:
                classes[''][tag].update(merged[tag])

    for d in root.findall('default'):
        walk(d, classes[''])
    return classes


def _merged_attrs(elem, defaults, tag):
    """Merge default-class attributes with the element's own attributes."""
    cls = elem.get('class', '')
    d = defaults.get(cls, defaults[''])
    attrs = dict(d[tag])
    attrs.update(elem.attrib)
    return attrs


def _parse_pose(elem, compiler):
    """Return (pos, quat) from a body/geom/inertial element."""
    pos = _parse_vec(elem.get('pos', '0 0 0'), 3)
    if 'quat' in elem.attrib:
        quat = _parse_vec(elem.get('quat'), 4)
    elif 'euler' in elem.attrib:
        euler = _parse_vec(elem.get('euler'), 3)
        quat = _euler_to_quat(
            euler, compiler['eulerseq'], degrees=(compiler['angle'] == 'degree'))
    elif 'axisangle' in elem.attrib:
        aa = _parse_vec(elem.get('axisangle'), 4)
        quat = _axisangle_to_quat(
            aa[:3], aa[3], degrees=(compiler['angle'] == 'degree'))
    elif 'zaxis' in elem.attrib:
        quat = _zaxis_to_quat(_parse_vec(elem.get('zaxis'), 3))
    else:
        quat = np.array([1.0, 0.0, 0.0, 0.0])
    return pos, quat


def _parse_assets(root, mjcf_dir, compiler):
    """Return dict mesh_name -> {'path': Path, 'scale': np.ndarray}."""
    assets = {}
    meshdir = compiler['meshdir']
    if meshdir and not os.path.isabs(meshdir):
        meshdir = str(Path(mjcf_dir) / meshdir)
    for mesh in root.findall('.//asset/mesh'):
        name = mesh.get('name')
        file_attr = mesh.get('file')
        if not name or not file_attr:
            continue
        if meshdir:
            mesh_path = Path(meshdir) / file_attr
        else:
            mesh_path = Path(mjcf_dir) / file_attr
        if not mesh_path.exists():
            mesh_path = Path(mjcf_dir) / file_attr
        scale = _parse_vec(mesh.get('scale', '1 1 1'), 3)
        assets[name] = {'path': mesh_path, 'scale': scale}
    return assets


def _parse_actuators(root):
    """Return dict joint_name -> {'effort': float, 'ctrlrange': list}."""
    actuators = {}
    for act in root.findall('.//actuator/*'):
        joint_attr = act.get('joint')
        if not joint_attr:
            continue
        info = {'effort': None, 'ctrlrange': None}
        forcerange = act.get('forcerange')
        if forcerange:
            fr = _parse_vec(forcerange, 2)
            info['effort'] = max(abs(fr[0]), abs(fr[1]))
        ctrlrange = act.get('ctrlrange')
        if ctrlrange:
            info['ctrlrange'] = _parse_vec(ctrlrange, 2)
        for joint_name in joint_attr.split():
            actuators[joint_name] = info
    return actuators


def _parse_equalities(root):
    """Return dict child_joint -> {'joint': parent, 'multiplier': float, 'offset': float}."""
    mimics = {}
    for eq in root.findall('.//equality/joint'):
        pair = eq.get('pair', '')
        parts = pair.split()
        if len(parts) != 2:
            continue
        parent, child = parts[0], parts[1]
        polycoef = _parse_vec(eq.get('polycoef', '0 1'), 2)
        mimics[child] = {
            'joint': parent,
            'multiplier': polycoef[1],
            'offset': polycoef[0],
        }
    return mimics


# ---------------------------------------------------------------------------
# Geometry helpers (volume / inertia of primitives)
# ---------------------------------------------------------------------------


def _geom_volume(geom_type, size):
    if geom_type == 'box':
        return 8.0 * size[0] * size[1] * size[2]
    if geom_type == 'sphere':
        return 4.0 / 3.0 * np.pi * size[0] ** 3
    if geom_type in ('cylinder', 'capsule'):
        return np.pi * size[0] ** 2 * 2.0 * size[1]
    if geom_type == 'ellipsoid':
        return 4.0 / 3.0 * np.pi * size[0] * size[1] * size[2]
    return 0.0


def _geom_inertia(geom_type, size, mass):
    if geom_type == 'box':
        hx, hy, hz = size
        return mass / 3.0 * np.diag(
            [hy ** 2 + hz ** 2, hx ** 2 + hz ** 2, hx ** 2 + hy ** 2])
    if geom_type == 'sphere':
        r = size[0]
        return 2.0 / 5.0 * mass * r ** 2 * np.eye(3)
    if geom_type in ('cylinder', 'capsule'):
        r, hl = size[0], size[1]
        h = 2.0 * hl
        return mass * np.diag(
            [(3 * r ** 2 + h ** 2) / 12.0, (3 * r ** 2 + h ** 2) / 12.0, r ** 2 / 2.0])
    if geom_type == 'ellipsoid':
        rx, ry, rz = size
        return mass / 5.0 * np.diag(
            [ry ** 2 + rz ** 2, rx ** 2 + rz ** 2, rx ** 2 + ry ** 2])
    return np.zeros((3, 3))


# ---------------------------------------------------------------------------
# Mesh handling
# ---------------------------------------------------------------------------


def _ensure_stl(mesh_path, output_dir):
    """Convert a mesh to STL (in meters) if it is not already STL/OBJ.

    The URDF import pipeline scales STL/OBJ meshes by 1000 (m -> mm), so
    referencing the original STL/OBJ files directly is correct for MJCF
    (which uses meters). Other formats are converted to STL so the scale
    handling stays consistent.
    """
    mesh_path = Path(mesh_path)
    suffix = mesh_path.suffix.lower()
    if suffix in ('.stl', '.obj'):
        return mesh_path
    if output_dir is None:
        return mesh_path
    out_path = Path(output_dir) / 'meshes' / (mesh_path.stem + '.stl')
    if out_path.exists():
        return out_path
    try:
        import Mesh as fcmesh
        mesh = fcmesh.read(str(mesh_path))
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fcmesh.export([mesh], str(out_path))
        return out_path
    except Exception:
        return mesh_path


def _mesh_urdf_path(mesh_path, package_path, repository_path):
    """Return a URDF mesh filename for the given absolute mesh path."""
    mesh_path = Path(mesh_path)
    if package_path:
        pkg = Path(package_path)
        try:
            rel = mesh_path.relative_to(pkg)
            return f"package://{pkg.name}/{rel.as_posix()}"
        except ValueError:
            pass
    if repository_path:
        repo = Path(repository_path)
        try:
            rel = mesh_path.relative_to(repo)
            return f"package://{repo.name}/{rel.as_posix()}"
        except ValueError:
            pass
    return f"file://{mesh_path.as_posix()}"


# ---------------------------------------------------------------------------
# URDF building
# ---------------------------------------------------------------------------


def _add_inertial(link, inertial_elem, compiler, frame_offset=None):
    pos, quat = _parse_pose(inertial_elem, compiler)
    if frame_offset is not None:
        pos, quat = _pose_compose(frame_offset, (pos, quat))
    mass = float(inertial_elem.get('mass', '0'))
    diaginertia = inertial_elem.get('diaginertia')
    fullinertia = inertial_elem.get('fullinertia')
    if diaginertia:
        ixx, iyy, izz = _parse_vec(diaginertia, 3)
        ixy = ixz = iyz = 0.0
    elif fullinertia:
        ixx, ixy, ixz, iyy, iyz, izz = _parse_vec(fullinertia, 6)
    else:
        ixx = iyy = izz = 0.0
        ixy = ixz = iyz = 0.0

    inertial = ET.SubElement(link, 'inertial')
    origin = ET.SubElement(inertial, 'origin')
    origin.set('xyz', _fmt_vec(pos))
    origin.set('rpy', _fmt_vec(_mat_to_rpy(_quat_to_mat(quat))))
    ET.SubElement(inertial, 'mass', {'value': f'{mass:.9g}'})
    ET.SubElement(inertial, 'inertia', {
        'ixx': f'{ixx:.9g}', 'ixy': f'{ixy:.9g}', 'ixz': f'{ixz:.9g}',
        'iyy': f'{iyy:.9g}', 'iyz': f'{iyz:.9g}', 'izz': f'{izz:.9g}',
    })


def _add_inertial_from_geoms(link, body_elem, compiler, defaults, context, frame_offset=None):
    """Compute an inertial element from the body's geoms (mass/density)."""
    total_mass = 0.0
    com = np.zeros(3)
    inertia_about_origin = np.zeros((3, 3))
    I_about_origin = inertia_about_origin
    for geom in body_elem.findall('geom'):
        attrs = _merged_attrs(geom, defaults, 'geom')
        geom_type = attrs.get('type', 'sphere')
        if geom_type in ('plane', 'hfield'):
            continue
        size = _parse_vec(attrs.get('size', ''), 3) if attrs.get('size') else None
        if size is None:
            continue
        pos, quat = _parse_pose(geom, compiler)
        if frame_offset is not None:
            pos, quat = _pose_compose(frame_offset, (pos, quat))
        mass_attr = attrs.get('mass')
        density_attr = attrs.get('density')
        if mass_attr:
            mass = float(mass_attr)
        elif density_attr:
            mass = _geom_volume(geom_type, size) * float(density_attr)
        else:
            continue
        if mass <= 0:
            continue
        I_local = _geom_inertia(geom_type, size, mass)
        R = _quat_to_mat(quat)
        I_rot = R @ I_local @ R.T
        r = pos
        I_about_origin += I_rot + mass * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
        total_mass += mass
        com += mass * pos

    if total_mass <= 0:
        return
    com /= total_mass
    I_com = inertia_about_origin - total_mass * (
        np.dot(com, com) * np.eye(3) - np.outer(com, com))

    inertial = ET.SubElement(link, 'inertial')
    origin = ET.SubElement(inertial, 'origin')
    origin.set('xyz', _fmt_vec(com))
    origin.set('rpy', '0 0 0')
    ET.SubElement(inertial, 'mass', {'value': f'{total_mass:.9g}'})
    ET.SubElement(inertial, 'inertia', {
        'ixx': f'{I_com[0, 0]:.9g}', 'ixy': f'{I_com[0, 1]:.9g}',
        'ixz': f'{I_com[0, 2]:.9g}', 'iyy': f'{I_com[1, 1]:.9g}',
        'iyz': f'{I_com[1, 2]:.9g}', 'izz': f'{I_com[2, 2]:.9g}',
    })


def _add_geom(link, geom_elem, compiler, defaults, context, frame_offset=None):
    attrs = _merged_attrs(geom_elem, defaults, 'geom')
    geom_type = attrs.get('type', 'sphere')
    geom_name = attrs.get('name', '')
    pos, quat = _parse_pose(geom_elem, compiler)
    if frame_offset is not None:
        pos, quat = _pose_compose(frame_offset, (pos, quat))
    size = _parse_vec(attrs.get('size', ''), 3) if attrs.get('size') else None
    rgba = _parse_vec(attrs.get('rgba', ''), 4) if attrs.get('rgba') else None

    # Skip infinite planes and heightfields (terrain).
    if geom_type in ('plane', 'hfield'):
        return

    geom_xml = None
    if geom_type == 'box':
        if size is None:
            return
        geom_xml = ET.SubElement(link, 'visual')
        if geom_name:
            geom_xml.set('name', geom_name)
        origin = ET.SubElement(geom_xml, 'origin')
        origin.set('xyz', _fmt_vec(pos))
        origin.set('rpy', _fmt_vec(_mat_to_rpy(_quat_to_mat(quat))))
        geometry = ET.SubElement(geom_xml, 'geometry')
        box = ET.SubElement(geometry, 'box')
        box.set('size', _fmt_vec(size * 2.0))
    elif geom_type == 'sphere':
        if size is None:
            return
        geom_xml = ET.SubElement(link, 'visual')
        if geom_name:
            geom_xml.set('name', geom_name)
        origin = ET.SubElement(geom_xml, 'origin')
        origin.set('xyz', _fmt_vec(pos))
        origin.set('rpy', _fmt_vec(_mat_to_rpy(_quat_to_mat(quat))))
        geometry = ET.SubElement(geom_xml, 'geometry')
        sphere = ET.SubElement(geometry, 'sphere')
        sphere.set('radius', f'{size[0]:.9g}')
    elif geom_type in ('cylinder', 'capsule'):
        if size is None:
            return
        geom_xml = ET.SubElement(link, 'visual')
        if geom_name:
            geom_xml.set('name', geom_name)
        origin = ET.SubElement(geom_xml, 'origin')
        origin.set('xyz', _fmt_vec(pos))
        origin.set('rpy', _fmt_vec(_mat_to_rpy(_quat_to_mat(quat))))
        geometry = ET.SubElement(geom_xml, 'geometry')
        cylinder = ET.SubElement(geometry, 'cylinder')
        cylinder.set('radius', f'{size[0]:.9g}')
        cylinder.set('length', f'{2.0 * size[1]:.9g}')
    elif geom_type == 'ellipsoid':
        if size is None:
            return
        geom_xml = ET.SubElement(link, 'visual')
        if geom_name:
            geom_xml.set('name', geom_name)
        origin = ET.SubElement(geom_xml, 'origin')
        origin.set('xyz', _fmt_vec(pos))
        origin.set('rpy', _fmt_vec(_mat_to_rpy(_quat_to_mat(quat))))
        geometry = ET.SubElement(geom_xml, 'geometry')
        sphere = ET.SubElement(geometry, 'sphere')
        sphere.set('radius', f'{max(size):.9g}')
    elif geom_type == 'mesh':
        mesh_name = attrs.get('mesh')
        if not mesh_name or mesh_name not in context['assets']:
            return
        asset = context['assets'][mesh_name]
        mesh_path = _ensure_stl(asset['path'], context['output_dir'])
        urdf_path = _mesh_urdf_path(
            mesh_path, context['package_path'], context['repository_path'])
        geom_xml = ET.SubElement(link, 'visual')
        if geom_name:
            geom_xml.set('name', geom_name)
        origin = ET.SubElement(geom_xml, 'origin')
        origin.set('xyz', _fmt_vec(pos))
        origin.set('rpy', _fmt_vec(_mat_to_rpy(_quat_to_mat(quat))))
        geometry = ET.SubElement(geom_xml, 'geometry')
        mesh = ET.SubElement(geometry, 'mesh')
        mesh.set('filename', urdf_path)
        scale = asset['scale'] * _parse_vec(attrs.get('scale', '1 1 1'), 3)
        mesh.set('scale', _fmt_vec(scale))
    else:
        return

    if rgba is not None:
        material = ET.SubElement(geom_xml, 'material')
        material.set('name', f'mat_{geom_name or "geom"}')
        color = ET.SubElement(material, 'color')
        color.set('rgba', _fmt_vec(rgba))


def _add_joint(robot, joint_elem, compiler, defaults, context, origin_pose=None):
    attrs = _merged_attrs(joint_elem, defaults, 'joint')
    joint_name = attrs.get('name', '')
    if not joint_name:
        return
    joint_type = attrs.get('type', 'hinge')
    parent = attrs.get('parent', '')
    child = attrs.get('child', '')
    if not parent or not child:
        return

    # Map MJCF joint types to URDF.
    if joint_type == 'hinge':
        urdf_type = 'revolute'
    elif joint_type == 'slide':
        urdf_type = 'prismatic'
    elif joint_type == 'ball':
        urdf_type = 'floating'
    elif joint_type == 'free':
        urdf_type = 'floating'
    else:
        urdf_type = 'fixed'

    joint = ET.SubElement(robot, 'joint')
    joint.set('name', joint_name)
    joint.set('type', urdf_type)
    ET.SubElement(joint, 'parent', {'link': parent})
    ET.SubElement(joint, 'child', {'link': child})

    if origin_pose is None:
        pos, quat = _parse_pose(joint_elem, compiler)
    else:
        pos, quat = origin_pose
    origin = ET.SubElement(joint, 'origin')
    origin.set('xyz', _fmt_vec(pos))
    origin.set('rpy', _fmt_vec(_mat_to_rpy(_quat_to_mat(quat))))

    if urdf_type in ('revolute', 'prismatic'):
        axis = _parse_vec(attrs.get('axis', '0 0 1'), 3)
        norm = np.linalg.norm(axis)
        if norm < 1e-12:
            axis = np.array([0.0, 0.0, 1.0])
        else:
            axis = axis / norm
        ET.SubElement(joint, 'axis', {'xyz': _fmt_vec(axis)})

        limit = ET.SubElement(joint, 'limit')
        range_attr = attrs.get('range')
        if range_attr:
            lo, hi = _parse_vec(range_attr, 2)
            if compiler['angle'] == 'degree' and urdf_type == 'revolute':
                lo, hi = np.radians(lo), np.radians(hi)
            limit.set('lower', f'{lo:.9g}')
            limit.set('upper', f'{hi:.9g}')
        else:
            limit.set('lower', '-3.141592653589793')
            limit.set('upper', '3.141592653589793')
        effort = None
        act = context['actuators'].get(joint_name)
        if act and act['effort'] is not None:
            effort = act['effort']
        if effort is None:
            effort = 1000.0
        limit.set('effort', f'{effort:.9g}')
        limit.set('velocity', '1000')

    mimic = context['mimics'].get(joint_name)
    if mimic:
        mimic_xml = ET.SubElement(joint, 'mimic')
        mimic_xml.set('joint', mimic['joint'])
        mimic_xml.set('multiplier', f'{mimic["multiplier"]:.9g}')
        mimic_xml.set('offset', f'{mimic["offset"]:.9g}')


def _add_body(robot, body_elem, compiler, defaults, context,
              parent_body_global, parent_link_global, parent_link_name):
    """Add a body (and its subtree) to the URDF.

    Args:
        parent_body_global: Global pose of the parent body frame.
        parent_link_global: Global pose of the parent link (joint) frame.
            The URDF joint origin is expressed relative to this frame.
        parent_link_name: Name of the parent URDF link.
    """
    body_name = body_elem.get('name', '')
    if not body_name:
        body_name = f'body_{len(context["links"])}'
    body_pose = _parse_pose(body_elem, compiler)
    body_global = _pose_compose(parent_body_global, body_pose)

    joints = body_elem.findall('joint')
    n_joints = len(joints)

    if n_joints == 0:
        # No explicit joint: the link frame is the body frame and the body
        # is connected to its parent by a fixed joint.
        link_global = body_global
        frame_offset = None
    else:
        # The URDF link frame is the frame of the LAST joint (the one whose
        # child is the body link). Geoms/inertials, expressed in the body
        # frame, are transformed into this link frame.
        last_joint_pose = _parse_pose(joints[-1], compiler)
        link_global = _pose_compose(body_global, last_joint_pose)
        frame_offset = _pose_inverse(last_joint_pose)

    link = ET.SubElement(robot, 'link')
    link.set('name', body_name)
    context['links'].append(body_name)

    # Inertial: explicit <inertial> or computed from geoms.
    inertial_elem = body_elem.find('inertial')
    if inertial_elem is not None:
        _add_inertial(link, inertial_elem, compiler, frame_offset)
    else:
        _add_inertial_from_geoms(link, body_elem, compiler, defaults, context, frame_offset)

    # Visual geoms.
    for geom in body_elem.findall('geom'):
        _add_geom(link, geom, compiler, defaults, context, frame_offset)

    # Build the joint chain: parent -> int_0 -> ... -> int_{n-2} -> body.
    # Multiple joints on a body (parallel joints in MJCF) are represented
    # with intermediate massless links, because each URDF link has exactly
    # one parent joint.
    prev_link_name = parent_link_name
    prev_link_global = parent_link_global
    for i, joint_elem in enumerate(joints):
        attrs = _merged_attrs(joint_elem, defaults, 'joint')
        joint_name = attrs.get('name', '')
        if not joint_name:
            continue
        joint_pose = _parse_pose(joint_elem, compiler)
        joint_global = _pose_compose(body_global, joint_pose)
        is_last = (i == n_joints - 1)
        if is_last:
            child_name = body_name
        else:
            child_name = f'{body_name}_joint{i}'
            intermediate = ET.SubElement(robot, 'link')
            intermediate.set('name', child_name)
            context['links'].append(child_name)
        joint_origin = _pose_compose(
            _pose_inverse(prev_link_global), joint_global)
        joint_elem.set('parent', prev_link_name)
        joint_elem.set('child', child_name)
        _add_joint(robot, joint_elem, compiler, defaults, context, joint_origin)
        prev_link_name = child_name
        prev_link_global = joint_global

    if n_joints == 0:
        # Fixed joint connecting the body to its parent.
        joint_origin = _pose_compose(
            _pose_inverse(parent_link_global), link_global)
        fixed = ET.SubElement(robot, 'joint')
        fixed.set('name', f'{body_name}_fixed')
        fixed.set('type', 'fixed')
        ET.SubElement(fixed, 'parent', {'link': parent_link_name})
        ET.SubElement(fixed, 'child', {'link': body_name})
        origin = ET.SubElement(fixed, 'origin')
        origin.set('xyz', _fmt_vec(joint_origin[0]))
        origin.set('rpy', _fmt_vec(_mat_to_rpy(_quat_to_mat(joint_origin[1]))))

    # Child bodies.
    for child in body_elem.findall('body'):
        _add_body(robot, child, compiler, defaults, context,
                  body_global, link_global, body_name)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def _cache_dir():
    cache_root = os.path.expanduser(
        os.environ.get(
            "ROBOT_DESCRIPTIONS_CACHE",
            "~/.cache/robot_descriptions",
        )
    )
    return os.path.join(cache_root, "mjcf_to_urdf")


def get_urdf_path(
    mjcf_path,
    package_path=None,
    repository_path=None,
    output_dir=None,
):
    """Convert an MJCF file to URDF and return the path to the URDF file.

    Args:
        mjcf_path: Path to the MJCF file.
        package_path: Optional package path (used to resolve mesh paths).
        repository_path: Optional repository path (used to resolve mesh paths).
        output_dir: Optional output directory for the generated URDF and
            converted meshes. Defaults to a cache directory keyed by the
            MJCF file content.

    Returns:
        Path to the generated URDF file.
    """
    mjcf_path = Path(mjcf_path)
    if not mjcf_path.exists():
        raise FileNotFoundError(f'MJCF file not found: {mjcf_path}')

    # Parse the MJCF file.
    tree = ET.parse(str(mjcf_path))
    root = tree.getroot()
    if root.tag != 'mujoco':
        raise ValueError(f'Not an MJCF file: {mjcf_path}')

    _resolve_includes(root, mjcf_path.parent)
    compiler = _parse_compiler(root)
    defaults = _parse_defaults(root)
    assets = _parse_assets(root, mjcf_path.parent, compiler)
    actuators = _parse_actuators(root)
    mimics = _parse_equalities(root)

    # Determine output directory.
    if output_dir is None:
        with open(str(mjcf_path), 'rb') as f:
            content_hash = hashlib.sha256(f.read()).hexdigest()[:16]
        output_dir = os.path.join(_cache_dir(), content_hash)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    context = {
        'assets': assets,
        'actuators': actuators,
        'mimics': mimics,
        'package_path': package_path,
        'repository_path': repository_path,
        'output_dir': output_dir,
        'links': [],
    }

    robot = ET.Element('robot')
    robot.set('name', mjcf_path.stem)

    # World body: add its geoms to a "base_link" and process child bodies.
    world_body = root.find('worldbody')
    if world_body is not None:
        base_link = ET.SubElement(robot, 'link')
        base_link.set('name', 'base_link')
        context['links'].append('base_link')
        for geom in world_body.findall('geom'):
            _add_geom(base_link, geom, compiler, defaults, context)
        identity = (np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
        for child in world_body.findall('body'):
            _add_body(robot, child, compiler, defaults, context,
                      identity, identity, 'base_link')

    # Serialize.
    urdf_path = output_dir / f'{mjcf_path.stem}.urdf'
    ET.indent(robot, space='  ')
    tree_out = ET.ElementTree(robot)
    tree_out.write(str(urdf_path), encoding='utf-8', xml_declaration=True)

    return str(urdf_path)