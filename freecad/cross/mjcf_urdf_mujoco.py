#!/usr/bin/env python3
"""Alternative MJCF -> URDF converter based on the ``mujoco`` Python bindings.

This is a self-contained re-implementation inspired by
``mjcf_urdf_simple_converter`` (https://github.com/Yasu31/mjcf_urdf_simple_converter).
It loads the MJCF with :mod:`mujoco` (which fully resolves ``default`` /
``childclass`` inheritance, ``<include>``, implicit mesh names and ``meshdir``)
and emits a URDF directly from the compiled :class:`mujoco.MjModel`.

Compared to the original snippet, the following functionality was added so the
output is usable for normal robot workflows (visualisation, kinematics,
collision checking):

* a ``world`` root link is always emitted so the MJCF ``world`` frame is
  preserved (the URDF root frame becomes the MJCF world frame);
* ``slide`` (prismatic), ``ball`` and ``free`` joints in addition to ``hinge``
  (``ball``/``free`` are approximated by a ``floating`` joint);
* primitive geoms (``box``, ``sphere``, ``cylinder``, ``capsule``,
  ``ellipsoid``, ``plane``) in addition to ``mesh`` geoms;
* separate ``<visual>`` and ``<collision>`` elements, classified from the
  MuJoCo geom ``contype`` / ``conaffinity`` / ``group`` fields;
* per-visual ``<material>`` colours taken from the MuJoCo material / geom RGBA;
* correct handling of mesh-referencing geoms (MuJoCo bakes the asset ``scale``
  and centring offset into ``geom_pos`` / ``geom_quat``, so the raw
  ``mesh_vert`` buffer can be exported as-is);
* ``autolimits``-aware joint limits (unlimited hinges become ``continuous``)
  with sane ``effort`` / ``velocity`` defaults;
* correct multi-joint-per-body chain conversion (the intermediate frames are
  derived from each joint's position, fixing an off-by-one accumulation bug in
  the original snippet). Bodies with a single joint map directly onto one URDF
  joint (no auxiliary link), while multi-joint bodies use intermediate dummy
  links only when needed.

The module exposes a single public entry point:
:func:`convert_mjcf_to_urdf`.
"""

import os
import os.path as osp
import xml.etree.ElementTree as ET
from xml.dom import minidom

import numpy as np

# The ``mujoco`` module is imported lazily (see ``convert_mjcf_to_urdf``) so
# that the caller can register ``AdditionalPythonPackages`` on ``sys.path``
# first. Sub-functions reference this module-level name.
mujoco = None

# Conversion output version, used for the cache key of the caller.
CONVERTER_VERSION = '1'

# Name of the emitted root link (the MJCF ``world`` frame).
_WORLD_LINK = 'world'

# Default effort/velocity for joints whose MJCF does not define them.
_DEFAULT_EFFORT = '1000'
_DEFAULT_VELOCITY = '1000'


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------


def _array2str(arr):
    """Format a numpy array as a space-separated string of floats."""
    return ' '.join('%.9g' % float(x) for x in np.asarray(arr).reshape(-1))


def _quat_to_rpy(quat):
    """Return URDF roll-pitch-yaw for a w-x-y-z quaternion."""
    w, x, y, z = (float(v) for v in quat)
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r10 = 2.0 * (x * y + w * z)
    r20 = 2.0 * (x * z - w * y)
    r21 = 2.0 * (y * z + w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    sy = float(np.sqrt(r00 * r00 + r10 * r10))
    if sy > 1e-8:
        roll = float(np.arctan2(r21, r22))
        pitch = float(np.arctan2(-r20, sy))
        yaw = float(np.arctan2(r10, r00))
    else:
        roll = float(np.arctan2(-r21, r22))
        pitch = float(np.arctan2(-r20, sy))
        yaw = 0.0
    return np.array([roll, pitch, yaw])


def _quat_rotate(quat, vec):
    """Rotate a 3-vector by a w-x-y-z quaternion."""
    w, x, y, z = (float(v) for v in quat)
    qv = np.array([x, y, z], dtype=float)
    v = np.asarray(vec, dtype=float)
    # v' = v + 2 w (q x v) + 2 q x (q x v)
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def _sanitize(name):
    return ''.join(c if (c.isalnum() or c in '-_.') else '_' for c in name) or 'item'


def _name(model, obj_type, index):
    name = mujoco.mj_id2name(model, obj_type, index)
    return name if name else None


# ---------------------------------------------------------------------------
# OBJ export
# ---------------------------------------------------------------------------


def _export_obj(model, mesh_id, path):
    """Write a mesh (already centred and scaled by MuJoCo) to an OBJ file."""
    va = int(model.mesh_vertadr[mesh_id])
    vn = int(model.mesh_vertnum[mesh_id])
    verts = model.mesh_vert[va:va + vn]
    fa = int(model.mesh_faceadr[mesh_id])
    fn = int(model.mesh_facenum[mesh_id])
    faces = model.mesh_face[fa:fa + fn]
    name = _name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id) or 'mesh'
    with open(path, 'w') as f:
        f.write('o %s\n' % _sanitize(name))
        for v in verts:
            f.write('v %.9g %.9g %.9g\n' % (float(v[0]), float(v[1]), float(v[2])))
        for face in faces:
            f.write('f %d %d %d\n' % (
                int(face[0]) + 1, int(face[1]) + 1, int(face[2]) + 1))


# ---------------------------------------------------------------------------
# Geom classification
# ---------------------------------------------------------------------------


def _geom_is_collision(model, geom_id):
    """Classify a geom as collision (as opposed to visual).

    MuJoCo has no dedicated visual/collision flag, so several signals are
    combined (in priority order):

    1. ``group == 2`` is visual and ``group == 3`` is collision (the MuJoCo
       Menagerie convention, used by go1/go2/panda/ur10e/h1/... and also by
       models such as Apptronik Apollo that additionally set
       ``contype = conaffinity = 0`` on *both* kinds, so the contact flags
       alone cannot distinguish them);
    2. the geom name (``*collision*`` / ``*_col`` vs ``*visual*``) — needed for
       models such as Shadow DexEE whose collision *estimate* primitives use
       ``group = 5`` with ``contype = conaffinity = 0`` but are named
       ``*CollisionGeom_*``;
    3. mesh geoms without ``group == 3`` are visual (collision meshes are
       always marked with ``group == 3`` in practice);
    4. otherwise, geoms that can never collide (``contype == conaffinity == 0``)
       are treated as visual.
    """
    contype = int(model.geom_contype[geom_id])
    conaffinity = int(model.geom_conaffinity[geom_id])
    group = int(model.geom_group[geom_id])
    if group == 2:
        return False
    if group == 3:
        return True

    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ''
    lname = name.lower()
    if 'collision' in lname:
        return True
    if 'visual' in lname:
        return False

    if int(model.geom_type[geom_id]) == int(mujoco.mjtGeom.mjGEOM_MESH):
        return False
    if contype == 0 and conaffinity == 0:
        return False
    return True


def _geom_rgba(model, geom_id):
    matid = int(model.geom_matid[geom_id])
    if matid >= 0 and int(model.nmat) > 0:
        return [float(v) for v in model.mat_rgba[matid]]
    return [float(v) for v in model.geom_rgba[geom_id]]


# ---------------------------------------------------------------------------
# XML builders
# ---------------------------------------------------------------------------


def _add_origin(parent, xyz, quat):
    ET.SubElement(parent, 'origin', {
        'xyz': _array2str(xyz),
        'rpy': _array2str(_quat_to_rpy(quat)),
    })


def _add_link(root, name):
    return ET.SubElement(root, 'link', {'name': name})


def _add_inertial(link, model, body_id):
    mass = float(model.body_mass[body_id])
    inertia = model.body_inertia[body_id]
    inertial = ET.SubElement(link, 'inertial')
    _add_origin(inertial, model.body_ipos[body_id], model.body_iquat[body_id])
    ET.SubElement(inertial, 'mass', {'value': '%.9g' % mass})
    ET.SubElement(inertial, 'inertia', {
        'ixx': '%.9g' % float(inertia[0]),
        'iyy': '%.9g' % float(inertia[1]),
        'izz': '%.9g' % float(inertia[2]),
        'ixy': '0', 'ixz': '0', 'iyz': '0',
    })


def _build_geometry(geometry, model, geom_id, mesh_ref):
    """Append the primitive/mesh tag for a geom. Return False if unsupported."""
    gtype = int(model.geom_type[geom_id])
    size = np.asarray(model.geom_size[geom_id], dtype=float)
    mj = mujoco.mjtGeom
    if gtype == int(mj.mjGEOM_MESH):
        ET.SubElement(geometry, 'mesh', {'filename': mesh_ref})
    elif gtype == int(mj.mjGEOM_BOX):
        # MuJoCo box size is the half-extent; URDF box size is the full extent.
        ET.SubElement(geometry, 'box', {'size': _array2str(size * 2.0)})
    elif gtype == int(mj.mjGEOM_SPHERE):
        ET.SubElement(geometry, 'sphere', {'radius': '%.9g' % float(size[0])})
    elif gtype == int(mj.mjGEOM_CYLINDER):
        ET.SubElement(geometry, 'cylinder', {
            'radius': '%.9g' % float(size[0]),
            'length': '%.9g' % (2.0 * float(size[1])),
        })
    elif gtype == int(mj.mjGEOM_CAPSULE):
        # URDF has no capsule; approximate with a cylinder of equal radius and
        # total length (the closest representable shape).
        ET.SubElement(geometry, 'cylinder', {
            'radius': '%.9g' % float(size[0]),
            'length': '%.9g' % (2.0 * float(size[1])),
        })
    elif gtype == int(mj.mjGEOM_ELLIPSOID):
        # URDF has no ellipsoid; approximate with an axis-aligned bounding box.
        ET.SubElement(geometry, 'box', {'size': _array2str(size * 2.0)})
    else:
        # PLANE and any future type: not representable / irrelevant (ground).
        return False
    return True


def _add_visual(link, model, geom_id, mesh_ref, material_name):
    visual = ET.SubElement(link, 'visual')
    _add_origin(visual, model.geom_pos[geom_id], model.geom_quat[geom_id])
    geometry = ET.SubElement(visual, 'geometry')
    if not _build_geometry(geometry, model, geom_id, mesh_ref):
        link.remove(visual)
        return False
    r, g, b, a = _geom_rgba(model, geom_id)
    material = ET.SubElement(visual, 'material', {'name': material_name})
    ET.SubElement(material, 'color', {
        'rgba': '%.6g %.6g %.6g %.6g' % (r, g, b, a),
    })
    return True


def _add_collision(link, model, geom_id, mesh_ref):
    collision = ET.SubElement(link, 'collision')
    _add_origin(collision, model.geom_pos[geom_id], model.geom_quat[geom_id])
    geometry = ET.SubElement(collision, 'geometry')
    if not _build_geometry(geometry, model, geom_id, mesh_ref):
        link.remove(collision)
        return False
    return True


# ---------------------------------------------------------------------------
# Joints
# ---------------------------------------------------------------------------


def _mj_joint_to_urdf_type(model, jid):
    """Map a MuJoCo joint type to a URDF joint type."""
    mtype = int(model.jnt_type[jid])
    mj = mujoco.mjtJoint
    if mtype == int(mj.mjJNT_HINGE):
        # Unlimited hinges are rotationally free -> ``continuous``.
        if not int(model.jnt_limited[jid]):
            return 'continuous'
        return 'revolute'
    if mtype == int(mj.mjJNT_SLIDE):
        return 'prismatic'
    if mtype in (int(mj.mjJNT_BALL), int(mj.mjJNT_FREE)):
        # No URDF equivalent for a ball joint; ``floating`` is the closest
        # supported type (and matches FreeCAD's own Ball convention).
        return 'floating'
    return 'fixed'


def _add_joint(root, name, jtype, parent, child, origin_xyz, origin_quat,
               axis=None, limits=None):
    joint = ET.SubElement(root, 'joint', {'name': name, 'type': jtype})
    ET.SubElement(joint, 'parent', {'link': parent})
    ET.SubElement(joint, 'child', {'link': child})
    _add_origin(joint, origin_xyz, origin_quat)
    if jtype in ('revolute', 'prismatic', 'continuous'):
        if axis is not None:
            ET.SubElement(joint, 'axis', {'xyz': _array2str(axis)})
        limit = {'effort': _DEFAULT_EFFORT, 'velocity': _DEFAULT_VELOCITY}
        if limits is not None:
            limit['lower'] = '%.9g' % float(limits[0])
            limit['upper'] = '%.9g' % float(limits[1])
        ET.SubElement(joint, 'limit', limit)
    return joint


def _add_body_joint_chain(root, model, bid, parent_name, body_name,
                          origin_xyz, origin_quat, base_names, dummy_counter):
    """Emit the joint chain connecting ``parent_name`` to ``body_name``.

    ``origin_xyz`` / ``origin_quat`` describe the body frame in the parent link
    frame. For a body with no joints a single fixed joint is emitted. For a body
    with joints, a chain of dummy links is inserted so each MuJoCo joint keeps
    its own anchor and axis (see the module docstring).
    """
    jnt_adr = int(model.body_jntadr[bid])
    jnt_num = int(model.body_jntnum[bid])

    if jnt_num == 0:
        _add_joint(root, '%s2%s_fixed' % (parent_name, body_name), 'fixed',
                   parent_name, body_name, origin_xyz, origin_quat)
        return

    # Fast path: a body with a single joint maps directly onto one URDF joint.
    # The joint position only shifts the joint frame inside the child link and is
    # folded into the joint origin, so no intermediate (massless) link is needed.
    if jnt_num == 1:
        jid = jnt_adr
        jname = _name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) or 'joint_%d' % jid
        jpos = np.asarray(model.jnt_pos[jid], dtype=float)
        jaxis = np.asarray(model.jnt_axis[jid], dtype=float)
        jtype = _mj_joint_to_urdf_type(model, jid)
        anchor_xyz = origin_xyz + _quat_rotate(origin_quat, jpos)
        limits = None
        if jtype in ('revolute', 'prismatic'):
            lo, hi = (float(v) for v in model.jnt_range[jid])
            limits = (lo, hi)
            if jtype == 'prismatic' and not int(model.jnt_limited[jid]):
                limits = (-1.0, 1.0)
        _add_joint(root, jname, jtype, parent_name, body_name, anchor_xyz,
                   origin_quat, axis=jaxis, limits=limits)
        return

    current_parent = parent_name
    prev_pos = np.zeros(3)
    last_jname = None
    for i in range(jnt_num):
        jid = jnt_adr + i
        jname = _name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) or 'joint_%d' % jid
        last_jname = jname
        jpos = np.asarray(model.jnt_pos[jid], dtype=float)
        jaxis = np.asarray(model.jnt_axis[jid], dtype=float)
        jtype = _mj_joint_to_urdf_type(model, jid)

        dummy_counter[0] += 1
        dummy = '%s_jointbody_%d' % (jname, dummy_counter[0])
        _add_link(root, dummy)
        base_names.add(dummy)

        if i == 0:
            # The first joint is anchored in the child body frame and the joint
            # frame orientation equals the body frame orientation.
            anchor_xyz = origin_xyz + _quat_rotate(origin_quat, jpos)
            anchor_quat = origin_quat
        else:
            # Subsequent joints are relative to the previous joint anchor; the
            # intermediate frames share the body frame orientation.
            anchor_xyz = jpos - prev_pos
            anchor_quat = np.array([1.0, 0.0, 0.0, 0.0])

        limits = None
        if jtype in ('revolute', 'prismatic'):
            lo, hi = (float(v) for v in model.jnt_range[jid])
            limits = (lo, hi)
            if jtype == 'prismatic' and not int(model.jnt_limited[jid]):
                limits = (-1.0, 1.0)

        _add_joint(root, jname, jtype, current_parent, dummy, anchor_xyz,
                   anchor_quat, axis=jaxis, limits=limits)

        current_parent = dummy
        prev_pos = jpos

    # Fixed joint that "brings back" the frame to the child body frame.
    _add_joint(root, '%s_offset' % last_jname, 'fixed', current_parent,
               body_name, -prev_pos, np.array([1.0, 0.0, 0.0, 0.0]))


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def convert_mjcf_to_urdf(mjcf_file, urdf_file, mesh_dir=None, package_prefix=None):
    """Convert an MJCF file to URDF using the MuJoCo Python bindings.

    Args:
        mjcf_file: Path to the MJCF file.
        urdf_file: Path to the URDF file to write.
        mesh_dir: Directory where the exported OBJ meshes are written. Defaults
            to a ``meshes`` directory next to ``urdf_file``.
        package_prefix: Optional prefix for mesh filenames, e.g.
            ``package://my_pkg/``. When ``None``, ``file://`` absolute paths are
            used so the FreeCAD importer can resolve them directly.

    Returns:
        The path to the written URDF file (as :class:`str`).
    """
    global mujoco
    if mujoco is None:
        import mujoco as _mujoco  # noqa: PLC0415
        mujoco = _mujoco

    mjcf_file = str(mjcf_file)
    urdf_file = str(urdf_file)
    if mesh_dir is None:
        mesh_dir = osp.join(osp.dirname(urdf_file), 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(mjcf_file)

    root = ET.Element('robot', {'name': 'converted_robot'})
    root.append(ET.Comment(
        'generated with freecad.cross.mjcf_urdf_mujoco (mujoco MJCF->URDF)'))

    # Export each mesh once. MuJoCo has already applied the asset ``scale`` and
    # centring offset to ``mesh_vert`` and folded them into ``geom_*``.
    exported = {}
    for mid in range(model.nmesh):
        name = _name(model, mujoco.mjtObj.mjOBJ_MESH, mid) or 'mesh_%d' % mid
        fname = '%s.obj' % _sanitize(name)
        _export_obj(model, mid, osp.join(mesh_dir, fname))
        exported[mid] = fname

    # Root link: always emit the MJCF ``world`` frame as the URDF root.
    _add_link(root, _WORLD_LINK)

    # Links + geoms.
    body_names = {}
    for bid in range(model.nbody):
        if bid == 0:
            body_names[bid] = _WORLD_LINK
            continue
        name = _name(model, mujoco.mjtObj.mjOBJ_BODY, bid) or 'body_%d' % bid
        body_names[bid] = name
        link = _add_link(root, name)
        _add_inertial(link, model, bid)
        _add_body_geoms(link, model, bid, exported, mesh_dir, package_prefix)

    # Joints.
    dummy_counter = [0]
    dummy_names = set()
    for bid in range(1, model.nbody):
        parent_bid = int(model.body_parentid[bid])
        parent_name = body_names[parent_bid]
        body_name = body_names[bid]
        body_pos = np.asarray(model.body_pos[bid], dtype=float)
        body_quat = np.asarray(model.body_quat[bid], dtype=float)
        _add_body_joint_chain(root, model, bid, parent_name, body_name,
                              body_pos, body_quat, dummy_names, dummy_counter)

    xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent='  ')
    with open(urdf_file, 'w') as f:
        f.write(xmlstr)
    return urdf_file


def _add_body_geoms(link, model, body_id, exported, mesh_dir, package_prefix):
    """Emit the visual/collision elements of a body."""
    adr = int(model.body_geomadr[body_id])
    num = int(model.body_geomnum[body_id])
    for geom_id in range(adr, adr + num):
        gtype = int(model.geom_type[geom_id])
        if gtype == int(mujoco.mjtGeom.mjGEOM_MESH):
            mid = int(model.geom_dataid[geom_id])
            fname = exported.get(mid)
            if package_prefix:
                mesh_ref = package_prefix + 'meshes/' + fname
            else:
                mesh_ref = 'file://' + osp.join(mesh_dir, fname)
        else:
            mesh_ref = ''

        if _geom_is_collision(model, geom_id):
            _add_collision(link, model, geom_id, mesh_ref)
        else:
            _add_visual(link, model, geom_id, mesh_ref, 'mat_%d' % geom_id)
