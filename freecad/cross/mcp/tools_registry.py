"""MCP tool implementations for the RobotCAD workbench.

Each tool is a plain Python function that operates on the active FreeCAD
document. Tools are registered with the MCP server in ``server.py``. All
FreeCAD calls are executed on the main GUI thread via ``run_on_main_thread``.

Tools accept object identifiers as strings (``Name`` or ``Label``) and return
JSON-serializable dictionaries.
"""

from __future__ import annotations

import base64
import json
import math
import os
import tempfile
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator, Optional

import FreeCAD as fc
import FreeCADGui as fcgui

try:
    from PySide import QtGui  # FreeCAD's PySide
except ImportError:
    from PySide6 import QtGui

from ..freecad_utils import is_lcs
from ..freecadgui_utils import (
    createBoundBox,
    createBoundSphere,
    createBoundXAlignedCylinder,
    createBoundYAlignedCylinder,
    createBoundZAlignedCylinder,
    createCollisionCopyObj,
)
from ..joint_proxy import make_joint, make_robot_joint_filled, make_robot_joints_filled
from ..link_proxy import make_link, make_robot_links_filled
from ..robot_proxy import make_robot
from ..wb_gui_utils import createBoundObjects
from ..wb_utils import (
    get_parent_link_of_obj,
    is_joint,
    is_link,
    is_robot,
    make_lcs_at_link_body,
    rotate_origin,
    set_placement_fast,
)
from .main_thread import run_on_main_thread

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _active_doc() -> fc.Document:
    doc = fc.activeDocument()
    if doc is None:
        raise RuntimeError('No active FreeCAD document. Create or open one first.')
    return doc


def _resolve_object(name: str) -> fc.DocumentObject:
    """Resolve an object by Name or Label in the active document."""
    doc = _active_doc()
    obj = doc.getObject(name)
    if obj is None:
        # Try by label.
        matches = doc.getObjectsByLabel(name)
        if len(matches) == 1:
            obj = matches[0]
        elif len(matches) > 1:
            candidates = ', '.join(o.Name for o in matches)
            raise RuntimeError(
                f'Ambiguous label "{name}". Candidates: {candidates}. Use Name instead.',
            )
    if obj is None:
        raise RuntimeError(f'Object "{name}" not found in document "{doc.Name}".')
    return obj


def _resolve_robot(name: str) -> fc.DocumentObject:
    obj = _resolve_object(name)
    if not is_robot(obj):
        raise RuntimeError(f'Object "{name}" is not a Cross::Robot.')
    return obj


def _resolve_link(name: str) -> fc.DocumentObject:
    obj = _resolve_object(name)
    if not is_link(obj):
        raise RuntimeError(f'Object "{name}" is not a Cross::Link.')
    return obj


def _resolve_joint(name: str) -> fc.DocumentObject:
    obj = _resolve_object(name)
    if not is_joint(obj):
        raise RuntimeError(f'Object "{name}" is not a Cross::Joint.')
    return obj


def _placement_to_dict(placement: fc.Placement) -> dict[str, Any]:
    base = placement.Base
    rot = placement.Rotation
    # ``Base.Rotation`` has no ``Q0``..``Q3`` attributes; the quaternion is
    # available as ``Rotation.Q``, a tuple ``(q0, q1, q2, q3)``.
    q0, q1, q2, q3 = rot.Q
    return {
        'x': base.x,
        'y': base.y,
        'z': base.z,
        'q0': q0,
        'q1': q1,
        'q2': q2,
        'q3': q3,
    }


def _obj_basic_info(obj: fc.DocumentObject) -> dict[str, Any]:
    info: dict[str, Any] = {
        'Name': obj.Name,
        'Label': obj.Label,
        'TypeId': obj.TypeId,
    }
    if hasattr(obj, 'Placement'):
        info['Placement'] = _placement_to_dict(obj.Placement)
    return info


@contextmanager
def _transaction(doc: fc.Document, label: str) -> Iterator[None]:
    """Context manager for a FreeCAD transaction."""
    doc.openTransaction(label)
    try:
        yield
    except Exception:
        doc.abortTransaction()
        raise
    else:
        doc.commitTransaction()
        doc.recompute()


# ---------------------------------------------------------------------------
# Creation tools
# ---------------------------------------------------------------------------


def _doc_info(doc: fc.Document) -> dict[str, Any]:
    """Return basic information about a document."""
    return {
        'Name': doc.Name,
        'Label': doc.Label,
        'Active': doc is fc.activeDocument(),
    }


def create_document(name: str = 'Unnamed') -> dict[str, Any]:
    """Create a new document and make it active. Required if none is active."""

    def _impl() -> dict[str, Any]:
        doc = fc.newDocument(name)
        fc.setActiveDocument(doc.Name)
        return _doc_info(doc)

    return run_on_main_thread(_impl)


def create_robot(name: str = 'Robot') -> dict[str, Any]:
    """Create a new empty Cross::Robot container."""

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        with _transaction(doc, 'MCP: create robot'):
            robot = make_robot(name, doc)
        return _obj_basic_info(robot)

    return run_on_main_thread(_impl)


def create_link(robot: str, name: str = 'Link', add_to_robot: bool = True) -> dict[str, Any]:
    """Create a new Cross::Link and optionally add it to a robot."""

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        robot_obj = _resolve_robot(robot)
        with _transaction(doc, 'MCP: create link'):
            link = make_link(name, doc, recompute_after=False)
            if add_to_robot:
                link.adjustRelativeLinks(robot_obj)
                robot_obj.addObject(link)
        return _obj_basic_info(link)

    return run_on_main_thread(_impl)


def create_links_filled(robot: str, object_names: list[str]) -> dict[str, Any]:
    """Create Cross::Link objects filled with Real/Visual from existing objects."""

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        robot_obj = _resolve_robot(robot)
        objs = [_resolve_object(n) for n in object_names]
        with _transaction(doc, 'MCP: create filled links'):
            links = make_robot_links_filled(objs, robot=robot_obj)
        if not links:
            raise RuntimeError('No links were created. Check that objects are valid parts/bodies.')
        return {'links': [_obj_basic_info(l) for l in links]}

    return run_on_main_thread(_impl)


def create_joint(
    robot: str,
    name: str = 'Joint',
    parent_link: str = '',
    child_link: str = '',
    type: str = 'fixed',
    axis: Optional[list[float]] = None,
    lower: float = 0.0,
    upper: float = 0.0,
    effort: float = 0.0,
    velocity: float = 0.0,
) -> dict[str, Any]:
    """Create a Cross::Joint between two links and set its properties."""

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        robot_obj = _resolve_robot(robot)
        with _transaction(doc, 'MCP: create joint'):
            if parent_link and child_link:
                parent = _resolve_link(parent_link)
                child = _resolve_link(child_link)
                joint = make_robot_joint_filled(parent, child, robot=robot_obj)
                if not joint:
                    raise RuntimeError('Failed to create joint between the given links.')
            else:
                joint = make_joint(name, doc, robot=robot_obj, recompute_after=False)
            if type:
                joint.Type = type
            if axis is not None and len(axis) == 3:
                # The joint axis is encoded in the Origin rotation: rotate the
                # local Z axis to point along `axis`.
                v = fc.Vector(axis[0], axis[1], axis[2])
                if v.Length < 1e-9:
                    raise RuntimeError('Joint axis must be a non-zero vector.')
                v.normalize()
                rot = fc.Rotation(fc.Vector(0, 0, 1), v)
                joint.Origin = fc.Placement(joint.Origin.Base, rot)
            joint.LowerLimit = lower
            joint.UpperLimit = upper
            joint.Effort = effort
            joint.Velocity = velocity
        return _obj_basic_info(joint)

    return run_on_main_thread(_impl)


def create_joints_filled(
    robot: str,
    link_names_in_order: list[str],
    connect_type: str = 'chain',
) -> dict[str, Any]:
    """Create joints between links: ``chain`` = consecutive links,
    ``spider`` = all to the first (hub) link."""

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        robot_obj = _resolve_robot(robot)
        links = [_resolve_link(n) for n in link_names_in_order]
        if len(links) < 2:
            raise RuntimeError('At least 2 links are required.')
        with _transaction(doc, 'MCP: create filled joints'):
            joints = make_robot_joints_filled(
                links,
                robot=robot_obj,
                joints_group_connect_type=connect_type,
            )
        if not joints:
            raise RuntimeError('No joints were created.')
        return {'joints': [_obj_basic_info(j) for j in joints]}

    return run_on_main_thread(_impl)


# ---------------------------------------------------------------------------
# Collision tool
# ---------------------------------------------------------------------------

#: Collision type -> (bound-creation function, human-readable label).
_COLLISION_TYPES: dict[str, tuple[Any, str]] = {
    'copy': (createCollisionCopyObj, 'collision copy'),
    'box': (createBoundBox, 'collision box'),
    'sphere': (createBoundSphere, 'collision sphere'),
    'cylinder_z': (createBoundZAlignedCylinder, 'collision cylinder z'),
    'cylinder_x': (createBoundXAlignedCylinder, 'collision cylinder x'),
    'cylinder_y': (createBoundYAlignedCylinder, 'collision cylinder y'),
}


def create_collision(link_or_robot: str, type: str = 'copy') -> dict[str, Any]:
    """Create a collision for a link or robot. Default `type='copy'` (exact
    geometry copy — the default). Primitives (`box`, `sphere`,
    `cylinder_z/x/y` from the bounding box) ONLY on explicit user request."""
    if type not in _COLLISION_TYPES:
        valid = ', '.join(f"'{t}'" for t in sorted(_COLLISION_TYPES))
        raise ValueError(f'Unknown collision type {type!r}. Valid types: {valid}.')
    bound_func, label = _COLLISION_TYPES[type]

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        obj = _resolve_object(link_or_robot)
        # createBoundObjects reads the current selection; select the object.
        fcgui.Selection.clearSelection()
        fcgui.Selection.addSelection(doc.Name, obj.Name)
        with _transaction(doc, f'MCP: {label}'):
            createBoundObjects(createBoundFunc=bound_func)
        fcgui.Selection.clearSelection()
        return {'status': 'ok', 'created_for': obj.Name, 'method': label, 'type': type}

    return run_on_main_thread(_impl)


# ---------------------------------------------------------------------------
# Placement / rotation tools
# ---------------------------------------------------------------------------


def set_object_placement(
    object_name: str,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    rx: float = 0.0,
    ry: float = 0.0,
    rz: float = 0.0,
    euler_units: str = 'deg',
    relative_to: str = 'global',
) -> dict[str, Any]:
    """Set Placement (joint, link, LCS, part) — PRIMARY positioning method.

    Position the JOINT, not the link (the link follows its joint): 1) position
    the joint (sets ``Origin``); 2) snapshot with ``get_snapshot``; 3) if the
    link stands wrong, fix its contact point with the joint via
    ``relative_to='current'`` (shifts/rotates the link ``MountedPlacement``).
    ``relative_to='global'`` = absolute, ``'current'`` = offset."""

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        obj = _resolve_object(object_name)
        if euler_units == 'deg':
            rx_r, ry_r, rz_r = math.radians(rx), math.radians(ry), math.radians(rz)
        else:
            rx_r, ry_r, rz_r = rx, ry, rz
        new_placement = fc.Placement(
            fc.Vector(x, y, z),
            fc.Rotation(rx_r, ry_r, rz_r),
        )
        with _transaction(doc, 'MCP: set placement'):
            if is_link(obj) and hasattr(obj, 'MountedPlacement'):
                # For Cross::Link the position relative to the parent joint is
                # stored in MountedPlacement; Placement is managed by the robot.
                if relative_to == 'current':
                    obj.MountedPlacement = obj.MountedPlacement * new_placement
                else:
                    obj.MountedPlacement = new_placement
            elif hasattr(obj, 'Placement'):
                if relative_to == 'current':
                    obj.Placement = obj.Placement * new_placement
                else:
                    obj.Placement = new_placement
            else:
                raise RuntimeError(f'Object "{object_name}" has no Placement property.')
        return _obj_basic_info(obj)

    return run_on_main_thread(_impl)


def _find_link_real_element(link_obj: fc.DocumentObject) -> fc.DocumentObject:
    """Return the FreeCAD link (App::Link) to the Real element of a robot link."""
    for o in getattr(link_obj, 'Group', []):
        if o.Name.startswith('real_') and hasattr(o, 'LinkedObject'):
            return o
    raise RuntimeError(
        f'No Real element found for link "{link_obj.Name}". '
        'Make sure the link has Real geometry and is visible.',
    )


def _geometry_token(part: str) -> bool:
    """Return True if ``part`` is a subelement token (``Face1``, ``Edge2``...)."""
    for prefix in ('Face', 'Edge', 'Vertex'):
        if part.startswith(prefix) and part[len(prefix):].isdigit():
            return True
    return False


def _canonical_object_name(doc: fc.Document, name: str) -> str:
    """Return the internal ``Name`` of an object given by ``Name`` or ``Label``."""
    obj = doc.getObject(name)
    if obj is not None:
        return obj.Name
    matches = doc.getObjectsByLabel(name)
    if len(matches) == 1:
        return matches[0].Name
    return name


def _real_link_wrapper(real_link: fc.DocumentObject) -> Optional[fc.DocumentObject]:
    """Return the object the Real ``App::Link`` points to (or None)."""
    linked = getattr(real_link, 'LinkedObject', None)
    if isinstance(linked, (list, tuple)):
        linked = linked[0] if linked else None
    return linked


def _find_link_by_real_tree(
    doc: fc.Document, name: str,
) -> tuple[Optional[fc.DocumentObject], Optional[fc.DocumentObject]]:
    """Find ``(link, real_link)`` whose Real element tree contains ``name``."""
    obj = doc.getObject(name)
    if obj is None:
        matches = doc.getObjectsByLabel(name)
        if len(matches) == 1:
            obj = matches[0]
    if obj is None:
        return None, None
    for link_obj in doc.Objects:
        if not is_link(link_obj):
            continue
        try:
            real_link = _find_link_real_element(link_obj)
        except RuntimeError:
            continue
        wrapper = _real_link_wrapper(real_link)
        if wrapper is None:
            continue
        out_recursive = list(getattr(wrapper, 'OutListRecursive', []))
        if obj is real_link or obj is wrapper or obj in out_recursive:
            return link_obj, real_link
    return None, None


def _try_select_subelement(
    doc: fc.Document, real_link_name: str, sub_path: str,
) -> bool:
    """Select ``sub_path`` on ``real_link_name``.

    Return True only if FreeCAD actually resolved a subelement: an invalid
    path may silently select the whole object or nothing at all.
    """
    try:
        fcgui.Selection.removeSelection(doc.Name, real_link_name)
    except Exception:  # noqa: BLE001
        pass
    try:
        fcgui.Selection.addSelection(doc.Name, real_link_name, sub_path)
    except Exception:  # noqa: BLE001
        return False
    sel = fcgui.Selection.getSelectionEx(doc.Name, 0)
    for entry in sel:
        if entry.ObjectName == real_link_name and entry.SubElementNames:
            return True
    return False


def _subelement_path_candidates(
    doc: fc.Document,
    real_link: fc.DocumentObject,
    sub: str,
    link_obj: Optional[fc.DocumentObject] = None,
) -> list[str]:
    """Build candidate subelement paths relative to ``real_link``.

    ``sub`` is a user-provided path like ``real_l_chassis001_.chassis001.Box.
    Face3``. The candidates tolerate ``Name``/``Label`` mixups, a redundant or
    omitted wrapper (``App::Part``) level, a missing PartDesign Body level, a
    missing feature level and a redundant Real link prefix. Every candidate is
    verified by the caller, so extra candidates are harmless. A robot link
    (``l_...``) prefix is NOT tolerated: references must start with the Real
    element link (``real_l_...``).
    """
    parts = [p for p in sub.split('.') if p]
    # Tolerate a redundant internal Real link prefix (real_l_...).
    if parts and parts[0].startswith('real_'):
        parts = parts[1:]
    if not parts:
        return []
    candidates: list[str] = []

    def _add(candidate: list[str]) -> None:
        text = '.'.join(candidate)
        if text and text not in candidates:
            candidates.append(text)

    _add(parts)
    mapped = [_canonical_object_name(doc, p) for p in parts]
    _add(mapped)
    wrapper = _real_link_wrapper(real_link)
    if wrapper is None:
        return candidates
    wrapper_name = wrapper.Name
    rest = mapped[1:] if mapped[0] == wrapper_name else mapped
    # As given under the wrapper, and without the wrapper level.
    _add([wrapper_name, *rest])
    _add(rest)
    bodies = [
        o for o in getattr(wrapper, 'Group', [])
        if o.isDerivedFrom('PartDesign::Body')
    ]
    for body in bodies:
        children = {o.Name for o in getattr(body, 'Group', [])}
        tip = getattr(body, 'Tip', None)
        # Insert the missing Body level: '<wrapper>.<body>.<feature>...'.
        if rest and rest[0] in children:
            _add([wrapper_name, body.Name, *rest])
        # The GUI path goes through the body tip feature: map a non-tip
        # feature (or a missing feature level) to the tip.
        if tip is not None and rest and rest[0] != tip.Name and rest[0] in children:
            _add([wrapper_name, body.Name, tip.Name, *rest[1:]])
            _add([wrapper_name, tip.Name, *rest[1:]])
        if tip is not None and len(rest) == 1 and _geometry_token(rest[0]):
            _add([wrapper_name, body.Name, tip.Name, rest[0]])
            _add([wrapper_name, tip.Name, rest[0]])
    return candidates


def _is_robot_link_name(doc: fc.Document, name: str) -> bool:
    """Return True if ``name`` resolves to a Cross::Link (robot link)."""
    obj = doc.getObject(name)
    if obj is None:
        matches = doc.getObjectsByLabel(name)
        if len(matches) == 1:
            obj = matches[0]
    return obj is not None and is_link(obj)


def _select_link_subelement(doc: fc.Document, ref: str) -> dict[str, str]:
    """Select a subelement on the Real element of a robot link.

    ``ref`` is a subelement path in the form
    ``<real_link>.<inner_link_name>.<feature>.<subelement>`` (e.g.
    ``real_l_chassis001_.chassis001.Box.Face3``) where ``<real_link>`` is the
    Real element link (``real_l_...``) of a robot link, ``<inner_link_name>``
    is the **Name of the App::Link inside the Real element** (e.g.
    ``chassis001``, ``wheel001``) — NOT the name of the source body the link
    was filled from (e.g. ``chassis``, ``wheel``) — and the subelement is a
    face, edge, circle or vertex of the link's ``Real`` element. The path is
    tolerant: the inner link may be given by ``Name`` or ``Label``,
    intermediate levels may be omitted and the internal Real element name
    (``real_l_...``) is accepted though not needed. **The robot link itself
    (``l_...``) cannot be used as a reference** — the path must start with the
    Real element link.
    """
    parts = [p for p in ref.split('.') if p]
    if len(parts) < 2:
        raise RuntimeError(
            f'"{ref}" is not a valid reference. Use a subelement path '
            '"<real_link>.<inner_link_name>.<feature>.<subelement>" (e.g. '
            '"real_l_chassis001_.chassis001.Box.Face3") or an LCS name. '
            'The inner link name is the Name of the App::Link inside the Real '
            'element (e.g. "chassis001"), NOT the source body name.'
        )
    # A robot link (l_...) cannot be a reference: the path must start with
    # the Real element link (real_l_...).
    if _is_robot_link_name(doc, parts[0]):
        try:
            real_name = _find_link_real_element(_resolve_link(parts[0])).Name
        except RuntimeError:
            real_name = f'real_{parts[0]}_'
        raise RuntimeError(
            f'"{ref}" starts with the robot link "{parts[0]}". A robot link '
            'cannot be a reference. Use the Real element link path instead: '
            '"<real_link>.<inner_link_name>.<feature>.<subelement>" (e.g. '
            f'"{real_name}.<inner_link_name>.<Face|Edge|VertexN>").'
        )
    link_obj = None
    real_link = None
    # The first part must be the Real element link (real_l_...) or an object
    # inside the Real element tree (the App::Part wrapper, a body, a feature).
    for candidate in doc.Objects:
        if not is_link(candidate):
            continue
        try:
            real = _find_link_real_element(candidate)
        except RuntimeError:
            continue
        if real.Name == parts[0] or real.Label == parts[0]:
            link_obj, real_link = candidate, real
            break
    if link_obj is None:
        link_obj, real_link = _find_link_by_real_tree(doc, parts[0])
    if link_obj is None or real_link is None:
        raise RuntimeError(
            f'"{ref}" is not a valid reference: "{parts[0]}" is not the Real '
            'element link of any robot link. Use '
            '"<real_link>.<inner_link_name>.<feature>.<subelement>" (e.g. '
            '"real_l_chassis001_.chassis001.Box.Face3") or an LCS name.'
        )
    candidates = _subelement_path_candidates(doc, real_link, ref, link_obj)
    for candidate in candidates:
        if _try_select_subelement(doc, real_link.Name, candidate):
            return {
                'object': real_link.Name,
                'subelement': candidate,
                'link': link_obj.Name,
            }
    try:
        fcgui.Selection.removeSelection(doc.Name, real_link.Name)
    except Exception:  # noqa: BLE001
        pass
    wrapper = _real_link_wrapper(real_link)
    wrapper_name = wrapper.Name if wrapper is not None else '<inner_link_name>'
    raise RuntimeError(
        f'Cannot select "{ref}" on the Real element of link '
        f'"{link_obj.Name}". Use '
        f'"{real_link.Name}.{wrapper_name}.<feature>.<Face|Edge|VertexN>" '
        f'(e.g. "{real_link.Name}.{wrapper_name}.Vertex1"). '
        f'The inner link name is the Name of the App::Link inside the Real '
        f'element (e.g. "chassis001"), NOT the source body name. '
        f'Tried paths: {", ".join(candidates)}.'
    )


def _select_reference(doc: fc.Document, ref: str) -> dict[str, str]:
    """Add a positioning reference (orienteer) to the FreeCAD selection.

    A reference for the Set Placement tools is either:
    - an LCS object (by ``Name`` or ``Label``), or
    - a subelement (face, edge, circle or vertex) of the ``Real`` element of
      a robot link, given as
      ``<real_link>.<inner_link_name>.<feature>.<subelement>``, e.g.
      ``real_l_chassis001_.chassis001.Box.Face3``. ``<inner_link_name>`` is
      the **Name of the App::Link inside the Real element** (e.g.
      ``chassis001``, ``wheel001``) — NOT the source body name (e.g.
      ``chassis``, ``wheel``).

    A robot link itself cannot be a reference.
    """
    try:
        obj = _resolve_object(ref)
    except RuntimeError:
        return _select_link_subelement(doc, ref)
    if is_lcs(obj):
        fcgui.Selection.addSelection(doc.Name, obj.Name)
        return {'object': obj.Name, 'subelement': ''}
    raise RuntimeError(
        f'"{ref}" cannot be used as a reference. A robot link (or any whole '
        'object) is not allowed: use a face, edge, vertex or circle of the '
        'Real element of a robot link in the form '
        '"<real_link>.<inner_link_name>.<feature>.<subelement>" (e.g. '
        '"real_l_chassis001_.chassis001.Box.Face3"), or an LCS. The inner '
        'link name is the Name of the App::Link inside the Real element '
        '(e.g. "chassis001"), NOT the source body name.'
    )


def set_placement_between(
    target: str,
    ref1: str,
    ref2: str,
    move: str = 'leaf',
) -> dict[str, Any]:
    """Position a link/joint by aligning two references — PRIMARY method.

    The default positioning method: snap the contact zones of two
    neighbouring links. ``ref1``/``ref2``: face/edge/vertex/circle of a link
    Real element as ``<real_link>.<inner_link_name>.<feature>.<subelement>``
    (e.g. ``real_l_chassis001_.chassis001.Box.Face3``) or an already existing
    LCS; a robot link (``l_...``) cannot be a reference. ``<inner_link_name>``
    is the **Name of the App::Link inside the Real element** (e.g.
    ``chassis001``, ``wheel001``) — NOT the source body name (e.g.
    ``chassis``, ``wheel``). One ref on the parent link, one on the child. Do
    NOT create LCS objects for this — plain subelement references are enough.
    ``move='leaf'`` (default, only supported) — final chain element only;
    ``child_branch``/``parent_tree`` are advanced."""

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        target_obj = _resolve_object(target)
        # set_placement_fast reads the selection: select only the two
        # orienteers (ref1, ref2); the target is NOT selected.
        fcgui.Selection.clearSelection()
        ref1_info = _select_reference(doc, ref1)
        ref2_info = _select_reference(doc, ref2)
        with _transaction(doc, 'MCP: set placement between'):
            if move == 'child_branch':
                result = set_placement_fast(child_branch_to_parent_tree=True)
            elif move == 'parent_tree':
                result = set_placement_fast(parent_tree_to_child_branch=True)
            elif move == 'leaf':
                result = set_placement_fast()
            else:
                raise RuntimeError(
                    f'Unknown move "{move}". Use "leaf" (default), '
                    '"child_branch" or "parent_tree".',
                )
        fcgui.Selection.clearSelection()
        if result is False:
            raise RuntimeError('set_placement_fast failed. Check the selection order and link chain.')
        return {
            'status': 'ok',
            'target': target_obj.Name,
            'move': move,
            'ref1': ref1_info,
            'ref2': ref2_info,
        }

    return run_on_main_thread(_impl)


def rotate_object(object_name: str, axis: str = 'z', angle_deg: float = 45.0) -> dict[str, Any]:
    """Rotate a joint Origin, link MountedPlacement or LCS by an angle about the given axis."""

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        obj = _resolve_object(object_name)
        fcgui.Selection.clearSelection()
        fcgui.Selection.addSelection(doc.Name, obj.Name)
        with _transaction(doc, 'MCP: rotate object'):
            if axis == 'x':
                ok = rotate_origin(x=angle_deg)
            elif axis == 'y':
                ok = rotate_origin(y=angle_deg)
            elif axis == 'z':
                ok = rotate_origin(z=angle_deg)
            else:
                raise RuntimeError(f'Unknown axis "{axis}". Use x, y or z.')
        fcgui.Selection.clearSelection()
        if not ok:
            raise RuntimeError('rotate_origin failed. Select a joint, link or LCS.')
        return {'status': 'ok', 'object': obj.Name, 'axis': axis, 'angle_deg': angle_deg}

    return run_on_main_thread(_impl)


def create_lcs(link: str, subelement: str = '') -> dict[str, Any]:
    """Create an LCS on a face/edge/circle/vertex of a link Real element.

    ONLY on explicit user request. ``subelement``: ``<real_link>.<body>.
    <subelement>`` (e.g. ``real_l_chassis001_.chassis001.Box.Face3``); short
    forms ``chassis001.Box.Face3`` / ``Box.Face3`` accepted; empty = Real
    origin. Tolerant to Name/Label mixups and missing levels; a robot link
    (``l_...``) cannot be used."""

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        link_obj = _resolve_link(link)
        real_link = _find_link_real_element(link_obj)
        # Select the subelement on the Real element. The path is tolerant to
        # Name/Label mixups and missing intermediate levels (see
        # _subelement_path_candidates).
        fcgui.Selection.clearSelection()
        selected = ''
        if subelement:
            sub_parts = [p for p in subelement.split('.') if p]
            if sub_parts and _is_robot_link_name(doc, sub_parts[0]):
                raise RuntimeError(
                    f'"{subelement}" starts with the robot link '
                    f'"{sub_parts[0]}". A robot link cannot be used in a '
                    'subelement path. Use the Real element link path instead: '
                    '"<real_link>.<body>.<subelement>" (e.g. '
                    '"real_l_...").'
                )
            candidates = _subelement_path_candidates(
                doc, real_link, subelement, link_obj,
            )
            for candidate in candidates:
                if _try_select_subelement(doc, real_link.Name, candidate):
                    selected = candidate
                    break
            if not selected:
                try:
                    fcgui.Selection.removeSelection(doc.Name, real_link.Name)
                except Exception:  # noqa: BLE001
                    pass
                wrapper = _real_link_wrapper(real_link)
                wrapper_name = wrapper.Name if wrapper is not None else '<body>'
                raise RuntimeError(
                    f'Cannot select subelement "{subelement}" on the Real '
                    f'element of link "{link_obj.Name}". Use '
                    f'"{real_link.Name}.{wrapper_name}.<feature>.'
                    '<Face|Edge|VertexN>" (e.g. '
                    f'"{real_link.Name}.{wrapper_name}.Vertex1"). '
                    f'Tried paths: {", ".join(candidates)}.'
                )
        else:
            fcgui.Selection.addSelection(doc.Name, real_link.Name)
        sel = fcgui.Selection.getSelectionEx('', 0)
        if not sel:
            raise RuntimeError('Failed to select the Real element subelement.')
        orienteer = sel[0]
        with _transaction(doc, 'MCP: create LCS'):
            lcs, body_lcs_wrapper, lcs_placement, doc_of_lcs = make_lcs_at_link_body(
                orienteer,
                delete_created_objects=False,
                deactivate_after_map_mode=True,
            )
        fcgui.Selection.clearSelection()
        return {
            'status': 'ok',
            'lcs': lcs.Name,
            'lcs_label': lcs.Label,
            'wrapper': body_lcs_wrapper.Name,
            'link': link_obj.Name,
            'subelement': selected,
        }

    return run_on_main_thread(_impl)


def set_placement_vision_mode(robot: str = '') -> dict[str, Any]:
    """Show only Real elements, hide Visual and Collision (call before
    positioning). Empty ``robot`` = all robots in the active document."""

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        if robot:
            robots = [_resolve_robot(robot)]
        else:
            robots = [o for o in doc.Objects if is_robot(o)]
        if not robots:
            raise RuntimeError('No robot found in the active document.')
        updated = []
        for robot_obj in robots:
            links = robot_obj.Proxy.get_links()
            for link in links:
                vobj = getattr(link, 'ViewObject', None)
                if vobj is None:
                    continue
                vobj.ShowReal = True
                vobj.ShowVisual = False
                vobj.ShowCollision = False
                updated.append(link.Name)
        doc.recompute()
        return {
            'status': 'ok',
            'robots': [r.Name for r in robots],
            'links': updated,
        }

    return run_on_main_thread(_impl)


# ---------------------------------------------------------------------------
# Material / inertia tools
# ---------------------------------------------------------------------------


def _find_material_card(name: str) -> Optional[str]:
    """Find a .FCMat material card by name (e.g. ``ABS-Generic``).

    Searches the workbench ``resources/materials`` directory and FreeCAD's
    built-in material libraries. Returns the card path or ``None``.
    """
    if not name:
        return None
    candidates = [name]
    if not name.endswith('.FCMat'):
        candidates.append(f'{name}.FCMat')
    wb_root = Path(__file__).resolve().parents[3]
    search_dirs = [
        wb_root / 'resources' / 'materials',
        Path(fc.getHomePath()) / 'Mod' / 'Material' / 'resources' / 'Materials',
        Path(fc.getHomePath()) / 'Mod' / 'Material',
    ]
    for search_dir in search_dirs:
        if not search_dir.is_dir():
            continue
        for root, _dirs, files in os.walk(search_dir):
            for candidate in candidates:
                if candidate in files:
                    return os.path.join(root, candidate)
    return None


def set_material(
    object_name: str,
    material_card_path: Optional[str] = None,
    density: Optional[float] = None,
    card_name: Optional[str] = None,
) -> dict[str, Any]:
    """Set the material of a link or robot.

    ``material_card_path`` may be a path to a ``.FCMat`` file or just a
    material name (e.g. ``ABS-Generic``); in the latter case the card is
    looked up automatically in the workbench resources and FreeCAD material
    libraries, and its path is substituted. ``density`` is optional: if not
    given, it is read from the material card. ``card_name`` is optional.
    """

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        obj = _resolve_object(object_name)
        if not (is_link(obj) or is_robot(obj)):
            raise RuntimeError(f'Object "{object_name}" is not a Cross::Link or Cross::Robot.')
        resolved_path = material_card_path
        if resolved_path and not os.path.isfile(resolved_path):
            # Treat the value as a material name and try to locate the card.
            resolved_path = _find_material_card(resolved_path)
            if resolved_path is None:
                raise RuntimeError(
                    f'Material card not found: "{material_card_path}". '
                    'Provide a valid .FCMat path or a known material name.',
                )
        with _transaction(doc, 'MCP: set material'):
            if resolved_path:
                obj.MaterialCardPath = resolved_path
                if card_name:
                    obj.MaterialCardName = card_name
                else:
                    obj.MaterialCardName = Path(resolved_path).stem
                # Try to read density from the card.
                try:
                    from importFCMat import read as read_fcmat
                    data = read_fcmat(resolved_path)
                    if 'Density' in data:
                        obj.MaterialDensity = data['Density']
                except Exception:  # noqa: BLE001
                    pass
                if density is not None:
                    obj.MaterialDensity = f'{density} kg/m^3'
            elif density is not None:
                if density <= 0.0:
                    raise RuntimeError('Density must be strictly positive.')
                obj.MaterialDensity = f'{density} kg/m^3'
                if card_name:
                    obj.MaterialCardName = card_name
            else:
                raise RuntimeError('Provide either material_card_path or density.')
        return {
            'status': 'ok',
            'object': obj.Name,
            'MaterialCardPath': getattr(obj, 'MaterialCardPath', ''),
            'MaterialCardName': getattr(obj, 'MaterialCardName', ''),
            'MaterialDensity': getattr(obj, 'MaterialDensity', ''),
        }

    return run_on_main_thread(_impl)


def calculate_mass_and_inertia(robot_or_link_names: list[str]) -> dict[str, Any]:
    """Calculate mass, inertia and center of mass for links of a robot.

    Accepts a robot name or a list of link names. Uses the same logic as the
    panel command ``CalculateMassAndInertia``.
    """

    def _impl() -> dict[str, Any]:
        from ..ui.command_calculate_mass_and_inertia import _CalculateMassAndInertiaCommand

        doc = _active_doc()
        objs = [_resolve_object(n) for n in robot_or_link_names]
        # Reuse the command's Activated logic by temporarily setting the
        # selection to the resolved objects.
        fcgui.Selection.clearSelection()
        for o in objs:
            fcgui.Selection.addSelection(doc.Name, o.Name)
        cmd = _CalculateMassAndInertiaCommand()
        cmd.Activated()
        fcgui.Selection.clearSelection()
        return {'status': 'ok', 'processed': [o.Name for o in objs]}

    return run_on_main_thread(_impl)


def set_joint_values(robot: str, values: dict[str, float]) -> dict[str, Any]:
    """Set joint values of a robot.

    ``values`` maps joint ROS names (or Labels) to values in degrees (revolute)
    or mm (prismatic).
    """

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        robot_obj = _resolve_robot(robot)
        # Map names to joint objects.
        joints = robot_obj.Proxy.get_joints()
        by_name = {j.Name: j for j in joints}
        by_label = {j.Label: j for j in joints}
        by_ros = {}
        for j in joints:
            try:
                from ..wb_utils import ros_name
                by_ros[ros_name(j)] = j
            except Exception:  # noqa: BLE001
                pass
        joint_values = {}
        for key, val in values.items():
            joint = by_name.get(key) or by_label.get(key) or by_ros.get(key)
            if joint is None:
                raise RuntimeError(f'Joint "{key}" not found in robot "{robot}".')
            joint_values[joint] = val
        with _transaction(doc, 'MCP: set joint values'):
            robot_obj.Proxy.set_joint_values(joint_values)
        return {'status': 'ok', 'robot': robot_obj.Name, 'values': values}

    return run_on_main_thread(_impl)


# ---------------------------------------------------------------------------
# Scene tools
# ---------------------------------------------------------------------------


def list_scene_objects() -> dict[str, Any]:
    """Return a JSON description of the active document's scene."""

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        robots = []
        links = []
        joints = []
        others = []
        for obj in doc.Objects:
            if is_robot(obj):
                robots.append(_obj_basic_info(obj))
            elif is_link(obj):
                links.append(_obj_basic_info(obj))
            elif is_joint(obj):
                info = _obj_basic_info(obj)
                info['Parent'] = getattr(obj, 'Parent', '')
                info['Child'] = getattr(obj, 'Child', '')
                info['Type'] = getattr(obj, 'Type', '')
                joints.append(info)
            else:
                others.append(_obj_basic_info(obj))
        return {
            'document': doc.Name,
            'robots': robots,
            'links': links,
            'joints': joints,
            'other_objects': others,
        }

    return run_on_main_thread(_impl)


# ---------------------------------------------------------------------------
# Geometry inspection helpers
# ---------------------------------------------------------------------------


def _vector_to_list(vector: fc.Vector) -> list[float]:
    """Convert a FreeCAD vector to a JSON-serializable ``[x, y, z]`` list."""
    return [vector.x, vector.y, vector.z]


def _bound_box_to_list(bound_box: fc.BoundBox) -> list[float]:
    """Convert a bound box to ``[xmin, ymin, zmin, xmax, ymax, zmax]``."""
    return [
        bound_box.XMin, bound_box.YMin, bound_box.ZMin,
        bound_box.XMax, bound_box.YMax, bound_box.ZMax,
    ]


def _resolve_shape_object(obj: fc.DocumentObject) -> Optional[fc.DocumentObject]:
    """Return the object whose ``Shape`` holds the geometry to report.

    For a ``Cross::Link`` the geometry lives in its ``Real`` element (the
    ``App::Link`` named ``real_...``); for any other object the object itself
    is used when it has a ``Shape``. Returns ``None`` when no shape is found.
    """
    if is_link(obj):
        try:
            return _find_link_real_element(obj)
        except RuntimeError:
            pass
        # Fallback: the first Real element that carries a shape.
        for real in getattr(obj, 'Real', []):
            if hasattr(real, 'Shape'):
                return real
        return None
    if hasattr(obj, 'Shape'):
        return obj
    return None


def _ancestor_placement(obj: fc.DocumentObject) -> fc.Placement:
    """Return the ancestor transform of ``obj`` (its own placement removed).

    ``obj.Shape`` is **already placed by ``obj.Placement``**: for a box at
    ``Placement=(100,0,0)`` the shape's bound box is ``(100,0,0)-(110,20,30)``,
    i.e. the local box shifted by the object's own placement. ``obj.Placement``
    is the object's own placement, while ``getGlobalPlacement()`` returns
    ``ancestors * obj.Placement``. Removing the object's own placement leaves
    the ancestor transform, which is what must be applied on top of
    ``obj.Shape`` to reach global coordinates::

        global_point = getGlobalPlacement() * obj.Placement.inverse() * shape_point
                     = ancestors * obj.Placement * obj.Placement.inverse() * shape_point
                     = ancestors * shape_point

    The result is the identity for top-level objects and for the ``Real``
    element of a robot link, whose shape is already in global coordinates
    because the robot sets the ``App::Link`` placement.
    """
    if hasattr(obj, 'getGlobalPlacement'):
        try:
            return obj.getGlobalPlacement() * obj.Placement.inverse()
        except Exception:  # noqa: BLE001
            return fc.Placement()
    return fc.Placement()


def _face_info(
    face: Any, index: int, ancestor_placement: fc.Placement,
) -> dict[str, Any]:
    """Return the spatial description of a single face (global coordinates).

    The face is identified by its 1-based ``index`` (matching FreeCAD's
    ``Face1``, ``Face2``, ...). The description contains the centre of mass,
    the normal at the centre, the surface type, the area, the bounding box and
    the coordinates of the face's vertices, so the agent can understand where
    the face lies on the object's surface. ``ancestor_placement`` is applied to
    the shape's points (which are already placed by the object's own
    placement) to reach global coordinates.
    """
    info: dict[str, Any] = {
        'index': index,
        'name': f'Face{index}',
        'area': face.Area,
        'center_of_mass': _vector_to_list(
            ancestor_placement.multVec(face.CenterOfMass),
        ),
        'bound_box': _bound_box_to_list(face.BoundBox),
        'orientation': str(face.Orientation),
        'vertices': [
            _vector_to_list(ancestor_placement.multVec(v.Point))
            for v in face.Vertexes
        ],
    }
    # Normal at the centre of the face's parameter range.
    try:
        u0, u1, v0, v1 = face.ParameterRange
        normal = face.normalAt((u0 + u1) / 2.0, (v0 + v1) / 2.0)
        info['normal'] = _vector_to_list(
            ancestor_placement.Rotation.multVec(normal),
        )
    except Exception:  # noqa: BLE001
        pass
    surface = getattr(face, 'Surface', None)
    surface_type = surface.__class__.__name__ if surface is not None else 'Unknown'
    info['surface_type'] = surface_type
    # Surface-specific parameters, transformed to global coordinates.
    try:
        if surface_type == 'Plane':
            info['plane_normal'] = _vector_to_list(
                ancestor_placement.Rotation.multVec(surface.Axis),
            )
            info['plane_origin'] = _vector_to_list(
                ancestor_placement.multVec(surface.Position),
            )
        elif surface_type == 'Cylinder':
            info['axis'] = _vector_to_list(
                ancestor_placement.Rotation.multVec(surface.Axis),
            )
            info['radius'] = surface.Radius
            info['center'] = _vector_to_list(
                ancestor_placement.multVec(surface.Center),
            )
        elif surface_type == 'Sphere':
            info['radius'] = surface.Radius
            info['center'] = _vector_to_list(
                ancestor_placement.multVec(surface.Center),
            )
        elif surface_type == 'Cone':
            info['axis'] = _vector_to_list(
                ancestor_placement.Rotation.multVec(surface.Axis),
            )
            info['radius'] = surface.Radius
            info['semi_angle'] = surface.SemiAngle
            info['apex'] = _vector_to_list(
                ancestor_placement.multVec(surface.Apex),
            )
        elif surface_type == 'Torus':
            info['axis'] = _vector_to_list(
                ancestor_placement.Rotation.multVec(surface.Axis),
            )
            info['major_radius'] = surface.MajorRadius
            info['minor_radius'] = surface.MinorRadius
            info['center'] = _vector_to_list(
                ancestor_placement.multVec(surface.Center),
            )
    except Exception:  # noqa: BLE001
        pass
    return info


def _vertex_info(
    vertex: Any, index: int, ancestor_placement: fc.Placement,
) -> dict[str, Any]:
    """Return the spatial description of a single vertex (global coordinates).

    The vertex is identified by its 1-based ``index`` (matching FreeCAD's
    ``Vertex1``, ``Vertex2``, ...). ``ancestor_placement`` is applied to the
    shape's point (already placed by the object's own placement) to reach
    global coordinates.
    """
    return {
        'index': index,
        'name': f'Vertex{index}',
        'point': _vector_to_list(ancestor_placement.multVec(vertex.Point)),
    }


def _object_geometry_info(
    obj: fc.DocumentObject,
    max_internal_geometry_items: int,
) -> dict[str, Any]:
    """Return the faces and vertices of an object in global coordinates."""
    # Never allow fewer than 100 items, so the geometry description stays
    # useful for positioning references.
    max_internal_geometry_items = max(100, max_internal_geometry_items)
    shape_obj = _resolve_shape_object(obj)
    if shape_obj is None:
        return {
            'error': (
                f'Object "{obj.Name}" has no shape geometry. For a Cross::Link '
                'make sure it has a Real element.'
            ),
        }
    shape = shape_obj.Shape
    # ``shape`` is already placed by ``shape_obj.Placement``; the ancestor
    # transform is applied on top to reach global coordinates.
    ancestor_placement = _ancestor_placement(shape_obj)
    result: dict[str, Any] = {
        'shape_object': shape_obj.Name,
        'coordinate_system': 'global',
        'ancestor_placement': _placement_to_dict(ancestor_placement),
        'bound_box': _bound_box_to_list(shape.BoundBox),
        'face_count': len(shape.Faces),
        'vertex_count': len(shape.Vertexes),
    }
    try:
        result['center_of_mass'] = _vector_to_list(
            ancestor_placement.multVec(shape.CenterOfGravity),
        )
    except Exception:  # noqa: BLE001
        pass
    try:
        result['volume'] = shape.Volume
        result['area'] = shape.Area
    except Exception:  # noqa: BLE001
        pass
    faces = list(shape.Faces)
    truncated = len(faces) > max_internal_geometry_items
    if truncated:
        faces = faces[:max_internal_geometry_items]
    result['faces'] = [
        _face_info(face, i + 1, ancestor_placement)
        for i, face in enumerate(faces)
    ]
    if truncated:
        result['faces_truncated'] = True
    vertices = list(shape.Vertexes)
    truncated = len(vertices) > max_internal_geometry_items
    if truncated:
        vertices = vertices[:max_internal_geometry_items]
    result['vertices'] = [
        _vertex_info(vertex, i + 1, ancestor_placement)
        for i, vertex in enumerate(vertices)
    ]
    if truncated:
        result['vertices_truncated'] = True
    return result


def get_object_info(
    object_name: str,
    max_internal_geometry_items: int = 1000,
) -> dict[str, Any]:
    """Return detailed information about a single object, including a ``Geometry`` entry with the global-coordinate description of every face and vertex."""

    def _impl() -> dict[str, Any]:
        obj = _resolve_object(object_name)
        info = _obj_basic_info(obj)
        # The composition of the object's Group (children in the tree).
        group = getattr(obj, 'Group', None)
        if group is not None:
            info['Group'] = [
                {
                    'Name': o.Name,
                    'Label': o.Label,
                    'TypeId': o.TypeId,
                }
                for o in group
            ]
        if is_link(obj):
            info['Real'] = [r.Name for r in getattr(obj, 'Real', [])]
            info['Visual'] = [v.Name for v in getattr(obj, 'Visual', [])]
            info['Collision'] = [c.Name for c in getattr(obj, 'Collision', [])]
            info['MountedPlacement'] = _placement_to_dict(obj.MountedPlacement)
            # Mass is a Base.Quantity, convert to a JSON-serializable string.
            mass = getattr(obj, 'Mass', None)
            info['Mass'] = str(mass) if mass is not None else None
            info['MaterialCardName'] = getattr(obj, 'MaterialCardName', '')
        elif is_joint(obj):
            info['Parent'] = getattr(obj, 'Parent', '')
            info['Child'] = getattr(obj, 'Child', '')
            info['Type'] = getattr(obj, 'Type', '')
            info['Origin'] = _placement_to_dict(getattr(obj, 'Origin', fc.Placement()))
            # Limits are Base.Quantity, convert to JSON-serializable strings.
            lower = getattr(obj, 'LowerLimit', None)
            upper = getattr(obj, 'UpperLimit', None)
            info['LowerLimit'] = str(lower) if lower is not None else None
            info['UpperLimit'] = str(upper) if upper is not None else None
        info['Geometry'] = _object_geometry_info(obj, max_internal_geometry_items)
        return info

    return run_on_main_thread(_impl)


# ---------------------------------------------------------------------------
# Snapshot tool
# ---------------------------------------------------------------------------


def get_snapshot(
    width: int = 1024,
    height: int = 768,
    format: str = 'png',
) -> dict[str, Any]:
    """Capture the active 3D view and return it as a base64 data URI."""

    def _impl() -> dict[str, Any]:
        if not fc.GuiUp:
            raise RuntimeError('FreeCAD GUI is not available.')
        doc = _active_doc()
        # Make sure the document is active in the GUI and has a 3D view.
        gui_doc = fcgui.getDocument(doc.Name)
        if gui_doc is None:
            raise RuntimeError(f'Document "{doc.Name}" is not open in the GUI.')
        fcgui.ActiveDocument = gui_doc
        view = gui_doc.ActiveView
        if view is None:
            # Try to open a 3D view for the document.
            mw = fcgui.getMainWindow()
            if mw is not None:
                mw.newWindow(doc)
                view = gui_doc.ActiveView
        if view is None:
            raise RuntimeError('No active 3D view available.')
        # Fit the view to the scene so the whole robot is visible.
        fcgui.SendMsgToActiveView('ViewFit')
        # Let the GUI process the view fit / redraw before capturing.
        QtGui.QApplication.processEvents()
        # saveImage() writes to a file, so use a temporary one and read it back.
        fd, save_path = tempfile.mkstemp(suffix=f'.{format}')
        os.close(fd)
        try:
            # saveImage() may return None even on success (it saves the file
            # directly), so verify the file on disk instead of the return value.
            view.saveImage(save_path, width, height, 'White')
            if not os.path.isfile(save_path) or os.path.getsize(save_path) == 0:
                raise RuntimeError('Failed to capture the 3D view.')
            with open(save_path, 'rb') as f:
                data = f.read()
        finally:
            try:
                os.remove(save_path)
            except OSError:
                pass
        mime = 'image/png' if format == 'png' else f'image/{format}'
        return {
            'width': width,
            'height': height,
            'format': format,
            'data_uri': f'data:{mime};base64,{base64.b64encode(data).decode("ascii")}',
        }

    return run_on_main_thread(_impl)


# ---------------------------------------------------------------------------
# Instructions tool
# ---------------------------------------------------------------------------

#: Header of the full agent instructions (``topic='all'``). The body is built
#: by concatenating the per-topic snippets (see ``_INSTRUCTIONS_FULL`` below).
_INSTRUCTIONS_HEADER = """\
# RobotCAD MCP — Agent Instructions

You are controlling the RobotCAD (FreeCAD OVERCROSS) workbench through MCP
tools. All tools operate on the **active FreeCAD document** and accept objects
by `Name` or `Label`. The sections below are the recommended workflow; the
**General algorithm** is the source of truth — follow it and resolve any
contradictions in its favor.
"""

#: Per-topic instruction snippets, in the order they are concatenated into the
#: full instructions (``_INSTRUCTIONS_FULL``). Keys are the accepted ``topic``
#: values. The ``general_algorithm`` snippet is the source of truth: the other
#: snippets must not contradict it.
_INSTRUCTIONS_BY_TOPIC: dict[str, str] = {
    'general_algorithm': """\
## General algorithm (source of truth)

1. No active document? `create_document(name)`.
2. `create_robot(name)`.
3. `create_links_filled(robot, object_names)` — create links from existing
   bodies; the same body may be reused for several links.
4. Create joints: select 2 neighbouring links at a time, root link first.
5. `set_placement_vision_mode(robot)` — show Real, hide Visual/Collision.
6. Position links pairwise along the chain. Default method:
   `set_placement_between(target, ref1, ref2)` — snap the contact zones of
   two neighbouring links. Per pair:
   a. Choose one reference on the parent link and one on the child link: a
      face/edge/vertex/circle of the link Real element as
      `<real_link>.<inner_link_name>.<feature>.<subelement>` (e.g.
      `real_l_chassis001_.chassis001.Box.Face3`). `<inner_link_name>` is the
      Name of the App::Link inside the Real element (e.g. `chassis001`,
      `wheel001`) — NOT the source body name (e.g. `chassis`, `wheel`).
      Do NOT create LCS objects —
      plain subelement references are enough. To pick the right face/vertex,
      first call `get_object_info(link)` and use the
      geometric information (`center_of_mass`, `normal`, `surface_type`,
      `bound_box`, `vertices`, `point`) to understand where each face/vertex
      actually lies — do not guess from the index alone.
   b. `set_placement_between(target, ref1, ref2)` — only the two references
      go into the selection.
   c. Control snapshot: `get_snapshot(...)`.
   d. Make the JOINT's local Z axis lie along the child link's functional
      axis (a revolute/continuous joint always rotates around its local Z, a
      prismatic joint always moves along its local Z). Set this direction with
      the `axis` parameter of `create_joint` (the local Z is rotated to point
      along `[x, y, z]`; e.g. a wheel whose axle is along Y -> `axis=[0, 1,
      0]`). A wheel whose axle is not along the joint's local Z will not roll.
      Do NOT rotate the LINK to achieve this: the link is mounted on the joint
      and follows it, so rotating the link would break its alignment with the
      joint's local Z.
   e. Then aim the whole assembly by rotating the JOINT with
      `rotate_object(object_name, axis, angle_deg)`: rotating the joint turns
      its local Z (and with it the child kinematic chain and end link) into
      the required direction. Do NOT rotate the link to fix the final pose —
      rotate the joint.
   e-bis. Mirroring a link to the other side of the parent: if the joint is
      oriented correctly AND the link is oriented correctly, but the link's
      body penetrates the parent link's body through its full height (the
      link should sit on the opposite side), rotate the LINK 180° about the
      X axis: `rotate_object(link_name, 'x', 180)`. The link is aligned along
      its local Z (the functional axis), so rotating it about X mirrors it
      about Z — the link flips to the other side of the joint while its
      Z-axis alignment and the joint orientation stay intact. Use the X axis,
      NOT Z: rotating the link about Z would only spin it around its own
      functional axis and would not mirror it at all. Use this for wheels on
      one side, or generally to place symmetric links on opposite sides of a
      parent kinematic chain.
   f. Verify with another snapshot.
7. Add collisions: `create_collision(link_or_robot)` — default WITHOUT `type`
   (type `copy`). Primitive `type` values (`box`, `sphere`, `cylinder_x/y/z`)
   ONLY on explicit user request.
8. `set_material(...)`; if not specified, use `ABS-Generic`.
9. `calculate_mass_and_inertia(robot)`.
10. Use `list_scene_objects()`, `get_object_info(...)`, `get_snapshot(...)`
    as needed to inspect and verify.
""",
    'create_robot': """\
## Creating a robot

- `create_document(name)` — new document (required if none is active).
- `create_robot(name)` — empty `Cross::Robot`.
- `create_link(robot, name, add_to_robot=True)` — empty `Cross::Link`.
- `create_links_filled(robot, object_names)` — links filled with
  Real/Visual from existing objects, in the given order.
""",
    'positioning': """\
## Positioning

- `set_placement_between(target, ref1, ref2, move)` — PRIMARY method: snaps
  the contact zones of two neighbouring links by two references (one on the
  parent link, one on the child link): a face/edge/vertex/circle of the link
  Real element as `<real_link>.<inner_link_name>.<feature>.<subelement>` (e.g.
  `real_l_chassis001_.chassis001.Box.Face3`) or an already existing LCS; a
  robot link (`l_...`) cannot be a reference. `move='leaf'` (default) — only
  for the final chain element; `child_branch`/`parent_tree` are advanced.
  Only the two references go into the selection (the target is not selected).
- To choose the correct face/vertex reference, use the geometric information
  from `get_object_info(link)` (`center_of_mass`,
  `normal`, `surface_type`, `bound_box`, `vertices`, `point`) to understand
  the spatial arrangement of the faces/vertices — do not rely on the index
  alone.
- Do NOT create LCS objects: plain subelement references are enough.
  `create_lcs` ONLY on explicit user request.
- `set_placement_vision_mode(robot)` — show Real, hide Visual/Collision;
  call before positioning.
- `rotate_object(object_name, axis, angle_deg)` — rotates a joint `Origin`,
  link `MountedPlacement` or LCS; correct orientation after a snapshot.
  Rotating a joint aims its local Z axis (revolute/continuous rotate around
  Z, prismatic moves along Z) and rotates its child kinematic chain and end
  link together with it.
- Orientation rule: make the JOINT's local Z lie along the child link's
  functional axis by setting the `axis` parameter of `create_joint` (e.g.
  `axis=[0, 1, 0]` for a wheel whose axle is along Y), then rotate the JOINT
  to aim that axis in the required direction. A revolute/continuous joint
  always spins around its local Z, so a wheel whose axle is not along that Z
  will not roll. Do NOT rotate the LINK for this: the link is mounted on the
  joint and follows it, so rotating the link breaks its alignment with the
  joint's local Z.
- Mirroring a link to the other side of the parent: when the joint is
  oriented correctly AND the link is oriented correctly, but the link's body
  penetrates the parent link's body through its full height (the link should
  be mirrored to the opposite side), rotate the LINK 180° about the X axis
  with `rotate_object(link_name, 'x', 180)`. The link is aligned along its
  local Z (the functional axis), so rotating it about X mirrors it about Z:
  the link flips to the other side of the joint while its Z-axis alignment
  and the joint orientation stay intact. Use the X axis, NOT Z: rotating the
  link about Z would only spin it around its own functional axis and would
  not mirror it. Use this for wheels on one side, or generally to place
  symmetric links on opposite sides of a parent kinematic chain.
- `create_lcs(link, subelement)` — LCS on a face/edge/circle/vertex of the
  Real element. ONLY on explicit user request.
""",
    'joints': """\
## Joints

IMPORTANT: a joint always rotates around its **local Z axis** (revolute,
continuous) or translates along its **local Z axis** (prismatic). The `axis`
parameter only reorients the joint frame so that its local Z points along the
given `[x, y, z]` direction — it does not add a new degree of freedom.

- `create_joint(robot, name, parent_link, child_link, type, axis, lower,
  upper, effort, velocity)` — joint between two links; `type`: `fixed`,
  `revolute`, `prismatic`, `continuous`, ...; `axis` `[x, y, z]` (local Z is
  rotated to point along it); limits in degrees (revolute) or mm (prismatic).
- `create_joints_filled(robot, link_names_in_order, connect_type)` — auto
  joints: `chain` = consecutive links, `spider` = all to the first link.
- `set_joint_values(robot, values)` — joint ROS names (or Labels) -> degrees
  (revolute) or mm (prismatic).

### Joint axis orientation (IMPORTANT)

- A `revolute` or `continuous` joint **rotates around its local Z axis** — the
  blue arrow shown on the joint in the 3D view.
- A `prismatic` joint **moves along its local Z axis** — the same blue arrow.
- After positioning, the joint can be rotated with the rotation tools
  (`rotate_object`) to aim its Z axis in the required direction. Rotating a
  joint also rotates its **child kinematic chain and the end link** together
  with it, so the whole downstream branch follows the joint orientation.
- Therefore, to orient a wheel/arm correctly, rotate the JOINT (not only the
  link): the child link and everything after it turn with the joint.
- **Make the joint's local Z lie along the child link's functional axis.**
  A revolute or continuous joint always rotates around its local Z, so the
  child link's functional axis (e.g. a wheel's axle) must be aligned with that
  local Z — otherwise the joint will spin the link about the wrong axis and
  the wheel will not roll. Set this direction with the `axis` parameter of
  `create_joint` (the local Z is rotated to point along `[x, y, z]`; e.g. a
  wheel whose axle is along Y -> `axis=[0, 1, 0]`).
- **Do NOT rotate the LINK to achieve this.** The link is mounted on the joint
  and follows it; rotating the link turns it relative to the joint's local Z
  and therefore breaks exactly the alignment you need.
- **Then aim the whole assembly by rotating the JOINT**, not the link: use
  `rotate_object` on the joint to turn its local Z (and the child link with
  it) into the required direction. Rotating only the link would leave the
  joint's rotation axis pointing the wrong way.
- Summary: (1) set the joint's local Z along the link's functional axis via
  the `axis` parameter of `create_joint`; (2) rotate the JOINT to point that
  axis where it is needed.
""",
    'collisions': """\
## Collisions

- `create_collision(link_or_robot, type)` — default WITHOUT `type` (type
  `copy` = exact geometry copy; the only allowed default). Primitive types
  ONLY on explicit user request: `box`, `sphere`, `cylinder_x/y/z` (from the
  bounding box).
""",
    'materials': """\
## Materials and inertia

- `set_material(object_name, material_card_path, density, card_name)` —
  `.FCMat` path or material name; `density` in kg/m^3 (optional, from the
  card if omitted).
- `calculate_mass_and_inertia(robot_or_link_names)` — mass, inertia, center
  of mass; robot name or list of link names.
""",
    'inspection': """\
## Inspection

- `list_scene_objects()` — scene description (robots, links, joints, other
  objects) with placements.
- `get_object_info(object_name, max_internal_geometry_items)` — one object details
  (Real, Visual, Collision, Mass, joint limits, ...). It always returns
  `Geometry`: the spatial description of every face and vertex in
  **global** coordinates. A face has a 1-based `index` (`Face1`, `Face2`, ...),
  its `center_of_mass`, the `normal` at its centre, its `surface_type`
  (`Plane`, `Cylinder`, `Sphere`, `Cone`, `Torus`, ...), `area`, `bound_box`
  and the coordinates of its `vertices`; a vertex has a 1-based `index`
  (`Vertex1`, ...) and its `point`. For a `Cross::Link` the geometry of its
  `Real` element is reported, so the indices can be used directly in
  positioning references (e.g. `real_l_...Face3`).

  IMPORTANT: to understand the spatial arrangement of faces and vertices —
  which face is on top, which faces are parallel, which vertex is a corner,
  where a face lies on the object's surface — ALWAYS use this geometric
  information (`center_of_mass`, `normal`, `surface_type`, `bound_box`,
  `vertices`, `point`), NOT the face/vertex index alone. The index only
  identifies the subelement; the coordinates tell you where it actually is.
  Call `get_object_info(...)` before choosing
  positioning references, so the reference is picked from real coordinates
  rather than guessed.
- `get_snapshot(width, height, format)` — 3D view capture; returns a base64
  `data:` URI.
""",
    'best_practices': """\
## Best practices

- Resolve exact names with `list_scene_objects()` / `get_object_info()` before
  positioning.
- To understand where faces/vertices are located, use the geometric
  information from `get_object_info(...)`
  (`center_of_mass`, `normal`, `surface_type`, `bound_box`, `vertices`,
  `point`) — never rely on the face/vertex index alone.
- Order: robot and links -> joints -> positioning -> collisions/materials.
- `set_placement_vision_mode()` before positioning.
- Position with `set_placement_between()`; verify each step with
  `get_snapshot()`; fix orientation with `rotate_object()` if needed.
- Orient a jointed link in two steps: (1) set the joint's local Z along the
  link's functional axis via the `axis` parameter of `create_joint` (e.g.
  `axis=[0, 1, 0]` for a wheel whose axle is along Y); (2) rotate the JOINT to
  aim that axis where it is needed. A revolute/continuous joint always rotates
  around its local Z, so a wheel whose axle is not along that Z will not roll.
  Do NOT rotate the LINK for this — the link follows its joint, so rotating
  the link breaks its alignment with the joint's local Z.
- If the joint and the link are both correctly oriented but the link's body
  penetrates the parent link's body through its full height, mirror the link
  to the other side by rotating the LINK 180° about the X axis:
  `rotate_object(link_name, 'x', 180)`. The link is aligned along its local Z
  (the functional axis), so rotating it about X mirrors it about Z: the link
  simply ends up on the opposite side of the joint, still attached to it,
  while the joint orientation and the link's Z-axis alignment are preserved.
  Use the X axis, NOT Z: rotating the link about Z would only spin it around
  its own functional axis and would not mirror it. This is the way to place
  symmetric links (e.g. wheels) on opposite sides of a parent kinematic
  chain.
- Do NOT create LCS objects: `create_lcs()` only on explicit user request;
  plain subelement references are enough.
- Prefer explicit subelement references over guessing by eye.
""",
}

#: Concatenation order of the per-topic snippets for ``topic='all'``.
_INSTRUCTIONS_FULL_TOPIC_ORDER: list[str] = [
    'general_algorithm',
    'create_robot',
    'positioning',
    'joints',
    'collisions',
    'materials',
    'inspection',
    'best_practices',
]

#: Full agent instructions, returned by ``instructions_to_work_with_tools``.
#: Built by concatenating the header with the per-topic snippets; edit
#: ``_INSTRUCTIONS_BY_TOPIC`` (or ``_INSTRUCTIONS_HEADER``), not this constant.
_INSTRUCTIONS_FULL = _INSTRUCTIONS_HEADER + '\n'.join(
    _INSTRUCTIONS_BY_TOPIC[topic]
    for topic in _INSTRUCTIONS_FULL_TOPIC_ORDER
)


def instructions_to_work_with_tools(topic: str = 'all') -> dict[str, Any]:
    """Return step-by-step instructions for the agent on how to use the tools.

    Guides the agent through creating a robot, positioning its parts and
    working with the other MCP tools. ``topic`` selects a focused section:

    - ``'all'`` (default): the full workflow guide.
    - ``'general_algorithm'``: the source-of-truth step-by-step workflow.
    - ``'create_robot'``: creating documents, robots and links.
    - ``'positioning'``: set_placement_vision_mode / contact-zone selection /
      set_placement_between / rotate_object / create_lcs.
    - ``'joints'``: creating joints and setting joint values.
    - ``'collisions'``: collision geometry tools.
    - ``'materials'``: materials and mass/inertia.
    - ``'inspection'``: scene, object info, selection and snapshots.
    - ``'best_practices'``: general recommendations.

    This tool is pure information: it does not modify the document.
    """
    if topic == 'all':
        text = _INSTRUCTIONS_FULL
    elif topic in _INSTRUCTIONS_BY_TOPIC:
        text = _INSTRUCTIONS_BY_TOPIC[topic]
    else:
        valid = ', '.join(sorted(['all', *_INSTRUCTIONS_BY_TOPIC]))
        raise RuntimeError(f'Unknown topic "{topic}". Valid topics: {valid}.')
    return {
        'topic': topic,
        'instructions': text,
    }


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

#: Ordered list of (tool_name, callable) pairs. The MCP server registers these.
TOOLS: list[tuple[str, Any]] = [
    ('create_document', create_document),
    ('create_robot', create_robot),
    ('create_link', create_link),
    ('create_links_filled', create_links_filled),
    ('create_joint', create_joint),
    ('create_joints_filled', create_joints_filled),
    ('create_collision', create_collision),
    # set_object_placement is disabled: set_placement_between is the default
    # positioning method (see the instructions above). Re-enable only on
    # explicit request.
    # ('set_object_placement', set_object_placement),
    ('set_placement_between', set_placement_between),
    ('rotate_object', rotate_object),
    ('create_lcs', create_lcs),
    ('set_placement_vision_mode', set_placement_vision_mode),
    ('set_material', set_material),
    ('calculate_mass_and_inertia', calculate_mass_and_inertia),
    ('set_joint_values', set_joint_values),
    ('list_scene_objects', list_scene_objects),
    ('get_object_info', get_object_info),
    ('get_snapshot', get_snapshot),
    ('instructions_to_work_with_tools', instructions_to_work_with_tools),
]


def get_tools() -> list[tuple[str, Any]]:
    """Return the list of (name, callable) tool pairs."""
    return list(TOOLS)