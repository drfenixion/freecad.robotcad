"""MCP tool implementations for the RobotCAD workbench.

Each tool is a plain Python function that operates on the active FreeCAD
document. Tools are registered with the MCP server in ``server.py``. All
FreeCAD calls are executed on the main GUI thread via ``run_on_main_thread``.

Tools accept object identifiers as strings (``Name`` or ``Label``) and return
JSON-serializable dictionaries, except ``get_snapshot`` which returns MCP
content blocks (a text line plus an image).
"""

from __future__ import annotations

import base64
import json
import math
import os
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator, Optional

import FreeCAD as fc
import FreeCADGui as fcgui

try:
    from PySide import QtGui  # FreeCAD's PySide
except ImportError:
    from PySide6 import QtGui

from ..controller_proxy import get_controllers_data, make_broadcaster, make_controller
from ..freecad_utils import is_link as is_freecad_link
from ..freecad_utils import is_lcs
from ..sensors.sensor_factory import make_sensor
from ..sensors.sensor_proxy import get_sensors_data
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
from .. import wb_constants
from ..wb_utils import (
    get_parent_link_of_obj,
    is_broadcaster,
    is_controller,
    is_joint,
    is_link,
    is_robot,
    is_sensor,
    is_sensor_joint,
    is_sensor_link,
    make_lcs_at_link_body,
    ros_name,
    rotate_origin,
    set_placement_fast,
    set_placement_sensor,
)
from .errors import as_anticipated_error
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


def _validate_collision_type(type: str) -> tuple[Any, str]:
    """Return ``(bound_func, label)`` for a collision type or raise."""
    if type not in _COLLISION_TYPES:
        valid = ', '.join(f"'{t}'" for t in sorted(_COLLISION_TYPES))
        raise ValueError(f'Unknown collision type {type!r}. Valid types: {valid}.')
    return _COLLISION_TYPES[type]


def create_collision(link_or_robot: str, type: str = 'copy') -> dict[str, Any]:
    """Create a collision for a single link or robot. Default `type='copy'`
    (exact geometry copy — the default). Primitives (`box`, `sphere`,
    `cylinder_z/x/y` from the bounding box) ONLY on explicit user request.

    NOTE: this single-object function is NOT exposed to the agent as an MCP
    tool; it is called internally by ``create_collisions`` (once per element).
    """
    bound_func, label = _validate_collision_type(type)

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


def create_collisions(
    link_or_robot: list[str],
    type: list[str] | str = 'copy',
) -> dict[str, Any]:
    """Create collisions for several links/robots in ONE call.

    ``link_or_robot`` is a list of link/robot Names or Labels. ``type`` is either
    a single collision type applied to every element, or a list parallel to
    ``link_or_robot`` (one type per element). Default ``type='copy'`` (exact
    geometry copy — the default). Primitives (``box``, ``sphere``,
    ``cylinder_z/x/y`` from the bounding box) ONLY on explicit user request.

    A robot name covers the robot AS A WHOLE (all its links). When a robot and
    some of its own links are passed together, the robot's links are removed
    from the list FIRST (so they are not collision-ised twice): the pass runs
    over all the robot's links, then over the remaining explicitly-listed
    elements.

    Internally this calls ``create_collision`` for each element.
    """
    # Normalise `type` into a per-element list.
    if isinstance(type, str):
        types = [type]
    else:
        types = list(type)
    if not types:
        types = ['copy']
    for t in set(types):
        _validate_collision_type(t)

    elements = list(link_or_robot)
    if not elements:
        raise ValueError('link_or_robot must contain at least one link or robot name.')

    def _type_for(index: int) -> str:
        # A single type applies to every element; otherwise it is positional.
        return types[0] if len(types) == 1 else types[index]

    def _impl() -> dict[str, Any]:
        # Resolve everything up front (an unknown name aborts the whole call).
        resolved: list[tuple[str, fc.DocumentObject]] = []
        for name in elements:
            resolved.append((name, _resolve_object(name)))

        # Robot-link de-duplication: a robot covers all its links. Any
        # explicitly-passed link that belongs to a passed robot is removed from
        # the list, so it gets exactly one collision (via the robot).
        robot_links: set[str] = set()
        for _, obj in resolved:
            if is_robot(obj):
                for link in obj.Proxy.get_links():
                    robot_links.add(link.Name)

        created: list[dict[str, Any]] = []
        skipped: list[str] = []
        for index, (name, obj) in enumerate(resolved):
            coll_type = _type_for(index)
            if is_robot(obj):
                kind = 'robot'
            elif is_link(obj):
                kind = 'link'
            else:
                raise RuntimeError(
                    f'"{name}" is not a Cross::Link or Cross::Robot.',
                )
            # Covered by a robot passed in the same call -> avoid duplication.
            if kind == 'link' and obj.Name in robot_links:
                skipped.append(obj.Name)
                continue
            # Delegate the actual work to the single-object create_collision
            # (it re-enters on the main thread, so this is a direct call).
            single = create_collision(name, coll_type)
            created.append({
                'name': name,
                'object': single.get('created_for', obj.Name),
                'kind': kind,
                'type': coll_type,
                'method': single.get('method'),
            })

        result: dict[str, Any] = {
            'status': 'ok',
            'count': len(created),
            'created': created,
        }
        if skipped:
            result['skipped'] = skipped
        return result

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


def _reference_paths(link_obj: fc.DocumentObject) -> dict[str, Any]:
    """Return the exact subelement-reference paths for a robot link.

    A positioning reference for ``set_placement_between`` must be
    ``<real_link>.<inner_link_name>.<feature>.<subelement>`` where
    ``<inner_link_name>`` is the **Name of the App::Link inside the Real
    element** — NOT the source body name and NOT the robot link name.

    Several links filled from the **same** source body share one wrapper
    ``App::Part`` and therefore one inner link name. For four wheels created
    from one ``wheel`` body, every wheel link's inner link is ``wheel001``;
    the per-link suffix (``wheel002``, ``wheel003``, ...) is NOT a valid inner
    link name. This helper resolves the real path from the object tree so the
    agent never has to guess it.

    Return a dict with:
    - ``real_link``: Name of the Real element link (``real_l_...``).
    - ``inner_link``: Name of the App::Link inside the Real element.
    - ``feature``: Name of the geometry feature (``Box``, ``Cylinder``, ...)
      when it can be determined, else ``None``.
    - ``prefix``: ready-to-use path prefix
      ``<real_link>.<inner_link>.<feature>`` (feature omitted when unknown).
    - ``example``: a concrete example reference (``<prefix>.Face1``).

    Return an empty dict when the link has no Real element.
    """
    try:
        real_link = _find_link_real_element(link_obj)
    except RuntimeError:
        return {}
    wrapper = _real_link_wrapper(real_link)
    if wrapper is None:
        return {}
    # The inner App::Link is the single App::Link child of the wrapper.
    inner_link = None
    for child in getattr(wrapper, 'Group', []):
        if is_freecad_link(child):
            inner_link = child
            break
    if inner_link is None:
        # Fall back to the wrapper itself (e.g. a plain App::Part).
        inner_link = wrapper
    # The geometry feature is the tip of the PartDesign Body the inner link
    # points to, or the linked object itself.
    linked = getattr(inner_link, 'LinkedObject', None)
    if isinstance(linked, (list, tuple)):
        linked = linked[0] if linked else None
    feature = None
    if linked is not None:
        tip = getattr(linked, 'Tip', None)
        if tip is not None:
            feature = tip.Name
        elif hasattr(linked, 'Shape'):
            feature = linked.Name
    parts = [real_link.Name, inner_link.Name]
    if feature:
        parts.append(feature)
    prefix = '.'.join(parts)
    return {
        'real_link': real_link.Name,
        'inner_link': inner_link.Name,
        'feature': feature,
        'prefix': prefix,
        'example': f'{prefix}.Face1',
    }


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
    # The inner App::Link name (the only valid <inner_link_name>). Several
    # links filled from the same source body share one wrapper and therefore
    # one inner link name; the per-link suffix is NOT valid.
    inner_name = None
    for child in getattr(wrapper, 'Group', []):
        if is_freecad_link(child):
            inner_name = child.Name
            break
    if inner_name is None:
        inner_name = wrapper_name
    # If the caller gave a wrong inner-link segment (e.g. "wheel003" instead
    # of "wheel001"), substitute the real one so the reference still resolves.
    if inner_name and mapped and mapped[0] not in (wrapper_name, inner_name):
        _add([inner_name, *mapped[1:]])
        _add([wrapper_name, inner_name, *mapped[1:]])
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


def _is_lidar_sensor(obj: fc.DocumentObject) -> bool:
    """Return True if `obj` is a sensor whose type is a lidar (e.g. `gpu_lidar`)."""
    return 'lidar' in str(getattr(obj, 'attr_type', '') or '').lower()


def set_placement_between(
    target: str,
    ref1: str,
    ref2: str,
    move: str = 'leaf',
) -> dict[str, Any]:
    """Position a link/joint by snapping the contact zones of two neighbouring links via two subelement references (one on the parent, one on the child). The two references are joined BY THEIR CENTERS: the child is moved so that the center of ``ref2`` coincides with the center of ``ref1``.

    A link hosting a **camera or similar forward-looking** sensor (a "sensor
    link") must NOT be joined with this tool — it needs the sensor-specific
    orientation applied by
    ``set_placement_between_sensor_link_and_parent``. A link hosting
    **only a lidar mounted horizontally** to scan all around IS positioned with
    this tool (its parent joint is then turned about Z to aim the sector)."""

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        target_obj = _resolve_object(target)
        # A link that hosts a forward-looking sensor (a "sensor link", camera or
        # similar) must NOT be joined with this tool: it needs the
        # sensor-specific orientation applied by
        # set_placement_between_sensor_link_and_parent. A link that
        # hosts only lidar(s) IS positioned with this tool (see the docstring).
        non_lidar_sensors = [
            child for child in _link_sensors(target_obj)
            if not _is_lidar_sensor(child)
        ]
        if is_link(target_obj) and non_lidar_sensors:
            raise RuntimeError(
                f'"{target}" hosts a forward-looking sensor (a camera or '
                'similar) and is therefore a sensor link. Such a sensor link '
                'must be connected to its parent link ONLY with '
                'set_placement_between_sensor_link_and_parent, never '
                'with set_placement_between. (A link hosting only a lidar '
                'mounted horizontally to scan around is the exception and IS '
                'positioned with set_placement_between.)',
            )
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
# Controllers and broadcasters (ros2_control)
# ---------------------------------------------------------------------------


def _resolve_controller(name: str) -> fc.DocumentObject:
    obj = _resolve_object(name)
    if not (is_controller(obj) or is_broadcaster(obj)):
        raise RuntimeError(f'Object "{name}" is not a Cross::Controller or Cross::Broadcaster.')
    return obj


def _find_controller_entry(
    type_name: str,
) -> tuple[Optional[dict], Optional[str]]:
    """Resolve a controller type, returning ``(controller_data, kind)``.

    ``type_name`` may be the ROS controller name (e.g. ``diff_drive_controller``)
    or the plugin class full name. ``kind`` is ``'controllers'`` or
    ``'broadcasters'`` (i.e. one of the ``get_controllers_data()`` keys).
    ``(None, None)`` is returned when nothing matches.
    """
    data = get_controllers_data()
    kinds = ('controllers', 'broadcasters')
    for kind in kinds:
        if type_name in data.get(kind, {}):
            return data[kind][type_name], kind
    for kind in kinds:
        for controller_data in data.get(kind, {}).values():
            if controller_data['controller_plugin_class_name'] == type_name:
                return controller_data, kind
    # Tolerate a "name (controller)" / "name (broadcaster)" suffix.
    base = type_name.replace(' (controller)', '').replace(' (broadcaster)', '')
    if base != type_name:
        return _find_controller_entry(base)
    return None, None


def _find_controller_data(type_name: str) -> Optional[dict]:
    """Resolve a controller type by ROS name or plugin class name."""
    return _find_controller_entry(type_name)[0]


def _controller_info(obj: fc.DocumentObject) -> dict[str, Any]:
    """Return ``_obj_basic_info(obj)`` plus controller/broadcaster details."""
    info = _obj_basic_info(obj)
    info['Kind'] = 'broadcaster' if is_broadcaster(obj) else 'controller'
    info['PluginClassName'] = getattr(obj, 'plugin_class_name', '')
    info['ControllerName'] = ros_name(obj)
    return info


def _iter_controller_parameters(controller_data: dict) -> Iterator[dict[str, Any]]:
    """Yield the flattened leaf parameters of a controller with metadata."""
    for full_name, param in controller_data.get('parameters_flatten', {}).items():
        if not isinstance(param, dict) or 'type_fc' not in param:
            continue
        yield {
            'full_name': full_name,
            'type_fc': param.get('type_fc', ''),
            'type': param.get('type', ''),
            'default_value': param.get('default_value'),
            'description': param.get('description', ''),
        }


def _coerce_parameter_value(prop_type: str, value: Any) -> Any:
    """Coerce a JSON value to the Python type expected by a FreeCAD property."""
    if prop_type in ('App::PropertyLinkGlobal',):
        return _resolve_object(value) if isinstance(value, str) else value
    if prop_type in ('App::PropertyLinkListGlobal',):
        if isinstance(value, str):
            value = [value]
        return [_resolve_object(v) if isinstance(v, str) else v for v in value]
    if prop_type == 'App::PropertyBool':
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)
    if prop_type == 'App::PropertyInteger':
        return int(value)
    if prop_type == 'App::PropertyFloat':
        return float(value)
    if prop_type == 'App::PropertyVector':
        if isinstance(value, str):
            return fc.Vector(*[float(v) for v in value.replace(',', ' ').split()])
        return fc.Vector(*value)
    if prop_type == 'App::PropertyPosition':
        if isinstance(value, str):
            return fc.Vector(*[float(v) for v in value.replace(',', ' ').split()])
        return fc.Vector(*value)
    if prop_type in (
        'App::PropertyString',
        'App::PropertyStringList',
        'App::PropertyIntegerList',
        'App::PropertyFloatList',
        'App::PropertyBoolList',
    ):
        return value
    return value


def list_controller_types() -> dict[str, Any]:
    """List the available controllers and broadcasters (ros2_control) to add."""

    def _impl() -> dict[str, Any]:
        data = get_controllers_data()

        def _summarize(kind: str) -> list[dict[str, Any]]:
            result = []
            for name, controller_data in data.get(kind, {}).items():
                result.append({
                    'name': name,
                    'description': controller_data.get('description', ''),
                    'plugin_class_name': controller_data.get('controller_plugin_class_name', ''),
                    'parameter_count': len(
                        [p for p in _iter_controller_parameters(controller_data)],
                    ),
                })
            return sorted(result, key=lambda item: item['name'])

        return {
            'controllers': _summarize('controllers'),
            'broadcasters': _summarize('broadcasters'),
        }

    return run_on_main_thread(_impl)


def create_controller(
    robot: str,
    type: str,
    name: str = '',
    kind: str = 'controller',
) -> dict[str, Any]:
    """Add a controller (or broadcaster) to a robot.

    ``type`` is the ROS controller name (e.g. ``joint_state_broadcaster``,
    ``diff_drive_controller``) or its plugin class full name. ``kind`` is
    ``'controller'`` or ``'broadcaster'``. ``name`` overrides the Label of the
    created object.
    """

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        robot_obj = _resolve_robot(robot)
        controller_data, data_kind = _find_controller_entry(type)
        if controller_data is None:
            raise RuntimeError(
                f'Controller type "{type}" not found. '
                'Use list_controller_types() to see the available types.',
            )
        is_broadcaster_type = kind == 'broadcaster' or data_kind == 'broadcasters'
        with _transaction(doc, 'MCP: create controller'):
            if is_broadcaster_type:
                controller = make_broadcaster(controller_data, doc=doc)
            else:
                controller = make_controller(controller_data, doc=doc)
            if controller is None:
                raise RuntimeError(
                    f'Failed to create controller of type "{type}".',
                )
            if name:
                controller.Label = name
            robot_obj.addObject(controller)
        return _controller_info(controller)

    return run_on_main_thread(_impl)


def list_controller_parameters(controller: str) -> dict[str, Any]:
    """List the configurable parameters of a controller on a robot."""

    def _impl() -> dict[str, Any]:
        controller_obj = _resolve_controller(controller)
        controller_data = _find_controller_data(getattr(controller_obj, 'plugin_class_name', ''))
        info = _controller_info(controller_obj)
        parameters = []
        if controller_data is not None:
            for param in _iter_controller_parameters(controller_data):
                full_name = param['full_name']
                entry = dict(param)
                if hasattr(controller_obj, full_name):
                    try:
                        entry['current_value'] = getattr(controller_obj, full_name)
                    except Exception:  # noqa: BLE001
                        entry['current_value'] = None
                parameters.append(entry)
        info['parameters'] = parameters
        return info

    return run_on_main_thread(_impl)


def set_controller_parameter(
    controller: str,
    parameter: str,
    value: Any,
) -> dict[str, Any]:
    """Set a single parameter of a controller on a robot.

    ``parameter`` is the full parameter name (nested names are joined with
    ``'___'``). For link/joint parameters pass the object name (Name or Label)
    or a list of names. Distance parameters are in **metres** unless the
    parameter's description says otherwise — convert a model measurement to the
    parameter's unit before setting it (check the ``description`` from
    ``list_controller_parameters``).
    """

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        controller_obj = _resolve_controller(controller)
        if not hasattr(controller_obj, parameter):
            raise RuntimeError(
                f'Parameter "{parameter}" not found on "{controller}". '
                'Use list_controller_parameters() to see the available parameters.',
            )
        prop_type = controller_obj.getTypeIdOfProperty(parameter)
        with _transaction(doc, 'MCP: set controller parameter'):
            setattr(controller_obj, parameter, _coerce_parameter_value(prop_type, value))
        return {
            'status': 'ok',
            'controller': controller_obj.Name,
            'parameter': parameter,
            'property_type': prop_type,
        }

    return run_on_main_thread(_impl)


def set_controller_parameters(
    controller: str,
    values: dict[str, Any],
) -> dict[str, Any]:
    """Set several parameters of a controller on a robot.

    ``values`` maps parameter full names to values.
    """

    def _impl() -> dict[str, Any]:
        doc = _active_doc()
        controller_obj = _resolve_controller(controller)
        missing = [name for name in values if not hasattr(controller_obj, name)]
        if missing:
            raise RuntimeError(
                'Parameters not found on "' + controller + '": '
                + ', '.join(missing) + '. '
                'Use list_controller_parameters() to see the available parameters.',
            )
        with _transaction(doc, 'MCP: set controller parameters'):
            for name, value in values.items():
                prop_type = controller_obj.getTypeIdOfProperty(name)
                setattr(controller_obj, name, _coerce_parameter_value(prop_type, value))
        return {'status': 'ok', 'controller': controller_obj.Name, 'values': values}

    return run_on_main_thread(_impl)


def get_robot_controllers(robot: str) -> dict[str, Any]:
    """List the controllers and broadcasters present on a robot."""

    def _impl() -> dict[str, Any]:
        robot_obj = _resolve_robot(robot)
        controllers = [
            _controller_info(obj) for obj in (robot_obj.Proxy.get_controllers() or [])
        ]
        broadcasters = [
            _controller_info(obj) for obj in (robot_obj.Proxy.get_broadcasters() or [])
        ]
        return {
            'robot': robot_obj.Name,
            'controllers': controllers,
            'broadcasters': broadcasters,
        }

    return run_on_main_thread(_impl)


# ---------------------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------------------


#: Glue used to join the parts of a nested sensor parameter full name.
_SENSOR_PARAM_GLUE = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE


def _link_sensors(link_obj: fc.DocumentObject) -> list[fc.DocumentObject]:
    """Return the sensors hosted by a link (a "sensor link").

    A link-attached sensor is added to the link's ``Group`` by
    ``create_sensor``; ``OutList`` is checked as a fallback for documents
    written by older versions.
    """
    sensors = [
        child for child in list(getattr(link_obj, 'Group', []) or [])
        if is_sensor(child)
    ]
    if sensors:
        return sensors
    return [
        child for child in list(getattr(link_obj, 'OutList', []) or [])
        if is_sensor(child)
    ]


def _resolve_sensor(name: str) -> fc.DocumentObject:
    """Resolve a sensor by Name/Label, or by the link that hosts it.

    Agents naturally pass the **sensor link** (e.g. ``l_camera001``) rather
    than the sensor object itself, so a ``Cross::Link`` is accepted and its
    hosted sensor is returned. A forward-looking (non-lidar) sensor is
    preferred when the link hosts several.
    """
    obj = _resolve_object(name)
    if is_sensor(obj):
        return obj
    if is_link(obj):
        sensors = _link_sensors(obj)
        if sensors:
            forward_looking = [s for s in sensors if not _is_lidar_sensor(s)]
            return (forward_looking or sensors)[0]
        raise RuntimeError(
            f'Link "{name}" hosts no sensor. Create one first with '
            f'create_sensor("{name}", "<sensor type>"); see '
            'list_sensor_types() for the available types.',
        )
    raise RuntimeError(
        f'Object "{name}" is not a Cross::Sensor (nor a Cross::Link hosting '
        'one). Pass the sensor object or its sensor link.',
    )


def _find_sensor_entry(
    type_name: str,
) -> tuple[Optional[dict], Optional[str]]:
    """Resolve a sensor type, returning ``(sensor_data, sensor_dir_name)``.

    ``sensor_dir_name`` is the directory a sensor can be attached to:
    ``'link'`` or ``'joint'`` (and any future directory such as ``'robot'``).
    ``(None, None)`` is returned when nothing matches.
    """
    data = get_sensors_data()
    for sensor_dir_name, sensor_dir in data.items():
        if not isinstance(sensor_dir, dict):
            continue
        for sensor_name, sensor_data in sensor_dir.items():
            if sensor_name == type_name:
                return sensor_data, sensor_dir_name
    # Tolerate a "name (link)" / "name (joint)" suffix.
    base = type_name
    for suffix in (' (link)', ' (joint)', ' (robot)'):
        base = base.replace(suffix, '')
    if base != type_name:
        return _find_sensor_entry(base)
    return None, None


def _find_sensor_data(type_name: str) -> Optional[dict]:
    """Resolve a sensor type by name."""
    return _find_sensor_entry(type_name)[0]


def _sensor_info(obj: fc.DocumentObject) -> dict[str, Any]:
    """Return ``_obj_basic_info(obj)`` plus sensor details."""
    info = _obj_basic_info(obj)
    info['SensorName'] = ros_name(obj)
    info['SensorType'] = getattr(obj, 'attr_type', '')
    # ``is_sensor_link``/``is_sensor_joint`` also recognise objects created
    # with ``fpo`` (``Proxy.Type``), not only the old ``_Type`` property.
    info['IsLinkSensor'] = is_sensor_link(obj)
    info['IsJointSensor'] = is_sensor_joint(obj)
    parts = []
    for parent in getattr(obj, 'InList', []):
        if is_link(parent) or is_joint(parent) or is_robot(parent):
            parts.append({'name': parent.Name, 'type': getattr(parent, '_Type', '')})
    info['AttachedTo'] = parts
    return info


def list_sensor_types() -> dict[str, Any]:
    """List the available sensors to add, grouped by the element they attach to."""

    def _impl() -> dict[str, Any]:
        data = get_sensors_data()

        def _summarize(sensor_dir: dict) -> list[dict[str, Any]]:
            result = []
            for name, sensor_data in sensor_dir.items():
                if not isinstance(sensor_data, dict):
                    continue
                result.append({
                    'name': name,
                    'type': sensor_data.get('type', ''),
                    'description': sensor_data.get('description', ''),
                })
            return sorted(result, key=lambda item: item['name'])

        return {
            sensor_dir_name: _summarize(sensor_dir)
            for sensor_dir_name, sensor_dir in data.items()
            if isinstance(sensor_dir, dict)
        }

    return run_on_main_thread(_impl)


def create_sensor(
    element: str,
    type: str,
    name: str = '',
) -> dict[str, Any]:
    """Add a sensor to a link, joint or robot.

    ``element`` is the Name or Label of the parent ``Cross::Link``,
    ``Cross::Joint`` or ``Cross::Robot``. On a **joint** the sensor needs no
    position (it is oriented by the joint). On a **link** it becomes a child
    "sensor link" that must ALREADY be connected to the parent link by a joint
    (build the chain first with ``create_links_filled``/``create_link`` +
    ``create_joint``), then positioned with
    ``set_placement_between_sensor_link_and_parent`` (camera / vertical lidar)
    or ``set_placement_between`` + a Z rotation of its parent joint (horizontal
    lidar). If the sensor link already exists, do NOT create it again.
    """
    doc = _active_doc()

    def _impl() -> dict[str, Any]:
        parent = _resolve_object(element)
        if not (is_link(parent) or is_joint(parent) or is_robot(parent)):
            raise RuntimeError(
                f'Object "{element}" is not a Cross::Link, Cross::Joint or '
                'Cross::Robot.',
            )
        sensor_data, sensor_dir_name = _find_sensor_entry(type)
        if sensor_data is None:
            raise RuntimeError(
                f'Sensor type "{type}" not found. '
                'Use list_sensor_types() to see the available types.',
            )
        if sensor_dir_name == 'link' and not is_link(parent):
            raise RuntimeError(
                f'Sensor "{type}" attaches to a link; "{element}" is not a '
                'Cross::Link. Select a link (the sensor will be its child '
                'sensor link), or choose a joint sensor type.',
            )
        if sensor_dir_name == 'joint' and not is_joint(parent):
            raise RuntimeError(
                f'Sensor "{type}" attaches to a joint; "{element}" is not a '
                'Cross::Joint. Select a joint, or choose a link sensor type.',
            )
        with _transaction(doc, 'MCP: create sensor'):
            sensor = make_sensor(sensor_data, sensor_dir_name, doc=doc)
            if sensor is None:
                raise RuntimeError(
                    f'Failed to create sensor of type "{type}".',
                )
            if name:
                sensor.Label = name
            parent.addObject(sensor)
        return _sensor_info(sensor)

    return run_on_main_thread(_impl)


def set_placement_between_sensor_link_and_parent(
    sensor: str,
    ref_parent: str,
    ref_sensor: str,
) -> dict[str, Any]:
    """Position a **link**-attached forward-looking sensor by connecting it to its parent link.

    ``sensor`` is the sensor object **or the sensor link that hosts it** (e.g.
    ``l_camera001``) — both are accepted.

    The ONLY tool that may join a **camera (or similar forward-looking)**
    sensor link to its parent link — never ``set_placement_between`` for that.
    Mirrors the "Set placement - sensor" tool. ``ref_parent`` is a
    face/edge/vertex/circle reference on the parent link, ``ref_sensor`` the
    matching reference on the sensor link; the two are joined BY THEIR CENTERS.

    ORIENTATION: the sensor link's **parent joint** is turned so that the
    sensor's front follows the joint's local X (red arrow) = the look
    direction. The ROLL about X is NOT set: you MUST still rotate the joint
    about X until Z (blue) points perpendicularly UP relative to the parent
    part (skip it and the image lies on its side). Verify with
    ``get_snapshot('<joint>')`` and check **ALL THREE axes — X, Y and Z — one
    by one**: X = look direction, Z = UP relative to the parent part, Y = the
    frame's LEFT seen from behind (a wrong Y means the frame is rolled about
    the look axis). If the sensor looks the OPPOSITE way, verify with
    ``check_overlap`` and rotate the joint 180 deg about Y.

    Joint sensors need no position and are rejected here. EXCEPTION: a
    **horizontal lidar** is NOT positioned with this tool — use
    ``set_placement_between`` + a Z rotation of its parent joint.
    """
    doc = _active_doc()

    def _impl() -> dict[str, Any]:
        sensor_obj = _resolve_sensor(sensor)
        if is_sensor_joint(sensor_obj):
            raise RuntimeError(
                f'Sensor "{sensor_obj.Name}" is attached to a joint and needs '
                'no position; this tool only applies to link sensors.',
            )
        if not is_sensor_link(sensor_obj):
            raise RuntimeError(
                f'Object "{sensor_obj.Name}" is not a link sensor '
                '(Cross::SensorLink); this tool only applies to link sensors.',
            )
        fcgui.Selection.clearSelection()
        ref1_info = _select_reference(doc, ref_parent)
        ref2_info = _select_reference(doc, ref_sensor)
        with _transaction(doc, 'MCP: set sensor position'):
            result = set_placement_sensor()
            if result is False:
                raise RuntimeError(
                    'set_placement_sensor failed. Check the selection order '
                    '(parent reference first, sensor reference second) and '
                    'that the sensor link and the parent link are connected '
                    'by a joint.',
                )
            joint = result[0] if isinstance(result, tuple) else None
        fcgui.Selection.clearSelection()
        sensor_link = next(
            (o for o in getattr(sensor_obj, 'InList', []) if is_link(o)),
            None,
        )
        return {
            'status': 'ok',
            'sensor': sensor_obj.Name,
            'sensor_link': sensor_link.Name if sensor_link is not None else '',
            'joint': joint.Name if joint is not None else '',
            'ref_parent': ref1_info,
            'ref_sensor': ref2_info,
        }

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
        controllers = []
        broadcasters = []
        sensors = []
        others = []
        for obj in doc.Objects:
            if is_robot(obj):
                robots.append(_obj_basic_info(obj))
            elif is_link(obj):
                info = _obj_basic_info(obj)
                reference = _reference_paths(obj)
                if reference:
                    info['Reference'] = reference
                links.append(info)
            elif is_joint(obj):
                info = _obj_basic_info(obj)
                info['Parent'] = getattr(obj, 'Parent', '')
                info['Child'] = getattr(obj, 'Child', '')
                info['Type'] = getattr(obj, 'Type', '')
                joints.append(info)
            elif is_controller(obj):
                controllers.append(_controller_info(obj))
            elif is_broadcaster(obj):
                broadcasters.append(_controller_info(obj))
            elif is_sensor(obj):
                sensors.append(_sensor_info(obj))
            else:
                others.append(_obj_basic_info(obj))
        return {
            'document': doc.Name,
            'robots': robots,
            'links': links,
            'joints': joints,
            'controllers': controllers,
            'broadcasters': broadcasters,
            'sensors': sensors,
            'other_objects': others,
        }

    return run_on_main_thread(_impl)


# ---------------------------------------------------------------------------
# Geometry inspection helpers
# ---------------------------------------------------------------------------


def _vector_to_list(vector: fc.Vector) -> list[float]:
    """Convert a FreeCAD vector to a JSON-serializable ``[x, y, z]`` list."""
    return [vector.x, vector.y, vector.z]


def _round_floats(value: Any, ndigits: int = 2) -> Any:
    """Recursively round every float in ``value`` to ``ndigits`` decimals.

    Walks dicts, lists and tuples and rounds all ``float`` leaves, so the
    output of the inspection tools stays compact and readable. Non-float
    values (strings, ints, bools, ``None``) are returned unchanged.
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, float):
        return round(value, ndigits)
    if isinstance(value, dict):
        return {key: _round_floats(item, ndigits) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_round_floats(item, ndigits) for item in value]
    return value


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


#: Linear tolerance (mm) for the overlap check. A shared face (touch) produces
#: a zero-thickness intersection and must NOT count as an overlap.
_OVERLAP_LINEAR_TOL = 1e-6

#: Volume tolerance (mm^3). Intersections with a volume below this are
#: considered a mere touch (shared face/edge/vertex), not a real overlap.
_OVERLAP_VOLUME_TOL = 1e-6


def _placed_shape_and_ranges(
    obj: fc.DocumentObject,
) -> tuple[Optional[Any], Optional[dict[str, list[float]]]]:
    """Return ``(shape, ranges)`` for ``obj`` in global coordinates.

    ``shape`` is the object's shape (the ``Real`` element's shape for a
    ``Cross::Link``) in global coordinates and is used for the boolean solid
    test. ``ranges`` maps ``'x'``/``'y'``/``'z'`` to the global ``[min, max]``
    of the shape and is used for the axis-interval test.

    Both values are read from the document's own geometry — never from cached
    or assumed values. Returns ``(None, None)`` when the object has no shape.
    """
    shape_obj = _resolve_shape_object(obj)
    if shape_obj is None:
        return None, None
    shape = shape_obj.Shape
    # ``shape`` is already placed by ``shape_obj.Placement``; the ancestor
    # transform maps it to global coordinates (identity for the Real element
    # of a robot link; see ``_ancestor_placement``).
    ancestor = _ancestor_placement(shape_obj)
    bbox = shape.BoundBox
    corners = [
        fc.Vector(x, y, z)
        for x in (bbox.XMin, bbox.XMax)
        for y in (bbox.YMin, bbox.YMax)
        for z in (bbox.ZMin, bbox.ZMax)
    ]
    placed = [ancestor.multVec(corner) for corner in corners]
    ranges = {
        'x': [min(p.x for p in placed), max(p.x for p in placed)],
        'y': [min(p.y for p in placed), max(p.y for p in placed)],
        'z': [min(p.z for p in placed), max(p.z for p in placed)],
    }
    global_shape = shape.copy()
    try:
        if not ancestor.isIdentity():
            global_shape.transformShape(ancestor.Matrix)
    except Exception:  # noqa: BLE001
        pass
    return global_shape, ranges


def _axis_overlap(
    child_range: list[float],
    parent_range: list[float],
    tol: float,
) -> dict[str, Any]:
    """Return the overlap of two closed intervals on a single axis."""
    lo = max(child_range[0], parent_range[0])
    hi = min(child_range[1], parent_range[1])
    extent = max(0.0, hi - lo)
    return {
        'child': list(child_range),
        'parent': list(parent_range),
        'overlap_interval': [lo, hi],
        'overlap_extent': extent,
        'overlap': extent > tol,
    }


def _overlap_between(
    parent_obj: fc.DocumentObject,
    child_obj: fc.DocumentObject,
    tol: float = _OVERLAP_LINEAR_TOL,
) -> dict[str, Any]:
    """Deterministically test whether two shapes overlap in space.

    The decision is the **boolean intersection volume** of the two solids
    (``child.common(parent).Volume``), computed inside FreeCAD from the
    document geometry — not an agent-supplied value and not a bounding-box
    containment proxy. The per-axis AABB intervals and the minimum distance are
    returned as diagnostics. A shared face/edge/vertex (touch) yields a ~zero
    volume and is reported as **no overlap**.
    """
    parent_shape, parent_ranges = _placed_shape_and_ranges(parent_obj)
    child_shape, child_ranges = _placed_shape_and_ranges(child_obj)
    if parent_shape is None or child_shape is None:
        raise RuntimeError(
            f'Both "{parent_obj.Name}" and "{child_obj.Name}" must have shape '
            'geometry (a Cross::Link needs a Real element).',
        )
    axes: dict[str, Any] = {}
    overlap_axes: list[str] = []
    for axis in ('x', 'y', 'z'):
        info = _axis_overlap(child_ranges[axis], parent_ranges[axis], tol)
        axes[axis] = info
        if info['overlap']:
            overlap_axes.append(axis)
    aabb_overlap = len(overlap_axes) == 3
    common_volume = 0.0
    solid_test_ok = True
    try:
        common_volume = child_shape.common(parent_shape).Volume
    except Exception:  # noqa: BLE001
        solid_test_ok = False
    min_distance = None
    try:
        min_distance = child_shape.distToShape(parent_shape)[0]
    except Exception:  # noqa: BLE001
        min_distance = None
    if solid_test_ok:
        overlap = common_volume > _OVERLAP_VOLUME_TOL
    else:
        # Fall back to the AABB test only if the solid test failed.
        overlap = aabb_overlap and (min_distance is not None and min_distance <= tol)
    return {
        'parent': parent_obj.Name,
        'child': child_obj.Name,
        'overlap': bool(overlap),
        'common_volume': common_volume,
        'solid_test_ok': solid_test_ok,
        'aabb_overlap': aabb_overlap,
        'overlap_axes': overlap_axes,
        'min_distance': min_distance,
        'tolerance': tol,
        'axes': axes,
    }


def _face_info(
    face: Any, index: int, ancestor_placement: fc.Placement,
) -> dict[str, Any]:
    """Return the spatial description of a single face (global coordinates).

    The face is identified by its 1-based ``index`` (matching FreeCAD's
    ``Face1``, ``Face2``, ...). The description contains the centre of mass,
    the surface type, the area and the coordinates of the face's vertices, so
    the agent can understand where the face lies on the object's surface.
    ``ancestor_placement`` is applied to the shape's points (which are already
    placed by the object's own placement) to reach global coordinates.
    """
    info: dict[str, Any] = {
        'index': index,
        'name': f'Face{index}',
        'area': face.Area,
        'center_of_mass': _vector_to_list(
            ancestor_placement.multVec(face.CenterOfMass),
        ),
        'orientation': str(face.Orientation),
        'vertices': [
            _vertex_info(v, i + 1, ancestor_placement)
            for i, v in enumerate(face.Vertexes)
        ],
    }
    surface = getattr(face, 'Surface', None)
    surface_type = surface.__class__.__name__ if surface is not None else 'Unknown'
    info['surface_type'] = surface_type
    # Surface-specific parameters, transformed to global coordinates.
    try:
        if surface_type == 'Cylinder':
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
        'local_placement': _placement_to_dict(shape_obj.Placement),
        'ancestor_placement': _placement_to_dict(ancestor_placement),
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


def _object_info_impl(
    object_name: str,
    max_internal_geometry_items: int,
) -> dict[str, Any]:
    """Describe a single object (raw logic, runs on the main thread)."""
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
        # Exact subelement-reference prefix for set_placement_between.
        reference = _reference_paths(obj)
        if reference:
            info['Reference'] = reference
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
    return _round_floats(info)


def get_objects_info(
    object_names: list[str],
    max_internal_geometry_items: int = 1000,
) -> dict[str, Any]:
    """Return detailed information for several objects in ONE call.

    ``object_names`` is a list of Names or Labels. Each entry is resolved and
    described (basic info, ``Group``, and — for links/joints — Real/Visual/
    Collision/Mass/Reference or Parent/Child/limits, plus a ``Geometry`` entry
    with the global-coordinate description of every face and vertex) and the
    descriptions are merged into one result. ALWAYS use this batch tool instead
    of looking up objects one by one.

    Returns ``{'status': 'ok', 'objects': {<name>: <info>, ...}}``. A name that
    cannot be resolved does not abort the call: it is collected under ``errors``
    as ``{<name>: <message>}``.
    """

    def _impl() -> dict[str, Any]:
        objects: dict[str, Any] = {}
        errors: dict[str, str] = {}
        for name in object_names:
            try:
                objects[name] = _object_info_impl(name, max_internal_geometry_items)
            except Exception as exc:  # noqa: BLE001
                errors[name] = str(exc)
        result: dict[str, Any] = {'status': 'ok', 'objects': objects}
        if errors:
            result['errors'] = errors
        return result

    return run_on_main_thread(_impl)


# ---------------------------------------------------------------------------
# Overlap-check tool
# ---------------------------------------------------------------------------


def check_overlap(
    parent: str,
    child: str,
    tolerance: float = _OVERLAP_LINEAR_TOL,
) -> dict[str, Any]:
    """Deterministically test whether two objects/links overlap in space.

    PRIMARY verification tool for the mandatory overlap check. The decision is
    the boolean intersection volume of the two solids, computed inside FreeCAD
    from the document's own geometry — the agent must read the returned
    ``overlap`` boolean, never supply or assume the values. A shared face (the
    parts merely touching, ``common_volume`` ~ 0) is reported as no overlap.

    ``parent`` and ``child`` are object/link names or Labels; each may be a
    robot ``Cross::Link`` (its ``Real`` geometry is used) or any object with a
    ``Shape``. The result also contains the per-axis AABB intervals
    (``axes.x/y/z``) and ``min_distance`` for diagnostics.
    """

    def _impl() -> dict[str, Any]:
        parent_obj = _resolve_object(parent)
        child_obj = _resolve_object(child)
        result = _overlap_between(parent_obj, child_obj, tolerance)
        result['status'] = 'overlap' if result['overlap'] else 'ok'
        return _round_floats(result)

    return run_on_main_thread(_impl)


# ---------------------------------------------------------------------------
# Snapshot tool
# ---------------------------------------------------------------------------


def _add_joint_axis_labels(
        obj: fc.DocumentObject,
        rotation: Any = None,
        view_direction: Optional[fc.Vector] = None,
) -> list[tuple[Any, Any]]:
    """Annotate the joint's axes with ``X``/``Y``/``Z`` labels.

    The labels are added to the joint's markers node (the same node the axis
    arrows are drawn into), so they inherit the joint's visibility and the
    selection highlight: a label therefore gets exactly the color of the arrow
    it annotates, including the selection palette used by ``get_snapshot``.

    The label height is proportional to the joint's size, taken as its
    ``AxisLength`` (the length of the Z arrow). Each label sits near the tip of
    its arrow, pushed sideways off the axis so the letter does not overlap the
    arrow itself.

    ``rotation`` is the orientation of the text plane (a ``FreeCAD.Rotation``
    or an ``SbRotation``). Pass the camera's orientation so the letters face
    the viewer and stay upright instead of being tilted.

    ``view_direction`` is the camera's view direction. It is used for the
    sideways offset: the label is moved along the direction perpendicular to
    both the axis and the view direction, which lies in the image plane, so the
    letter appears beside the arrow on screen.

    Return the list of ``(parent_node, label_node)`` pairs so the caller can
    remove the labels again (see ``_remove_joint_axis_labels``).
    """
    from ..coin_utils import text_3d

    vobj = getattr(obj, 'ViewObject', None)
    if vobj is None:
        return []
    # No arrows are drawn (and no labels are needed) when the axes are hidden.
    if not (getattr(vobj, 'Visibility', False) and getattr(vobj, 'ShowAxis', False)):
        return []
    axis_length = getattr(vobj, 'AxisLength', None)
    length = axis_length.Value if axis_length is not None else 0.0
    if not length:
        return []
    placement = getattr(obj, 'Placement', None)
    if placement is None:
        return []
    # Same colors as the arrows in ``joint_proxy._ViewProviderJoint.draw``.
    if getattr(obj, 'Type', '') == 'fixed':
        z_color = (0.0, 0.0, 0.7)
    else:
        z_color = (0.0, 0.0, 1.0)
    # The label height is proportional to the joint's size (its axis length).
    size = length * 0.12
    # The text is centered on its position, so pull it back by half its height
    # (plus a small margin) to keep the whole letter INSIDE the axis extent.
    inset = size * 0.6
    # Push the label sideways off the arrow: the letter is placed on a line
    # parallel to the axis, offset by just over its own half-size, so it clears
    # the shaft and the cone while staying close to the axis.
    side_offset = size * 0.85
    # (label, local axis direction, local position, color)
    axes = (
        ('Z', fc.Vector(0.0, 0.0, 1.0),
         fc.Vector(0.0, 0.0, length - inset), z_color),
        ('X', fc.Vector(1.0, 0.0, 0.0),
         fc.Vector(length / 2.0 - inset, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ('Y', fc.Vector(0.0, 1.0, 0.0),
         fc.Vector(0.0, length / 2.0 - inset, 0.0), (0.0, 1.0, 0.0)),
    )
    # Draw into the markers node (the arrows' node) when available, so the
    # labels follow the standard visibility toggle; fall back to the root node.
    proxy = getattr(vobj, 'Proxy', None)
    parent = getattr(proxy, '_markers', None)
    if parent is None:
        parent = getattr(vobj, 'RootNode', None)
    if parent is None:
        return []
    added: list[tuple[Any, Any]] = []
    for label, local_direction, local_position, color in axes:
        position = placement * local_position
        if view_direction is not None and view_direction.Length > 0.0:
            # Offset the label in the image plane, perpendicular to the arrow as
            # seen on screen: the cross product of the axis and the view
            # direction is perpendicular to both, so the letter ends up beside
            # the arrow instead of on top of it.
            axis_world = placement.Rotation.multVec(local_direction)
            side = axis_world.cross(view_direction)
            if side.Length > 0.0:
                side.normalize()
                position = position + side * side_offset
        node = text_3d(
            label,
            position=position,
            color=color,
            size=size,
            rotation=rotation,
        )
        parent.addChild(node)
        added.append((parent, node))
    return added


def _remove_joint_axis_labels(added: list[tuple[Any, Any]]) -> None:
    """Remove the label nodes added by ``_add_joint_axis_labels``."""
    for parent, node in added:
        try:
            parent.removeChild(node)
        except Exception:  # noqa: BLE001
            pass


def _is_blank_image(path: str) -> bool:
    """Return ``True`` when the saved image is a single flat color.

    Right after FreeCAD starts (or after the 3D view was just modified) the
    first ``saveImage()`` can return an all-white frame because the view has
    not been rendered yet. Such a frame carries no information, so the caller
    retries instead of returning a useless white picture.
    """
    image = QtGui.QImage(path)
    if image.isNull():
        return True
    width = image.width()
    height = image.height()
    if not (width and height):
        return True
    # Sample a coarse grid instead of every pixel: a rendered view always has
    # some variation, a blank one is uniform.
    first = image.pixelColor(0, 0)
    for x in range(0, width, max(1, width // 16)):
        for y in range(0, height, max(1, height // 16)):
            if image.pixelColor(x, y) != first:
                return False
    return True


def _snapshot_content_blocks(
    width: int,
    height: int,
    format: str,
    data: bytes,
) -> list[Any]:
    """Build the MCP content blocks for a captured 3D view.

    A plain ``dict`` return value is JSON-dumped by the MCP SDK into a single
    *text* block (``_convert_to_content``), so a base64 ``data:`` URI would
    reach the agent as text - truncated when large - instead of as an image.
    Returning real content blocks (a ``TextContent`` line with the image size
    plus an ``ImageContent`` carrying the pixels) is what a vision agent can
    actually look at.

    The ``mcp`` SDK is imported on demand: it is a lazily pip-installed
    dependency, so it must not be required at module import time.
    """
    from mcp.types import ImageContent, TextContent

    mime = 'image/png' if format == 'png' else f'image/{format}'
    return [
        TextContent(
            type='text',
            text=f'3D view snapshot: {width}x{height} {format} ({mime}).',
        ),
        ImageContent(
            type='image',
            data=base64.b64encode(data).decode('ascii'),
            mime_type=mime,
        ),
    ]


def get_snapshot(
    width: int = 1024,
    height: int = 768,
    format: str = 'png',
    object_name: str = '',
) -> list[Any]:
    """Capture the active 3D view and return it as MCP content blocks.

    Returns a ``TextContent`` (image dimensions) and an ``ImageContent`` (the
    pixels), so vision agents get an actual image instead of base64. Registered
    *unstructured* (see ``server.py``), so the blocks are forwarded as-is.

    ``object_name`` selects the framing: empty (default) fits the WHOLE scene
    (``ViewFit``); an object's ``Name``/``Label`` (typically a **joint**)
    selects it and fits the view onto the SELECTION (``Std_ViewFitSelection``)
    so its axes are shown up close — use this before correcting a sensor's
    orientation. The view is then always switched to an ISOMETRIC orientation
    before the capture.

    When ``object_name`` is a **joint**, every OTHER joint is hidden for the
    capture (restored right after, success or not) and, because the visibility
    was just changed, the tool waits ONE SECOND (pumping the Qt event loop) so
    the render is up to date.

    AXIS COLORS follow the framing: WITH ``object_name`` (selection palette)
    **Z (longest) = light-blue/cyan, X = pink, Y = turquoise**; with NO
    ``object_name`` **Z (longest) = blue, X = red, Y = green**. Z (up) is always
    the longest arrow.

    When ``object_name`` is a **joint**, its three axes are additionally
    annotated with the letters ``Z``, ``X`` and ``Y``, drawn just past the tip
    of the matching arrow and in the SAME color as that arrow (the labels live
    in the joint's markers node, so they follow the selection highlight too).
    The letter height is proportional to the joint's size (its ``AxisLength``).
    The labels exist only for the capture: they are removed right after it, so
    the joint's axes are back to their normal state.
    """

    def _impl() -> list[Any]:
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
        # Visibility of the joints, saved to restore it after the snapshot.
        saved_joint_visibility: dict[str, bool] = {}
        # Axis labels added for the capture, removed right after it.
        added_axis_labels: list[tuple[Any, Any]] = []
        try:
            # Always switch to an isometric orientation so the snapshot is taken
            # from a stable, readable angle. It is applied AFTER the fit/selection
            # so it keeps the framing just computed and only changes the viewpoint.
            view.viewIsometric()
            # The joint whose axes get labelled, if any.
            joint_obj: Optional[fc.DocumentObject] = None
            if object_name:
                # Frame a single object: select it and fit the camera to the
                # SELECTION. ``ViewSelection`` is what Std_ViewFitSelection
                # sends; ``ViewFit`` would instead fit the whole scene.
                obj = _resolve_object(object_name)
                if is_joint(obj):
                    joint_obj = obj
                    # Show ONLY the requested joint: hide every other joint in
                    # the document so the requested joint's axes are not
                    # obscured. The previous visibility is saved and restored
                    # in the ``finally`` block below.
                    for other in doc.Objects:
                        other_vo = getattr(other, 'ViewObject', None)
                        if other_vo is None or not is_joint(other):
                            continue
                        saved_joint_visibility[other.Name] = other_vo.Visibility
                        other_vo.Visibility = (other.Name == obj.Name)
                fcgui.Selection.clearSelection()
                fcgui.Selection.addSelection(doc.Name, obj.Name)
                fcgui.SendMsgToActiveView('ViewSelection')
            else:
                # Fit the view to the scene so the whole robot is visible.
                fcgui.SendMsgToActiveView('ViewFit')

            # Let the GUI process the view changes / redraw before capturing.
            QtGui.QApplication.processEvents()
            if saved_joint_visibility:
                # The joints' visibility was just changed: wait one second, all
                # the while pumping the Qt event loop, so the 3D view has time to
                # finish rendering before the snapshot. Without this the captured
                # image may still show the previous visibility state.
                deadline = time.monotonic() + 1.0
                while time.monotonic() < deadline:
                    QtGui.QApplication.processEvents()
                    time.sleep(0.05)
            if joint_obj is not None:
                # Annotate the joint's axes with the letters Z, X and Y, in the
                # same color as the arrow each one belongs to. The labels are
                # added to the joint's markers node, so they are highlighted
                # together with the arrows (selection palette) and removed in
                # the ``finally`` block below, restoring the normal state.
                # The camera's own orientation is passed so the letters face the
                # viewer and stay upright (a billboard would tilt them). The view
                # direction is passed too, so each label is pushed sideways off
                # its arrow in the image plane instead of sitting on top of it.
                added_axis_labels = _add_joint_axis_labels(
                    joint_obj,
                    rotation=view.getCameraOrientation(),
                    view_direction=view.getViewDirection(),
                )
                if added_axis_labels:
                    # A ``Cross::Joint`` has no ``Shape``: its axes live only in
                    # the coin markers node, so ``Std_ViewFitSelection`` above
                    # had an empty bounding box and left the camera where it was
                    # (the snapshot then came out blank or zoomed into the
                    # ground). Fit the camera to the joint's coin scene graph
                    # instead, which holds the arrows and the labels just added.
                    camera = view.getCameraNode()
                    if camera is not None and hasattr(camera, 'viewAll'):
                        # ``SoCamera.viewAll`` needs an ``SbViewportRegion``
                        # (the aspect ratio is taken from it). Prefer the real
                        # viewer's region so the framing matches the on-screen
                        # view; fall back to the capture size.
                        viewport_region = None
                        try:
                            viewer = view.getViewer()
                            render_manager = viewer.getSoRenderManager()
                            viewport_region = render_manager.getViewportRegion()
                        except Exception:  # noqa: BLE001 - API may differ.
                            viewport_region = None
                        if viewport_region is None:
                            from pivy import coin

                            viewport_region = coin.SbViewportRegion(width, height)
                        # Fit the ROOT node, not the markers node: the root node
                        # carries the joint's placement transform, so its
                        # bounding box is in world coordinates - the space the
                        # camera works in. The markers node alone would be in
                        # local coordinates and the camera would be misplaced.
                        vobj = getattr(joint_obj, 'ViewObject', None)
                        root = getattr(vobj, 'RootNode', None)
                        target = root if root is not None else added_axis_labels[0][0]
                        camera.viewAll(target, viewport_region)
                    # The scene graph was just modified: pump the Qt event loop
                    # for a moment so the 3D view finishes the redraw before the
                    # capture. A single processEvents() is not enough and the
                    # image may come out blank or without the labels.
                    deadline = time.monotonic() + 1.0
                    while time.monotonic() < deadline:
                        QtGui.QApplication.processEvents()
                        time.sleep(0.05)
            # saveImage() writes to a file, so use a temporary one and read it
            # back.
            fd, save_path = tempfile.mkstemp(suffix=f'.{format}')
            os.close(fd)
            try:
                # saveImage() may return None even on success (it saves the
                # file directly), so verify the file on disk instead of the
                # return value. The first capture after FreeCAD starts (or
                # right after the scene graph was modified) can come out as a
                # flat white frame because the view has not been rendered yet,
                # so retry a few times, pumping the event loop in between.
                data = b''
                for attempt in range(4):
                    view.saveImage(save_path, width, height, 'White')
                    if not os.path.isfile(save_path) or os.path.getsize(save_path) == 0:
                        raise RuntimeError('Failed to capture the 3D view.')
                    with open(save_path, 'rb') as f:
                        data = f.read()
                    if not _is_blank_image(save_path):
                        break
                    # Blank frame: let the 3D view render and try again.
                    deadline = time.monotonic() + 1.0
                    while time.monotonic() < deadline:
                        QtGui.QApplication.processEvents()
                        time.sleep(0.05)
            finally:
                try:
                    # os.remove(save_path)
                    pass
                except OSError:
                    pass
        finally:
            # Remove the axis labels so the joint's axes are back to their
            # normal state (no letters) after the snapshot.
            _remove_joint_axis_labels(added_axis_labels)
            # Restore the joints' visibility to its state before the snapshot.
            for name, visible in saved_joint_visibility.items():
                other = doc.getObject(name)
                other_vo = getattr(other, 'ViewObject', None) if other is not None else None
                if other_vo is not None:
                    other_vo.Visibility = visible
            if saved_joint_visibility or added_axis_labels:
                # Refresh the view so the restored state is shown immediately.
                QtGui.QApplication.processEvents()
        return _snapshot_content_blocks(width, height, format, data)

    try:
        return run_on_main_thread(_impl)
    except Exception as exc:  # noqa: BLE001 - re-raised with the reason below.
        if isinstance(exc, (RuntimeError, ValueError)):
            raise
        # An unexpected failure (for example a ``Base.FreeCADError`` raised by
        # the coin layer) would otherwise reach the agent as a bare
        # "Error executing tool get_snapshot" with no reason at all. Re-raise
        # it as a ``RuntimeError`` so the message is forwarded to the caller
        # (``as_anticipated_error`` turns it into a ``ToolError``).
        import traceback

        raise RuntimeError(
            f'{type(exc).__name__}: {exc}\n{traceback.format_exc()}'
        ) from exc


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

0. PLAN FIRST. Write the plan AS A TODO LIST before touching the document and
   keep it updated: every step below is a separate checklist item (break step 6
   into per-link items), marked DONE right after it is completed. Following this
   algorithm MEANS working through that list in order — never skip, reorder or
   batch items. The plan MUST list every link, the parent/child topology, the
   reference chosen for each placement, and an explicit **mirroring check** per
   child (decide up front whether it may land on the wrong side of its parent
   and need `rotate_object(child_link, 'x', 180)`, see e-bis) — symmetric parts
   on opposite sides of a parent (e.g. left/right wheels) are the usual
   candidates.
1. No active document? `create_document(name)`.
2. `create_robot(name)`.
3. `create_links_filled(robot, object_names)` — links from existing bodies; the
   same body may be reused for several links.
4. Create joints: 2 neighbouring links at a time, root link first.
5. `set_placement_vision_mode(robot)` — show Real, hide Visual/Collision.
6. Position links pairwise along the chain with
   `set_placement_between(target, ref1, ref2)`. Per pair:
   a. Pick one reference on the parent and one on the child: a
      face/edge/vertex/circle of the link Real element, written
      `<real_link>.<inner_link_name>.<feature>.<subelement>` (e.g.
      `real_l_chassis001_.chassis001.Box.Face3`). `<inner_link_name>` is the
      Name of the App::Link inside the Real element (e.g. `chassis001`), NOT the
      source body name and NOT the robot link name. **NEVER guess this path**:
      read `Reference.prefix` from `get_objects_info([link])` and append
      `.FaceN`/`.VertexN`/`.EdgeN`. Links filled from the SAME source body share
      one inner link name (four wheels from one `wheel` body all have
      `wheel001`; `wheel002` is NOT valid). Do NOT create LCS objects — plain
      subelement references are enough. Choose the face/vertex from the
      `Geometry` data (`center_of_mass`, `surface_type`, `vertices`, `point`),
      never from the index alone.
   b. `set_placement_between(target, ref1, ref2)` — only the two references go
      into the selection. If it returns no error but the target did NOT move,
      the two references most likely resolved to the same point: verify with
      `get_objects_info([target])` and pick different references.
   c. Control snapshot: `get_snapshot(...)`.
   d. Orient the JOINT so its local Z lies along the child's functional axis
      (revolute/continuous rotate around local Z, prismatic moves along local
      Z) — ONLY with `rotate_object(joint_name, axis, angle_deg)`. NEVER rotate
      the LINK for this: the link follows the joint, so rotating it breaks the
      alignment (a wheel whose axle is not along the joint's local Z will not
      roll).
   e. Aim the assembly by rotating the JOINT (`rotate_object`): that turns its
      local Z and with it the child chain. Do NOT rotate the link to fix the
      final pose.
   e-ter. For a chassis with several symmetric wheels, ALL wheels' joint local
      Z axes (the axles) MUST be parallel and point the SAME way — to the LEFT.
      Never mirror the joints into opposite directions; flip a wrong one by
      rotating its JOINT.
   e-bis. Mirroring: if the joint and the link are both oriented correctly but
      the link's body penetrates the parent through its full height (it belongs
      on the opposite side), rotate the LINK 180° about X:
      `rotate_object(link_name, 'x', 180)`. The link is aligned along its local
      Z, so an X rotation mirrors it about Z — it flips to the other side while
      its Z alignment and the joint orientation stay intact. Use X, NOT Z
      (a Z rotation would only spin it around its own functional axis).
   e-quater. MANDATORY OVERLAP CHECK — HARD GATE. Run it for EACH child
      individually, ONLY AFTER its joint is oriented (d, e, e-ter) and after
      every later `rotate_object` on that joint (orientation changes the child's
      global ranges), and BEFORE collisions/materials. Never batch or assume
      symmetry. Compare the GLOBAL min/max of child and parent on ALL THREE axes
      X/Y/Z from `get_objects_info(...)` `Geometry` (never one axis, never by
      eye). If the child's range on ANY axis is CONTAINED in the parent's on
      that axis → OVERLAP FAILURE: mirror the child (e-bis) and re-check. PRINT
      per child: `overlap check <child>: X[..] Y[..] Z[..] vs parent X[..] Y[..]
      Z[..] -> OUTSIDE` (or `OVERLAP -> mirrored`). Never proceed until every
      child passes and the numbers are printed.
   f. Verify with another snapshot.
7. Sensors: MANDATORY — never skip. Choose the set by priority: (a) the user
   asked something specific → do exactly that; (b) else the source objects
   already contain specific sensors (a `Cross::Sensor` or a sensor link) → add
   THOSE; (c) else add the DEFAULT set (forward RGB-D camera + 360° lidar on
   the chassis, RGB-D camera on the manipulator aimed at the end tool). Attach
   each sensor to the link/joint it BELONGS to by meaning (never the root link
   by default). If a sensor link ALREADY exists, do NOT recreate it — position
   it. Then verify the sensor FRAME (MANDATORY): X (red) = look direction,
   Z (blue) = perpendicularly UP relative to the parent part, Y (green) = the
   frame's LEFT. See the **Sensors** section.
8. Controllers and broadcasters: choose them from the robot's PURPOSE and its
   ACTUATED joints (see **Controllers and broadcasters**). Actuated joints
   require a `joint_state_broadcaster`; a chassis gets a drive controller suited
   to its wheels; a manipulator/gripper gets the matching trajectory/gripper
   controllers.
9. Collisions: ONE `create_collisions([link1, link2, ...])` call, default
   WITHOUT `type` (type `copy`). A robot name covers the whole robot; its links
   passed in the same call are skipped. Primitive types (`box`, `sphere`,
   `cylinder_x/y/z`) ONLY on explicit user request. Never call it once per link.
10. `set_material(...)`; if not specified, use `ABS-Generic`.
11. `calculate_mass_and_inertia(robot)`.
12. Inspect and verify with `list_scene_objects()`, `get_objects_info(...)`,
    `get_snapshot(...)`. Prefer ONE `get_objects_info([...])` call with many
    names; pass an element (e.g. a sensor's joint) to
    `get_snapshot(object_name=...)` to frame it up close.
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

### Positioning algorithm (step by step)

1. Get the parent and its child links with ONE
   `get_objects_info([parent, child1, child2, ...])` call (Real geometry:
   faces, vertices, `center_of_mass`, `surface_type`).
2. Choose the faces/vertices to place each child on by their geometry, taking
   the link topology into account (e.g. a 4-wheeled chassis: parent = chassis,
   children = the four wheels). A face anchors the attachment at its center of
   mass.
3. Print `parent_robot_link_name face1/vertex1 - child_robot_link_name
   face1/vertex1` naming the parent reference and the child reference that will
   be joined.
4. `set_placement_between` for the parent/child pair.
5. Orient the child's JOINT so its local Z lies along the child's functional
   axis; for a parent with several symmetric wheels make ALL wheels' joint
   local Z axes point the SAME way (left) — see steps d–e-ter of the General
   algorithm.
6. ONLY AFTER step 5, run the MANDATORY OVERLAP CHECK for this child on ALL
   THREE axes X/Y/Z and PRINT the numeric evidence; mirror the child if it is
   contained on any axis. Do not start collisions/materials until every child
   passes.

Reference building, joint orientation, mirroring and the overlap procedure are
specified in the **General algorithm** (source of truth).

- `set_placement_between(target, ref1, ref2, move)` — PRIMARY method: snaps
  two neighbouring links by two references (one on the parent, one on the
  child): a face/edge/vertex/circle of the link Real element as
  `<real_link>.<inner_link_name>.<feature>.<subelement>` (e.g.
  `real_l_chassis001_.chassis001.Box.Face3`) or an existing LCS; a robot link
  (`l_...`) cannot be a reference. `move='leaf'` (default) — only for the final
  chain element; `child_branch`/`parent_tree` are advanced. Only the two
  references go into the selection (the target is not selected).
  CONNECTION MODEL: the references are joined **by their CENTERS** — the child
  moves so that the center of `ref2` coincides with the center of `ref1` (e.g.
  a hole/boss center lands on the mating hole/boss center). Pick ref1/ref2 so
  that the points which must touch are exactly those centers.
  NEVER use it on a **sensor link** hosting a camera or similar forward-looking
  sensor — use `set_placement_between_sensor_link_and_parent` (see the sensors
  topic). EXCEPTION: a link hosting only a **horizontal lidar** IS positioned
  with `set_placement_between` (then rotate its parent joint about Z to aim the
  sector).
- `set_placement_vision_mode(robot)` — show Real, hide Visual/Collision; call
  before positioning.
- `set_placement_between_sensor_link_and_parent(sensor, ref_parent, ref_sensor)`
  — positions a **link**-attached forward-looking sensor (see the sensors
  topic): joins the two links by the centers of the references and turns the
  parent joint so the sensor's front follows its local X (look direction). The
  roll about X must still be corrected afterwards. `sensor` may be the sensor
  object OR the sensor link that hosts it (e.g. `l_camera001`).
- `rotate_object(object_name, axis, angle_deg)` — rotates a joint `Origin`,
  link `MountedPlacement` or LCS; correct orientation after a snapshot.
- `create_lcs(link, subelement)` — LCS on a face/edge/circle/vertex of the
  Real element. ONLY on explicit user request.
""",
    'joints': """\
## Joints

IMPORTANT: a joint always rotates around its **local Z axis** (revolute,
continuous) or translates along its **local Z axis** (prismatic). The joint's
local Z is oriented by rotating the JOINT itself with `rotate_object` — there
is NO `axis` parameter on `create_joint`. See the **General algorithm**
(source of truth) for the orientation and mirroring rules.

- `create_joint(robot, name, parent_link, child_link, type, lower,
  upper, effort, velocity)` — joint between two links; `type`: `fixed`,
  `revolute`, `prismatic`, `continuous`, ...; limits in degrees (revolute) or
  mm (prismatic).
- `create_joints_filled(robot, link_names_in_order, connect_type)` — auto
  joints: `chain` = consecutive links, `spider` = all to the first link.
- `set_joint_values(robot, values)` — joint ROS names (or Labels) -> degrees
  (revolute) or mm (prismatic).
""",
    'collisions': """\
## Collisions

- `create_collisions(link_or_robot, type)` — create collisions for SEVERAL
  links/robots in ONE call. `link_or_robot` is a LIST of link/robot names or
  Labels. `type` is either a single collision type applied to every element or
  a LIST parallel to `link_or_robot` (one type per element). Default WITHOUT
  `type` (type `copy` = exact geometry copy; the only allowed default).
  Primitive types ONLY on explicit user request: `box`, `sphere`,
  `cylinder_x/y/z` (from the bounding box).
  Passing a **robot** covers the robot AS A WHOLE (all its links). When a robot
  and some of its OWN links are passed together, those links are removed from
  the list first and the pass runs over all the robot's links, then over the
  remaining explicitly-listed elements — so no link gets a duplicate collision.
  ALWAYS use this batch tool; never call it once per link.
""",
    'materials': """\
## Materials and inertia

- `set_material(object_name, material_card_path, density, card_name)` —
  `.FCMat` path or material name; `density` in kg/m^3 (optional, from the
  card if omitted).
- `calculate_mass_and_inertia(robot_or_link_names)` — mass, inertia, center
  of mass; robot name or list of link names.
""",
    'controllers': """\
## Controllers and broadcasters (ros2_control)

Give the robot the controllers its PURPOSE and its ACTUATED joints require
(`Cross::Controller` / `Cross::Broadcaster` from `ros2_controllers`). Decide
from the robot's topology and actuated joints — not from a fixed list — and
configure each controller for exactly its functional joint group:

- **Any robot with actuated joints**: a `joint_state_broadcaster`
  (`kind='broadcaster'`), otherwise joint states are not published.
- **Chassis / mobile base**: a drive controller chosen by the wheel layout —
  `diff_drive_controller` for two driven wheels/sides (`left_wheel_names`,
  `right_wheel_names`, wheel separation, radius); `ackermann_steering_controller`
  (or steering + traction) for a car-like base; the matching omni controller for
  mecanum/swerve. Set `wheel_*_names` / `left/right_wheel_names` /
  `steering_*_names` and `base_frame_id` / `odom_frame_id` to the real names.
  - **LEFT/RIGHT wheels**: derive each side from the robot's direction-defining
    parts (forward-looking sensors, end effector), not from an arbitrary axis;
    with no such feature use the base convention (local +X forward, +Y left).
  - **Wheel separation**: the distance between the **nearest (facing inner)
    faces** of the right and left wheel on one axis — not between centers and
    not between outer faces.
- **Manipulator arm**: a `joint_trajectory_controller` over the arm's
  revolute/prismatic joints (`joints`, `base_frame_id`).
- **Gripper / end effector**: `gripper_action_controller` /
  `position_controllers` (or `joint_trajectory_controller`) over the gripper
  joints.
- **Other groups** (pan-tilt, lift, conveyor, ...): a
  `joint_trajectory_controller` (or a suitable velocity/position controller).
- **Parameter units**: distance parameters are in **metres** unless the
  parameter's `description` says otherwise — convert model measurements
  (wheel separation, radius, offsets, limits) before setting them.

### Tools

- `list_controller_types()` — available controllers and broadcasters
  (`name`, `description`, `plugin_class_name`, `parameter_count`).
- `create_controller(robot, type, name, kind)` — `type` is the ROS controller
  name (e.g. `diff_drive_controller`, `joint_state_broadcaster`) or a plugin
  class full name; `kind` = `'controller'` (default) or `'broadcaster'`
  (auto-detected for broadcaster types).
- `get_robot_controllers(robot)` — controllers and broadcasters on a robot.
- `list_controller_parameters(controller)` — `full_name`, `type_fc`, `type`,
  `default_value`, `description`, `current_value`.
- `set_controller_parameter(controller, parameter, value)` — one parameter by
  `full_name`; nested names are joined with `'___'` (e.g.
  `linear__x__has_velocity_limits`). Joint/link parameters (`joints`,
  `left_wheel_names`) accept a Name/Label, a list of names or a ROS name.
- `set_controller_parameters(controller, values)` — several at once
  (`full_name` -> value).

### Typical workflow

1) Robot, links and joints first.
2) Identify the functional joint groups (chassis, arm, gripper, ...).
3) Actuated joints → `create_controller(robot, 'joint_state_broadcaster')`.
4) Per group: `list_controller_types()`, `create_controller(robot, type)`,
   `list_controller_parameters(controller)`, then set `joints` /
   `*_wheel_names` / frame parameters with `set_controller_parameter(...)`.
""",
    'sensors': """\
## Sensors

Adding sensors is a MANDATORY part of the general algorithm — never skip it.
Decide WHICH set to add FIRST; the three cases are mutually exclusive and are
checked in this order:

1. **The user asked something specific** (type, count, placement) → add exactly
   that, INSTEAD of the default set.
2. **The source objects already contain specific sensors** (a `Cross::Sensor`,
   a sensor link, or a plain camera/lidar part) → add THOSE INSTEAD of the
   default set.
3. **The user said nothing about sensors** and you follow the general algorithm
   → add the DEFAULT SENSOR SET below.

A robot hosts `Cross::Sensor` objects (Gazebo sensors from `resources/sensors`),
attached to a `Cross::Link`, `Cross::Joint` or `Cross::Robot`. **A sensor has no
position of its own**: it is oriented by the parent joint of the element it is
attached to (X = look, Z = up, Y = left). **Attach each sensor where it BELONGS
BY MEANING** — a camera on the head/end-tool link, a lidar on the chassis, a
wheel encoder on the wheel's joint, an IMU on the base; never the root link by
default.

### Default sensor set (case 3)

- **RGB-D camera** (`rgbd_camera`) on the chassis, looking FORWARD.
- **360-degree lidar** (`gpu_lidar`, circular scan) on the chassis.
- **RGB-D camera** (`rgbd_camera`) on the manipulator's end (tool-flange) link,
  aimed at the END TOOL.

No manipulator → skip that camera; no chassis → put the chassis sensors on the
base link. Adapt the set to what the robot actually has.

### Positioning an existing sensor link

A **sensor link** is a normal link that also hosts a `Cross::Sensor`. **Before
adding one, CHECK whether it already exists** (its whole chain from the robot
down to it — see the workflow below); if it does, do NOT recreate it, just
POSITION it:

- **Camera / vertical (forward-looking) lidar** →
  `set_placement_between_sensor_link_and_parent(...)` — the ONLY way to connect
  such a sensor link to its parent link; never `set_placement_between`. It turns
  the parent joint so the sensor's front follows its local X (look direction);
  the roll about X must still be corrected (see below).
- **Horizontal lidar** (scans all around) → the EXCEPTION: position it with
  `set_placement_between(...)`, then aim its sector by rotating its **parent
  joint** about **Z** until the sector front (the joint's positive X) points
  where it must.

### Check the sensor frame orientation (X = look, Y = left, Z = up)

After positioning, ALWAYS verify the frame of the sensor's **parent joint** —
MANDATORY and the step most often skipped, leaving the image LYING ON ITS SIDE.
Check **ALL THREE axes — X, Y AND Z — one by one**; do NOT stop after X and Z:

- **X (red) = the look direction**;
- **Z (blue) = the top of the frame** — **perpendicularly UP relative to the
  parent part**, NOT relative to the world and NOT along the look direction;
- **Y (green) = the frame's LEFT** seen when looking at the sensor **from
  behind**. A wrong Y means the frame is rolled about the look axis even when X
  and Z look plausible.

**Axis colors depend on the framing** — identify axes by LENGTH when in doubt
(Z, up, is ALWAYS the longest arrow): **WITH `object_name`** (selection palette)
Z = light-blue/cyan, X = pink, Y = turquoise; **NO `object_name`** (whole scene)
Z = blue, X = red, Y = green.

To judge the triad, zoom in on the sensor's **parent joint**:
`get_snapshot('<joint>')` selects it, fits the view onto it, switches to
ISOMETRIC, hides every OTHER joint for the capture (restored right after) and
waits ONE SECOND (Qt events pumped) so the render is up to date. A snapshot with
no argument fits the whole scene and is too far away to see the arrows.

Corrections — rotate the sensor's **parent joint** (never its link) with
`rotate_object(joint_name, axis, angle_deg)`:

- about **X** — tilts the frame forward/back; fixes Z (up). After
  `set_placement_between_sensor_link_and_parent` this is the roll you almost
  always still need: rotate until Z points perpendicularly up (a sideways blue
  arrow means the image is on its side).
- about **Z** — spins the frame left/right; aims a forward-looking sensor or the
  front of a horizontal lidar's sector.
- about **Y** — rolls the frame; fixes Y (left). If the sensor looks the
  OPPOSITE way, verify with `check_overlap` and rotate **180 deg about Y**.

Rotations combine: aim X with a Z rotation, then bring Z up with an X rotation.
Re-check with `get_snapshot('<joint>')` after each rotation and confirm ALL
THREE axes again before finishing.

### Tools

- `list_sensor_types()` — available sensors grouped by attachment directory
  (`link`, `joint`), each with `name`, `type`, `description`.
- `create_sensor(element, type, name)` — add a sensor to a link, joint or robot.
  On a **joint** it needs no position. On a **link** it becomes a child "sensor
  link" that must already be connected to the parent link by a joint, then
  positioned (see above). If the sensor link ALREADY exists, do NOT create it
  again.
- `set_placement_between_sensor_link_and_parent(sensor, ref_parent, ref_sensor)`
  — position a **link**-attached **forward-looking** sensor (details above).
  `sensor` may be the sensor object OR the sensor link that hosts it (e.g.
  `l_camera001`).
  Joint sensors are rejected; a horizontal lidar uses `set_placement_between` +
  a Z rotation.

### Workflow for a link sensor (e.g. a camera, lidar)

Creating the sensor does NOT build its kinematic structure. BEFORE creating it,
CHECK whether the sensor's chain already exists (the sensor link plus every link
and joint from the robot down to it) with `list_scene_objects()` /
`get_objects_info(...)`. If the source objects (bodies / `App::Link`s) are
present, BUILD the chain from them and only THEN attach the sensor; if they are
absent, say so instead of attaching a sensor that has nothing to connect to.

1) (if missing) build the chain: `create_links_filled(robot, [<object>])` (or
   `create_link(robot, ...)` with no geometry to reuse) for the sensor link and
   any intermediate links, connected with
   `create_joint(robot, name, parent_link=..., child_link=...)`. The sensor link
   must be a proper child of the parent link before you continue.
2) `create_sensor('<sensor link>', '<sensor type>')` — skip if it already exists.
3) Position it by its mounting (see "Positioning an existing sensor link"):
   camera / vertical lidar → `set_placement_between_sensor_link_and_parent(...)`;
   horizontal lidar → `set_placement_between(...)` + a Z rotation of the parent
   joint.
4) Verify the frame with `get_snapshot('<joint>')` and correct it until ALL
   THREE axes (X, Y and Z) are right. Aim until the sensor looks where it must:
   chassis camera forward / lidar all-round, manipulator camera toward the end
   tool.

### Joint sensor

For a joint sensor (e.g. a force-torque sensor) just
`create_sensor('<joint>', 'force_torque')` — no positioning needed.
""",
    'inspection': """\
## Inspection

- `list_scene_objects()` — scene description (robots, links, joints, other
  objects) with placements.
- `get_objects_info(object_names, max_internal_geometry_items)` — details for
  SEVERAL objects in ONE call (a LIST of names/Labels), merged into
  `{'status': 'ok', 'objects': {<name>: <info>, ...}}`; unresolvable names go
  under `errors`. ALWAYS prefer this batch call over repeated single lookups.
  Each entry has the basic info (Real, Visual, Collision, Mass, joint limits,
  ...) plus `Geometry`: every face and vertex in **global** coordinates — a face
  with a 1-based `index` (`Face1`, ...), `center_of_mass`, `surface_type`
  (`Plane`, `Cylinder`, ...), `area` and its `vertices`; a vertex with a 1-based
  `index` and its `point`. For a `Cross::Link` the `Real` element's geometry is
  reported, so the indices are usable directly in positioning references, and a
  `Reference` block gives the exact path (`real_link`, `inner_link`, `feature`,
  `prefix`, `example`): use `Reference.prefix` + `.FaceN`/`.VertexN` and never
  guess the inner link name.
  IMPORTANT: judge the spatial arrangement (which face is on top, which are
  parallel, which vertex is a corner) from `center_of_mass`, `surface_type`,
  `vertices` and `point` — NOT from the index alone, which only identifies the
  subelement. Call `get_objects_info(...)` before choosing references.
- `get_snapshot(width, height, format, object_name)` — 3D view capture as an MCP
  image block (not base64). `object_name` (e.g. a **joint**) selects it and fits
  the camera onto it so its axes fill the frame; empty = fit the WHOLE scene.
  Always ISOMETRIC; a **joint** target hides every other joint for the capture
  (restored right after) and waits one second so the render is up to date. Axis
  colors depend on the framing — see the sensors topic.
""",
    'best_practices': """\
## Best practices

- Work through the **General algorithm** (source of truth) as a TODO list,
  marking items done one by one; include the per-child **mirroring check**.
- Order: robot and links -> joints -> positioning -> sensors -> controllers ->
  collisions/materials.
- Resolve exact names with `list_scene_objects()` / `get_objects_info()` and
  call `set_placement_vision_mode()` before positioning.
- Position with `set_placement_between()`; verify each step with
  `get_snapshot()`; fix orientation with `rotate_object()`.
- Never guess the inner link name: read `Reference.prefix` from
  `get_objects_info([link])` and append the subelement (links from the same
  source body share one inner link name, e.g. all four wheels → `wheel001`).
- Error reporting: tools never block on a GUI dialog. A failure raises and the
  error text (with the workbench reason) is returned — read it and fix the call,
  do not retry blindly. A successful tool may return a `messages` field with
  non-fatal warnings; inspect it too.
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
    'controllers',
    'sensors',
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

    ``topic`` selects a section: ``'all'`` (default, the full workflow) or one
    of ``'general_algorithm'`` (source of truth), ``'create_robot'``,
    ``'positioning'``, ``'joints'``, ``'collisions'``, ``'materials'``,
    ``'controllers'``, ``'sensors'``, ``'inspection'``, ``'best_practices'``.

    Pure information: it does not modify the document.
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
    # ('create_document', create_document),
    ('create_robot', create_robot),
    ('create_link', create_link),
    ('create_links_filled', create_links_filled),
    ('create_joint', create_joint),
    ('create_joints_filled', create_joints_filled),
    ('create_collisions', create_collisions),
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
    ('list_controller_types', list_controller_types),
    ('create_controller', create_controller),
    ('get_robot_controllers', get_robot_controllers),
    ('list_controller_parameters', list_controller_parameters),
    ('set_controller_parameter', set_controller_parameter),
    ('set_controller_parameters', set_controller_parameters),
    ('list_sensor_types', list_sensor_types),
    ('create_sensor', create_sensor),
    ('set_placement_between_sensor_link_and_parent', set_placement_between_sensor_link_and_parent),
    ('list_scene_objects', list_scene_objects),
    ('get_objects_info', get_objects_info),
    ('check_overlap', check_overlap),
    ('get_snapshot', get_snapshot),
    ('instructions_to_work_with_tools', instructions_to_work_with_tools),
]


def get_tools() -> list[tuple[str, Any]]:
    """Return the list of (name, callable) tool pairs.

    Every tool is wrapped so that an expected failure (``RuntimeError`` /
    ``ValueError``: wrong object type, invalid reference, missing joint, ...)
    reaches the agent as the MCP SDK's ``ToolError``. The SDK then logs it at
    INFO without a traceback and forwards the message to the model, instead of
    reporting a crash ("Error executing tool ...") with a full ERROR traceback
    in the FreeCAD console.
    """
    return [(name, as_anticipated_error(fn)) for name, fn in TOOLS]
