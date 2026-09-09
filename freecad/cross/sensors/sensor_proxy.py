"""Proxy for Cross::Sensor FreeCAD objects

A sensors are representation of Gazebo sensors https://gazebosim.org/docs/latest/sensors/ gotten from handly maked sdf files (resources/sensors)
with added meta data (type, descriptions of fields, etc) from sdformat package (modules/sdformat) schema files via sdf_tree().
"""

from __future__ import annotations

from typing import ForwardRef, List, Optional, Union, cast
from typing import Iterable
from copy import deepcopy
from pathlib import Path
import os
import yaml
import xml.etree.ElementTree as ET
import re
import xmltodict

import FreeCAD as fc

from PySide.QtWidgets import QMenu  # FreeCAD's PySide

from ..freecad_utils import ProxyBase
from ..freecad_utils import add_property
from ..freecad_utils import error
from ..freecad_utils import get_valid_property_name
from ..freecad_utils import warn
from ..wb_utils import ICON_PATH
from ..wb_utils import SENSORS_DATA_PATH
from ..wb_utils import SDFORMAT_PATH
from ..wb_utils import MODULES_PATH
from ..wb_utils import is_joint
from ..wb_utils import is_link
from ..wb_utils import is_sensor
from ..wb_utils import return_true
from ..wb_utils import ros_name
from ..wb_utils import get_valid_urdf_name
from ..utils import deepmerge, replace_substring_in_keys
from .. import wb_constants
from ..exceptions import CallRecursion
from ..sdf.sdf_parser.sdf_schema_parser import sdf_schema_parser
from ..sdf.sdf_parser.sdf_tree import sdf_tree

# Stubs and type hints.
from .sensor import Sensor as CrossSensor  # A Cross::Sensor, i.e. a DocumentObject with Proxy "Sensor". # noqa: E501
DO = fc.DocumentObject
DOList = List[DO]
VPDO = ForwardRef('FreeCADGui.ViewProviderDocumentObject')  # Don't want to import FreeCADGui here. # noqa: E501
AppLink = DO  # TypeId == 'App::Link'.
check_functions_required = [is_joint, is_link, return_true, is_sensor] # dont remove used by check_functions()

class SensorProxy(ProxyBase):
    """The proxy for Sensor objects."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Cross::Sensor'

    def __init__(self, obj: CrossSensor):
        super().__init__(
            'sensor', [
            '_Type',
            ],
        )

        if obj.Proxy is not self:
            obj.Proxy = self
        self.sensor = obj

        self._init_properties(obj)


    def _init_properties(self, obj: CrossSensor):
        add_property(
            obj, 'App::PropertyString', '_Type', 'Internal',
            'The type',
        )
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        add_property(
            obj, 'App::PropertyPlacement', 'Placement', 'Internal',
            'Placement of the sensor in the robot frame '
            '(updated from the parent link/joint)',
            fc.Placement(),
        )
        obj.setPropertyStatus('Placement', ['ReadOnly'])


    def execute(self, obj: CrossSensor) -> None:
        pass


    def onChanged(self, obj: CrossSensor, prop: str) -> None:
        pass


    def onDocumentRestored(self, obj):
        """Restore attributes because __init__ is not called on restore."""
        self.__init__(obj)


    def dumps(self):
        return None


    def loads(self, state) -> None:
        pass


def get_sensor_placement(sensor: CrossSensor) -> fc.Placement:
    """Return the placement of a sensor in the global frame.

    The sensor is attached to a parent link or joint (kept in its Group),
    so the placement is taken from that parent element.
    Fallbacks: the robot placement, then the identity.

    Implementation note: a module-level function is used (instead of a
    proxy method) because the proxy can be restored without its
    attributes (e.g. on document restore) while `sensor` is always
    available from the view provider.
    """
    for parent in getattr(sensor, 'InList', []):
        if is_link(parent) or is_joint(parent):
            if hasattr(parent, 'getGlobalPlacement'):
                return parent.getGlobalPlacement()
            return parent.Placement
        if is_robot(parent) and hasattr(parent, 'Placement'):
            return parent.Placement
    return fc.Placement()


class _ViewProviderSensor(ProxyBase):
    """A view provider for the Sensor container object """

    # Last parts (suffixes) of camera sensor parameter full names that
    # change the shape of the field-of-view visualization.
    _frustum_param_suffixes = [
        'horizontal_fov',
        'width',
        'height',
        'near',
        'far',
    ]

    # Last parts (suffixes) of lidar sensor parameter full names that
    # change the shape of the field-of-view visualization. The suffixes are
    # used only to trigger a redraw on property changes; the actual values
    # are read with the full path (`_get_sensor_param_path`) because e.g.
    # `min_angle`/`max_angle` exist both in `scan/horizontal` and
    # `scan/vertical`.
    _lidar_param_suffixes = [
        'min_angle',
        'max_angle',
        'min',
        'max',
    ]

    def __init__(self, vobj: VPDO):
        super().__init__(
            'view_object',
            [
                'Visibility',
            ],
        )
        vobj.Proxy = self

    def getIcon(self):
        # Implementation note: "return 'sensor.svg'" works only after
        # workbench activation in GUI.
        return str(ICON_PATH / 'sensor.svg')

    def attach(self, vobj: VPDO):
        self.view_object = vobj
        self.sensor = vobj.Object
        # Add the display properties here only: doing it from within
        # `onChanged`/`visibilityChanged` (nested property modifications
        # during a property notification) breaks the visibility handling
        # in FreeCAD (e.g. the eye icon in the tree does not update).
        self._init_display_properties(vobj)
        self._init_display_mode(vobj)
        self._redraw_frustum()

    def _init_display_properties(self, vobj: VPDO) -> None:
        """Add the display options of the field-of-view visualization.

        Called from `attach` and lazily from `onChanged` because `onChanged`
        can be called before `attach` (e.g. on document restore of sensors
        created before these properties were introduced).

        The frustum visibility follows the standard `Visibility` property,
        so it can be toggled with the Space key on a selected sensor.
        """
        if not hasattr(vobj, 'FrustumColor'):
            vobj.addProperty(
                'App::PropertyColor', 'FrustumColor', 'Display Options',
                'Color of the field-of-view pyramid of a camera sensor',
            )
            vobj.FrustumColor = (0.0, 1.0, 0.0)  # Green.
        if not hasattr(vobj, 'FrustumTransparency'):
            vobj.addProperty(
                'App::PropertyIntegerConstraint', 'FrustumTransparency',
                'Display Options',
                'Transparency (%) of the field-of-view pyramid of a camera sensor',
            )
            # Setter with (value, min, max, step) sets the constraints.
            vobj.FrustumTransparency = (80, 0, 100, 1)

    def _init_display_mode(self, vobj: VPDO) -> bool:
        """Create the frustum display-mode node and register it.

        Return True when the frustum node exists and is ready to be drawn
        into.

        The node is registered with `vobj.addDisplayMode()` so that the
        standard `Visibility` property is applied by FreeCAD to this node
        through the display-mode switch (the same mechanism as used by
        `_ViewProviderPlanningScene`). This is what makes the Space key hide
        and show the frustum in both directions.
        """
        frustum = getattr(self, '_frustum', None)
        if (frustum is not None
                and getattr(self, '_frustum_view_object', None) is vobj):
            return True
        if (vobj is None) or not hasattr(vobj, 'addDisplayMode'):
            return False
        from pivy import coin

        try:
            frustum = coin.SoSeparator()
            frustum.setName('Frustum')
            vobj.addDisplayMode(frustum, 'Frustum')
        except Exception:
            # `addDisplayMode` is only possible once the view provider is
            # attached; do not draw (and do not fall back to `RootNode`,
            # which would break the visibility toggle) and wait for
            # `attach()` to run.
            self._frustum = None
            return False
        self._frustum = frustum
        self._frustum_view_object = vobj
        return True

    def getDisplayModes(self, vobj: VPDO) -> list[str]:
        """Return the available display modes."""
        return ['Frustum']

    def getDefaultDisplayMode(self) -> str:
        """Return the name of the default display mode."""
        return 'Frustum'

    def setDisplayMode(self, mode: str) -> str:
        """Accept the display mode requested by FreeCAD."""
        return mode

    def _get_view_object(self):
        """Return the view object or None.

        Implementation note: `getattr(self, 'view_object', None)` is used
        because the hooks can be called before `attach` has stored the
        view object.
        """
        return getattr(self, 'view_object', None)

    def _get_sensor(self):
        """Return the sensor DocumentObject or None."""
        view_object = self._get_view_object()
        if view_object is None:
            return None
        return getattr(view_object, 'Object', None)

    def _get_sensor_type(self) -> str:
        """Return the sensor type (SDF sensor@type), e.g. `camera`."""
        return str(getattr(self._get_sensor(), 'attr_type', '') or '')

    def _is_camera_sensor(self) -> bool:
        """Return True if the sensor is a camera (lidar and others excluded)."""
        return 'camera' in self._get_sensor_type()

    def _is_lidar_sensor(self) -> bool:
        """Return True if the sensor is a lidar (e.g. `gpu_lidar`)."""
        return 'lidar' in self._get_sensor_type()

    def _get_sensor_param(self, suffix: str):
        """Return a sensor parameter value by the last part of its full name.

        Full parameter names look like `camera___clip___far` (name parts are
        joined with the full-name glue), so `suffix` is the last part,
        e.g. `far` for `camera___clip___far`.

        Implementation note: use `_get_sensor_param_path()` to read a
        parameter whose last part is ambiguous (e.g. `min_angle` exists both
        in `scan/horizontal` and in `scan/vertical` of a lidar).
        """
        glue = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE
        sensor = self._get_sensor()
        full_names = getattr(sensor, 'sensor_parameters_fullnames_list', [])
        for full_name in full_names:
            if full_name.split(glue)[-1] == suffix:
                return getattr(sensor, full_name, None)
        return None

    def _get_sensor_param_path(self, parts: list[str]):
        """Return a sensor parameter value by its full path.

        Full parameter names look like `camera___clip___far` (name parts are
        joined with the full-name glue), so `parts` is the list of the full
        path parts, e.g. `['lidar', 'scan', 'horizontal', 'min_angle']` for
        `lidar___scan___horizontal___min_angle`.

        The explicit path is required when the last part alone is ambiguous
        (a lidar has `min_angle`/`max_angle` both in `scan/horizontal` and
        in `scan/vertical`).
        """
        glue = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE
        sensor = self._get_sensor()
        full_name = glue.join(parts)
        if full_name in getattr(sensor, 'sensor_parameters_fullnames_list', []):
            return getattr(sensor, full_name, None)
        return None

    def _redraw_frustum(self) -> None:
        """Draw the field-of-view visualization of the sensor.

        The shape depends on the sensor type:
        - a camera gets a frustum (a truncated pyramid) directed along the
          positive X axis of the parent link/joint (the direction the camera
          looks at), following Gazebo's camera convention: X points forward,
          Y points left (image width) and Z points up (image height). Its
          near face is at the `clip.near` distance and its far face is at
          the `clip.far` distance;
        - a lidar gets the volume between the `range.min` and `range.max`
          shells inside the horizontal/vertical scan angle ranges, in red
          with 80% transparency (see `_draw_lidar_fov`).
        """
        import math

        from pivy import coin

        from ..coin_utils import transform_from_placement

        view_object = self._get_view_object()
        obj = self._get_sensor()
        if view_object is None or obj is None:
            return
        if not self._init_display_mode(view_object):
            # The display-mode node is not available yet (e.g. the hooks
            # were called before `attach()`); do not draw into `RootNode`
            # because that would not be hidden by FreeCAD on the standard
            # visibility toggle.
            return

        # Draw the frustum into the registered display-mode node: FreeCAD
        # toggles that node through the display-mode switch on the standard
        # visibility change, so the Space key hides/shows the frustum in both
        # directions without depending on this redraw (same pattern as
        # `_ViewProviderPlanningScene`).
        frustum = self._frustum
        frustum.removeAllChildren()

        if not getattr(view_object, 'Visibility', True):
            # FreeCAD toggles the display-mode switch on the standard
            # visibility change, but keep the python-side redraw symmetric
            # for the cases where FreeCAD only changes the `Visibility`
            # property and expects the proxy to react.
            return
        if not self._is_camera_sensor():
            if self._is_lidar_sensor():
                # A lidar FOV is an angular sector between the range min
                # and range max shells (see `_draw_lidar_fov`); unlike a
                # camera frustum it is not defined by clip near/far planes.
                self._draw_lidar_fov(frustum, obj)
            # Sensors without a field of view have no visualization.
            return

        # Camera parameters from the sensor data.
        # `horizontal_fov` is in radians (as in SDF),
        # `near`/`far` are in meters, so convert them to FreeCAD units (mm).
        hfov = self._get_sensor_param('horizontal_fov')
        width = self._get_sensor_param('width')
        height = self._get_sensor_param('height')
        near = self._get_sensor_param('near')
        far = self._get_sensor_param('far')

        if hfov is None or far is None:
            return
        hfov = float(hfov)
        far_mm = float(far) * 1000.0
        near_mm = float(near) * 1000.0 if near is not None else 0.0
        # The near clip distance is the start of the visibility, it must be
        # less than the far one.
        near_mm = min(max(near_mm, 0.0), far_mm * 0.999)
        if hfov <= 0.0 or hfov >= math.pi or far_mm <= 0.0:
            return

        # Vertical FOV from the image aspect ratio (like Gazebo does),
        # fall back to the square image if the image size is unknown.
        if width and height:
            aspect_ratio = float(height) / float(width)
        else:
            aspect_ratio = 1.0
        vfov = 2.0 * math.atan(math.tan(hfov / 2.0) * aspect_ratio)

        # Vertices of the truncated pyramid (frustum) along +X:
        # the first 4 points are the near face (the start of the camera
        # visibility), the last 4 are the far face.
        # Gazebo's camera convention is respected relative to the parent
        # joint/link: X points forward, Y points left (image width) and
        # Z points up (image height).
        near_half_width = near_mm * math.tan(hfov / 2.0)
        near_half_height = near_mm * math.tan(vfov / 2.0)
        far_half_width = far_mm * math.tan(hfov / 2.0)
        far_half_height = far_mm * math.tan(vfov / 2.0)
        vertices = [
            (near_mm,  near_half_width,  near_half_height),
            (near_mm,  near_half_width, -near_half_height),
            (near_mm, -near_half_width, -near_half_height),
            (near_mm, -near_half_width,  near_half_height),
            (far_mm,  far_half_width,  far_half_height),  # Far face.
            (far_mm,  far_half_width, -far_half_height),
            (far_mm, -far_half_width, -far_half_height),
            (far_mm, -far_half_width,  far_half_height),
        ]

        indices = [
            [0, 1, 5, 4, -1],  # The 4 side faces.
            [1, 2, 6, 5, -1],
            [2, 3, 7, 6, -1],
            [3, 0, 4, 7, -1],
            [0, 1, 2, 3, -1],  # Near face (the start of the visibility).
            [4, 5, 6, 7, -1],  # Far face.
        ]

        sep = coin.SoSeparator()

        # Add a material node.
        # Implementation note: getattr with defaults is used because
        # `onChanged` can be called before the display properties are added.
        material = coin.SoMaterial()
        material.diffuseColor = getattr(
            self.view_object, 'FrustumColor', (0.0, 1.0, 0.0),
        )[:3]
        material.transparency = getattr(
            self.view_object, 'FrustumTransparency', 80,
        ) / 100.0
        sep.addChild(material)

        # Add a transform node.
        # The sensor itself has no meaningful own placement, so it is
        # computed from the parent link/joint (see get_sensor_placement).
        sep.addChild(transform_from_placement(get_sensor_placement(obj)))

        coord = coin.SoCoordinate3()
        coord.point.setValues(0, len(vertices), vertices)

        face_set = coin.SoIndexedFaceSet()
        face_set.coordIndex.setValues(
            0,
            sum(len(face) for face in indices),
            [i for face in indices for i in face],
        )

        sep.addChild(coord)
        sep.addChild(face_set)
        frustum.addChild(sep)

    def _draw_lidar_fov(self, frustum, obj) -> None:
        """Draw the field-of-view visualization of a lidar sensor.

        Unlike a camera, a lidar has no `clip` near/far planes: it measures
        points between a `range.min` and a `range.max` distance inside the
        angular sector defined by the `scan` parameters:
        - `scan/horizontal` (`min_angle`/`max_angle`) is the azimuth,
          measured around the vertical Z axis from the +X direction;
        - `scan/vertical` (`min_angle`/`max_angle`) is the elevation above
          (positive) or below (negative) the horizontal XY plane.

        The sector is drawn between the inner (range min) and the outer
        (range max) shells in the requested red color with 80% transparency
        and is placed in the sensor frame like the camera frustum (X
        forward). It is added to the same registered display-mode node so
        that the standard visibility toggle (Space key) applies to it too.
        """
        import math

        from pivy import coin

        from ..coin_utils import transform_from_placement

        # Lidar parameters are read by their full path (not by the last
        # part only): a lidar has `min_angle`/`max_angle` both in
        # `scan/horizontal` and in `scan/vertical`.
        h_min = self._get_sensor_param_path(
            ['lidar', 'scan', 'horizontal', 'min_angle'])
        h_max = self._get_sensor_param_path(
            ['lidar', 'scan', 'horizontal', 'max_angle'])
        v_min = self._get_sensor_param_path(
            ['lidar', 'scan', 'vertical', 'min_angle'])
        v_max = self._get_sensor_param_path(
            ['lidar', 'scan', 'vertical', 'max_angle'])
        range_min = self._get_sensor_param_path(['lidar', 'range', 'min'])
        range_max = self._get_sensor_param_path(['lidar', 'range', 'max'])
        if (h_min is None or h_max is None
                or v_min is None or v_max is None
                or range_max is None):
            return

        h_min = float(h_min)
        h_max = float(h_max)
        v_min = float(v_min)
        v_max = float(v_max)
        # Range min/max are distances in meters (as in SDF), so convert them
        # to FreeCAD units (mm).
        range_min_mm = (
            float(range_min) * 1000.0 if range_min is not None else 0.0)
        range_max_mm = float(range_max) * 1000.0
        if (h_max - h_min) <= 1e-9 or (v_max - v_min) <= 1e-9:
            return
        if range_max_mm <= 0.0:
            return
        range_min_mm = min(max(range_min_mm, 0.0), range_max_mm * 0.999)
        if range_min_mm <= 0.0:
            # Degenerate inner shell; keep a tiny radius so that the caps
            # of the sector stay closed.
            range_min_mm = range_max_mm * 1e-3

        # Sample the angular spans so that wide fields of view are drawn as
        # an accurate sector (the four corner rays alone would cut the
        # azimuth arcs of the lidar).
        h_count = max(
            2, int(math.ceil((h_max - h_min) / math.radians(10.0))) + 1)
        v_count = max(
            2, int(math.ceil((v_max - v_min) / math.radians(10.0))) + 1)
        radii = (range_min_mm, range_max_mm)

        def ray(r: float, h_angle: float, v_angle: float):
            # Lidar ray direction in the sensor frame: X forward, azimuth
            # around the Z axis (from +X toward +Y) and elevation above the
            # XY plane (toward +Z).
            h_cos = math.cos(h_angle)
            h_sin = math.sin(h_angle)
            v_cos = math.cos(v_angle)
            v_sin = math.sin(v_angle)
            return (r * h_cos * v_cos,
                    r * h_sin * v_cos,
                    r * v_sin)

        vertices = []
        point_index = {}

        def get_point(i: int, j: int, k: int) -> int:
            key = (i, j, k)
            index = point_index.get(key)
            if index is None:
                h_angle = h_min + (h_max - h_min) * i / (h_count - 1)
                v_angle = v_min + (v_max - v_min) * j / (v_count - 1)
                index = len(vertices)
                point_index[key] = index
                vertices.append(ray(radii[k], h_angle, v_angle))
            return index

        indices = []

        def add_quad(a: int, b: int, c: int, d: int) -> None:
            indices.extend((a, b, c, d, -1))

        # The inner and outer range shells of the sector.
        for k in (0, 1):
            for i in range(h_count - 1):
                for j in range(v_count - 1):
                    add_quad(
                        get_point(i, j, k),
                        get_point(i + 1, j, k),
                        get_point(i + 1, j + 1, k),
                        get_point(i, j + 1, k),
                    )
        # The top (max elevation) and bottom (min elevation) caps.
        for j in (v_count - 1, 0):
            for i in range(h_count - 1):
                add_quad(
                    get_point(i, j, 0),
                    get_point(i + 1, j, 0),
                    get_point(i + 1, j, 1),
                    get_point(i, j, 1),
                )
        # The side caps at the min and max azimuth.
        for i in (h_count - 1, 0):
            for j in range(v_count - 1):
                add_quad(
                    get_point(i, j, 0),
                    get_point(i, j + 1, 0),
                    get_point(i, j + 1, 1),
                    get_point(i, j, 1),
                )

        if not vertices or not indices:
            return

        sep = coin.SoSeparator()

        # Fixed red color with 80% transparency (the lidar FOV style).
        material = coin.SoMaterial()
        material.diffuseColor = (1.0, 0.0, 0.0)
        material.transparency = 0.8
        sep.addChild(material)

        # The sensor itself has no meaningful own placement, so it is
        # computed from the parent link/joint (see get_sensor_placement).
        sep.addChild(transform_from_placement(get_sensor_placement(obj)))

        coord = coin.SoCoordinate3()
        coord.point.setValues(0, len(vertices), vertices)

        face_set = coin.SoIndexedFaceSet()
        face_set.coordIndex.setValues(0, len(indices), indices)

        sep.addChild(coord)
        sep.addChild(face_set)
        frustum.addChild(sep)

    def updateData(self, obj: CrossSensor, prop: str):
        # Redraw when a parameter that defines the shape of the field-of-view
        # visualization (camera frustum or lidar sector) has changed or when
        # the sensor placement (parent link/joint pose) has been updated.
        prop_last_part = prop.split(
            wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE,
        )[-1]
        if (prop == 'Placement'
                or prop_last_part in self._frustum_param_suffixes
                or prop_last_part in self._lidar_param_suffixes):
            self._redraw_frustum()
        return

    def onChanged(self, vobj: VPDO, prop: str):
        # Redraw on display option and visibility changes. The frustum is
        # drawn into a display mode node whose visibility is managed by
        # FreeCAD itself, so the redraw only (re)creates the content;
        # `_redraw_frustum()` clears the node when the sensor is hidden.
        # Note: this hook must never modify properties of `vobj` (a nested
        # property modification during the notification breaks the
        # visibility handling in FreeCAD, e.g. the eye icon in the tree
        # does not update).
        if prop in ('Visibility', 'FrustumColor', 'FrustumTransparency'):
            try:
                self._redraw_frustum()
            except Exception:
                pass

    def visibilityChanged(self, vobj: VPDO, visible: bool) -> None:
        """Called by FreeCAD when the standard visibility changes.

        The frustum is drawn into a display mode node whose visibility is
        managed by FreeCAD itself (see `_init_display_mode`), so no action
        is needed here to hide or show it with the Space key. A redraw is
        only needed to (re)create the content after the object has been
        restored.
        """
        try:
            self._redraw_frustum()
        except Exception:
            pass

    def setupContextMenu(self, vobj: VPDO, menu: QMenu) -> None:
        return

    def doubleClicked(self, vobj: VPDO):
        gui_doc = vobj.Document
        if not gui_doc.getInEdit():
            gui_doc.setEdit(vobj.Object.Name)
        else:
            error('Task dialog already active')
        return True

    def setEdit(self, vobj: VPDO, mode):
        return False

    def unsetEdit(self, vobj: VPDO, mode):
        import FreeCADGui as fcgui
        fcgui.Control.closeDialog()

    def dumps(self):
        return None

    def loads(self, state) -> None:
        pass

    def on_context_menu(self, vobj: VPDO) -> None:
        pass


def add_sensor_properties_block(sensor: CrossSensor, sensor_data: dict) -> CrossSensor:

    sensor = add_sensor_properties(
        sensor,
        {sensor_data['name']: sensor_data['parameters']},
        sensor_data['name'],
    )

    adding_flatten_params = flatten_params(sensor_data['parameters'], flat_params = {})
    parameters_flatten = deepcopy(sensor_data['parameters_flatten'])
    sensor_data['parameters_flatten'] = {
        **parameters_flatten,
        **adding_flatten_params,
    }
    parameters_flatten_full_names = sensor_data['parameters_flatten'].keys()
    prop_name = 'sensor_parameters_fullnames_list'
    # add meta property
    # there are only list of full names of sensor parameters (gotten from sensor YAML config)
    if hasattr(sensor, prop_name):
        setattr(sensor, prop_name, parameters_flatten_full_names)
    else:
        sensor, used_property_name = add_property(
            sensor,
            'App::PropertyStringList',
            prop_name,
            'Internal',
            'List of full names of parameters',
            parameters_flatten_full_names,
        )
        sensor.setPropertyStatus(prop_name, ['Hidden', 'ReadOnly'])

    return sensor


def add_sensor_properties(
    sensor: CrossSensor,
    parameters: dict,
    parameter_name: str,
    parameter_full_name_glue: str = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE,
) -> CrossSensor:
    """Adding properties to sensor."""

    for param_name, param in parameters[parameter_name].items():

        try:
            # type param present only in leaf element
            # and exception used for recursion call.
            # If throw exception then go recursion
            prop_type = param['type_fc']

            default_value = None
            if 'default_value' in param:
                default_value = param['default_value']

            try:
                var_name = param['full_name']
            except KeyError:
                # avoiding except of outer try block by using other type error
                raise RuntimeError('param full_name - KeyError')

            # for recursive props make category for grouping them from names of each recursion dive
            # example linear__x__has_velosity_limits
            full_name_splited = param['full_name'].split(parameter_full_name_glue)
            if len(full_name_splited) > 1:
                category = parameter_full_name_glue.join(full_name_splited[:-1])
            else:
                # root categories
                category = 'Mandatory Root'
                if default_value is None \
                or (not default_value and default_value is not False):

                    if 'description' in param:
                        if '(Optional)' in param['description'] \
                        or '(optional)' in param['description']:
                            category = 'Root'
                else:
                    category = 'Root'

            # make description
            help_txt = ''
            if 'description' in param:
                help_txt = param['description']

            if prop_type in wb_constants.TYPE_CONVERT_FUNCTIONS:
                default_value = wb_constants.TYPE_CONVERT_FUNCTIONS[prop_type](default_value)

            # add property
            sensor, used_property_name = add_property(
                sensor,
                prop_type,
                var_name,
                category,
                help_txt,
                default_value,
            )

        except KeyError:
            # the type is not found at this level and should dive deeper
            sensor = add_sensor_properties(
                sensor,
                parameters[parameter_name],
                param_name,
            )

    return sensor


def get_sensors_data(SENSORS_PATH: Path = SENSORS_DATA_PATH) -> dict :
    ''' Get sensors data. '''

    def collect_sensors_parameters(sensors_dirs: dict) -> dict :
        ''' Adding to sensors their collected parameters. '''

        for sensor_attached_to in sensors_dirs:
            sensors_dir_data = sensors_dirs[sensor_attached_to]
            for sensor in list(sensors_dir_data['sensors'].values()):
                with open(sensor['path']) as stream:
                    try:
                        data_dict = xmltodict.parse(stream.read())
                    except:
                        pass

                    if 'file_data' in sensor:
                        sensor['file_data'].update(data_dict)
                    else:
                        sensor['file_data'] = data_dict

                    sensor_data = data_dict['sdf']['world'][sensor_attached_to]['sensor']
                    # remove @name because we will use sensor name from FC instead of sensor name from sdf
                    sensor_parameters = {key: value for key, value in sensor_data.items() if not key.startswith('@name')}

                    # replace attr prefix (@) with 'attr_' because @ cant be save in prop name
                    sensor_parameters = replace_substring_in_keys(
                        sensor_parameters,
                        wb_constants.XMLTODICT_ATTR_PREFIX_ORIGIN,
                        wb_constants.XMLTODICT_ATTR_PREFIX_FIXED_FOR_PROP_NAME,
                    )

                    if 'parameters' in sensor:
                        sensor['parameters'].update(sensor_parameters)
                    else:
                        sensor['parameters'] = sensor_parameters

        return sensors_dirs


    def add_schema_data(sensors, sensor_schema_as_dict) -> dict :

        def add_param_data(parameter: dict, parameter_name: str, sensor_schema_as_dict: dict) -> dict:
            """Add data (like data, description) to paramater from sdf schema"""
            for key in list(parameter):
                elem = parameter[key]
                try:
                    if isinstance(elem, dict) and '#text' not in elem:
                        raise CallRecursion('go recursion to leaf element of dict')

                    if not isinstance(elem, dict) and not isinstance(elem, list) and '#text' not in key:
                        parameter[key] = {'#text': elem}

                    index_type = sdf_schema_parser.get_technical_attr_prefix_with_attr_symbol() + 'type'
                    index_description = sdf_schema_parser.get_technical_attr_prefix_with_attr_symbol() + 'description'
                    if '#text' != key:
                        try:
                            parameter[key][index_type] = sensor_schema_as_dict[parameter_name][key][index_type]
                            parameter[key][index_description] = sensor_schema_as_dict[parameter_name][key][index_description]
                        except (KeyError, TypeError):
                            pass
                    else:
                        try:
                            parameter[index_type] = sensor_schema_as_dict[parameter_name][index_type]
                            parameter[index_description] = sensor_schema_as_dict[parameter_name][index_description]
                        except (KeyError, TypeError):
                            pass

                except CallRecursion:
                    parameter[key] = add_param_data(elem, key, sensor_schema_as_dict[parameter_name])

            return parameter


        for attached_group in sensors['sensors_dirs']:
            sensors_of_attached_group = sensors['sensors_dirs'][attached_group]['sensors']

            for sensor_name in sensors_of_attached_group:

                for parameter_name, parameter in sensors_of_attached_group[sensor_name]['parameters'].items():
                    if not isinstance(parameter, dict):
                        parameter_value = parameter
                        parameter = {'#text': parameter_value}
                    sensors_of_attached_group[sensor_name]['parameters'][parameter_name] = add_param_data(parameter, parameter_name, sensor_schema_as_dict)

            sensors['sensors_dirs'][attached_group]['sensors'] = sensors_of_attached_group

        return sensors


    tree=sdf_tree("sensor.sdf")
    sensor_schema_as_dict=tree.get_element_as_dict['sensor']

    sensors = get_sensors_root_dirs(SENSORS_PATH)
    sensors['sensors_dirs'] = collect_sensors_files_grouped_by_dirs(sensors['sensors_dirs'])
    sensors['sensors_dirs'] = collect_sensors_parameters(sensors['sensors_dirs'])
    sensors = add_schema_data(sensors, sensor_schema_as_dict)
    sensors = separate_sensors_from_dirs(sensors['sensors_dirs'])

    return sensors


def add_full_name_to_params(
    params: dict,
    param_name_prefix: list = [],
    parameter_full_name_glue: str = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE,
) -> dict:
    ''' Add full name with parent prefixes to every param.

    Params can be at various levels of nested deep.
    full_param_name means all parents prefixes + param_name joined with parameter_full_name_glue
    '''

    for param_name, param in params.items():
        try:
            if isinstance(param, dict) and '#text' not in param:
                raise CallRecursion('go recursion to leaf element of dict')

            full_param_name = parameter_full_name_glue.join(param_name_prefix + [param_name])

            if not isinstance(param, dict):
                params[param_name] = {}
                params[param_name]['#text'] = param

            params[param_name]['full_name'] = full_param_name
            params[param_name]['default_value'] = params[param_name].get('#text', '')
        except CallRecursion:
            param_name_prefix.append(param_name)
            params[param_name] = add_full_name_to_params(param, param_name_prefix)
            param_name_prefix.pop()

    return params


def flatten_params(params: dict, flat_params: dict) -> dict:
    ''' flattens parameters dict.
    Set full (with parent prefixes) param name as root dict attribute and this way make 1 level parameters
    '''

    for param_name, param in params.items():
        try:
            if isinstance(param, dict) and '#text' not in param:
                raise CallRecursion('go recursion to leaf element of dict')

            flat_params[param['full_name']] = param
        except CallRecursion:
            flat_params = flatten_params(param, flat_params)

    return flat_params


def unflatten_params(
    flatten_params: dict,
    param_to_replace: str | None = None,
    replace: str | None = None,
    parameter_full_name_glue: str = wb_constants.ROS2_CONTROLLERS_PARAM_FULL_NAME_GLUE,
) -> dict:
    ''' Unflattens parameters dict.

    Split flatten params name by parameter_full_name_glue and make nested structure with list
    '''

    def unflatten_params_recursion(params, param_name: str, leaf_param_data: dict = {}, params_nest_level_name: str = ''):
        params_nest = param_name.split(parameter_full_name_glue)
        params_nest_level_name = params_nest.pop(0)

        if param_to_replace is not None and replace is not None:
            if params_nest_level_name == param_to_replace:
                params_nest_level_name = replace
                leaf_param_data['full_name'] = leaf_param_data['full_name'].replace(param_to_replace, replace)

        result_params = {}

        if len(params_nest) > 0:
            params = unflatten_params_recursion(params, parameter_full_name_glue.join(params_nest), leaf_param_data, params_nest_level_name)
            result_params = {params_nest_level_name: params}
        else:
            result_params = {params_nest_level_name: leaf_param_data}

        return result_params


    params = {}
    for param_name, leaf_param_data in flatten_params.items():
        params = deepmerge(unflatten_params_recursion(params, param_name, leaf_param_data), params)

    return params


def separate_sensors_from_dirs(sensors_dirs: dict) -> dict :
    ''' Separate sensors from sensors directories dictionaries.

    Directory can contains several sensors.
    This make sensors as first level properties of dict.
    '''

    def add_fc_types_based_on_params_types(params: dict, param_name_prefix: list = []) -> dict:
        ''' Return params with FreeCAD types in addition to ros2_sensors parameters types
        '''

        # some types required to be replaced to more suited because can be more convenient to use
        replacement = wb_constants.ROS2_CONTROLLERS_PARAMS_TYPES_REPLACEMENTS
        for param_name, param in params.items():
            try:
                if isinstance(param, dict) and '#text' not in param:
                    raise CallRecursion('go recursion to leaf element of dict')

                try:
                    type = param[sdf_schema_parser.get_technical_attr_prefix_with_attr_symbol() + 'type']

                    try:
                        type_value = type['#text']
                    except (KeyError, TypeError):
                        type_value = type

                    type_fc = wb_constants.ROS2_CONTROLLERS_PARAMS_TO_FRECAD_PROP_MAP[type_value]
                except KeyError:
                    type = 'string'
                    type_fc = wb_constants.ROS2_CONTROLLERS_PARAMS_TO_FRECAD_PROP_MAP[type]

                params[param_name]['type_fc'] = type_fc
                params[param_name]['type_fc_origin'] = param['type_fc']

            except CallRecursion:
                param_name_prefix.append(param_name)
                params[param_name] = add_fc_types_based_on_params_types(param, param_name_prefix)
                param_name_prefix.pop()

        return params


    sensors = {}
    for sensor_dir_name, sensor_dir in sensors_dirs.items():
        for sensor_name, sensor in sensor_dir['sensors'].items():

            # prepare sensor parameters
            parameters = sensor['parameters']

            parameters = add_full_name_to_params(parameters)
            parameters = add_fc_types_based_on_params_types(parameters)
            parameters_flatten = flatten_params(parameters, flat_params = {})


            if sensor_dir_name not in sensors:
                sensors[sensor_dir_name] = {}

            # assembly sensor dict
            sensors[sensor_dir_name][sensor_name] = {
                'name': sensor_name,
                'type': sensor.get('type', ''),
                'description': sensor.get('description', ''),
                'sensor_path': sensor.get('path', ''),
                'parameters': parameters,
                'parameters_flatten': parameters_flatten,
                'sensor_dir_name': sensor_dir_name,
            }

    return sensors


def get_sensors_root_dirs(SENSORS_PATH: Path = SENSORS_DATA_PATH) -> dict :
    ''' Get sensors root dirs. '''
    sensors = {}

    sensor_key_words = ['joint', 'link'] # , 'model'
    sensor_key_words_blacklist = []
    sensors_dirs = get_files_or_dirs_by_filter(
        SENSORS_PATH,
        files_or_dirs = 'dirs',
        key_words = sensor_key_words,
        key_words_blacklist = sensor_key_words_blacklist,
        attr_name_for_found_result = 'dir',
    )

    sensors['sensors_dirs'] = sensors_dirs

    return sensors


def collect_sensors_files_grouped_by_dirs(sensors_dirs: dict) -> dict :
    ''' Get sensors files. '''

    for sensor_folder_name, sensor_value in sensors_dirs.items():

        sensors_files = get_files_or_dirs_by_filter(
            sensor_value['dir'],
            key_words = ['.sdf'],
            key_words_blacklist = [],
            attr_name_for_found_result = 'path',
        )
        sensors_dirs[sensor_folder_name].update({'sensors': sensors_files})

    return sensors_dirs


def get_files_or_dirs_by_filter(
    dir: Path,
    files_or_dirs:str = 'files',
    key_words:list = [],
    key_words_blacklist:list = [],
    attr_name_for_found_result:str | None = None,
) -> dict :
    ''' Get files or dirs in path by included keywords in file name. '''

    files_or_dirs_result = {}
    for root, dirs, files in os.walk(dir):
        for name in eval(files_or_dirs):
            # check key words
            for key_word in key_words:
                if key_word in name:

                    # check blacklist
                    name_in_blacklist = False
                    for key_word_bl in key_words_blacklist:
                        if key_word_bl in name:
                            name_in_blacklist = True
                            break

                    # cut extention
                    name_without_file_extention = name.split('.')[0]

                    # forming result
                    if not name_in_blacklist:
                        if name not in files_or_dirs_result:
                            if not attr_name_for_found_result:
                                files_or_dirs_result[name_without_file_extention] = {Path(root) / name}
                            else:
                                files_or_dirs_result[name_without_file_extention] = {attr_name_for_found_result: Path(root) / name}

                        if not attr_name_for_found_result:
                            files_or_dirs_result.update({name_without_file_extention: Path(root) / name})
                        else:
                            files_or_dirs_result.update({name_without_file_extention: {attr_name_for_found_result: Path(root) / name}})

        break

    return files_or_dirs_result
