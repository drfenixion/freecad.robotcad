from __future__ import annotations

from typing import NewType
from typing import Optional
from typing import TYPE_CHECKING

import FreeCAD as fc

from pivy import coin
import numpy as np

from .coin_utils import transform_from_placement
from .freecad_utils import warn
from .wb_utils import ICON_PATH
from .wb_utils import is_link
from freecad.cross.vendor.fcapi import fpo  # Cf. https://github.com/mnesarco/fcapi

# Typing hints.
from .lidar2d import Lidar2d as CrossLidar2d  # A Cross::Lidar2d, i.e. a DocumentObject with Proxy "Lidar2dProxy". # noqa: E501

if TYPE_CHECKING and hasattr(fc, 'GuiUp') and fc.GuiUp:
    from FreeCADGui import ViewProviderDocumentObject as VPDO
else:
    VPDO = NewType('VPDO', object)


@fpo.view_proxy(
        icon=str(ICON_PATH / 'lidar2d.svg'),
)
class Lidar2dViewProxy:
    rays_display_mode = fpo.DisplayMode(name='Rays', is_default=True)

    color = fpo.PropertyColor(
            name='Color',
            section='Display Options',
            default=(1.0, 0.0, 0.0),
            description=(
                'Color of the representation in the 3D view'
            ),
    )

    transparency = fpo.PropertyIntegerConstraint(
            name='Transparency',
            section='Display Options',
            default=0,
            description=(
                'Transparency of the representation in the 3D view'
            ),
    )

    def on_start(self) -> None:
        # Set the transparency to a range of 0-100 with a step of 1.
        # Implementation note: getter is an int, setter can be (val, min, max, step).
        self.transparency = (self.transparency, 0, 100, 1)
        self._redraw()

    def on_change(self) -> None:
        self._redraw()

    def on_object_change(self) -> None:
        self._redraw()

    def _redraw(self) -> None:
        """Draw the rays."""

        obj = self.Object
        view = self.ViewObject
        if not hasattr(view, 'RootNode'):
            return
        root = view.RootNode
        root.removeAllChildren()

        if not view.Visibility:
            return

        if ((obj.RangeMax < obj.RangeMin)
                or (obj.RangeMax < 0)
                or (obj.AngleMax < obj.AngleMin)
           ):
            return

        sep = coin.SoSeparator()

        # Add a material node.
        material = coin.SoMaterial()
        material.diffuseColor = self.color[:3]
        if self.transparency is not None:
            # Implementation note: self.on_change is called before
            # self.on_start.
            # TODO: Fix this in fcapi.
            material.transparency = self.transparency / 100.0
        sep.addChild(material)

        # Add a transform node.
        transform = transform_from_placement(self.Object.Placement)
        sep.addChild(transform)

        # Calculate the vertices of the lines.
        angle_min = float(obj.AngleMin.getValueAs('rad'))
        angle_max = float(obj.AngleMax.getValueAs('rad'))
        angle_increment = float(obj.AngleIncrement.getValueAs('rad'))
        angles = np.linspace(angle_min, angle_max, int(round((angle_max - angle_min) / angle_increment)) + 1, endpoint=True)
        n = len(angles)
        ca = np.cos(angles)
        sa = np.sin(angles)
        start_points = np.zeros((n, 3), dtype=float)
        # Implementation note: `.Value` is in millimeters.
        start_points[:, 0] = obj.RangeMin.Value * ca
        start_points[:, 1] = obj.RangeMin.Value * sa
        end_points = np.zeros((n, 3), dtype=float)
        end_points[:, 0] = obj.RangeMax.Value * ca
        end_points[:, 1] = obj.RangeMax.Value * sa

        vertex_property = coin.SoVertexProperty()
        vertex_property.vertex.setValues(
                0,
                3 * n,
                np.column_stack((start_points, end_points)).flatten().reshape(-1, 3),
        )
        line_set = coin.SoLineSet()
        sep.addChild(line_set)
        line_set.numVertices.setValues(0, n, [2] * n)
        line_set.vertexProperty = vertex_property

        root.addChild(sep)


@fpo.proxy(
    object_type='App::FeaturePython',
    subtype='Cross::Lidar2d',
    view_proxy=Lidar2dViewProxy,
)
class Lidar2dProxy:

    angle_increment = fpo.PropertyAngle(
            name='AngleIncrement',
            section='Sensor Options',
            description='Angle between two rays',
            default=1.0,
    )

    angle_min = fpo.PropertyAngle(
            name='AngleMin',
            section='Sensor Options',
            description='Angle of the first ray',
            default=-180.0,
    )

    angle_max = fpo.PropertyAngle(
            name='AngleMax',
            section='Sensor Options',
            description='Angle of the last ray',
            default=180.0,
    )

    link = fpo.PropertyLink(
            name='Link',
            section='Elements',
            description='Cross::Link to attach to',
    )

    placement = fpo.PropertyPlacement(
            name='Placement',
            section='Internal',
            mode=fpo.PropertyMode.ReadOnly,
            description='Placement of the sensor in the robot frame',
    )

    range_min = fpo.PropertyLength(
            name='RangeMin',
            section='Sensor Options',
            description='Minimum range value',
            default=100,
    )

    range_max = fpo.PropertyLength(
            name='RangeMax',
            section='Sensor Options',
            description='Maximum range value',
            default=10_000,
    )


def make_lidar2d(
        name: str,
        doc: Optional[fc.Document] = None,
) -> CrossLidar2d:
    """Add a CROSS::Lidar2d to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj: CrossLidar2d = Lidar2dProxy.create(name=name, doc=doc)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        import FreeCADGui as fcgui

        # Make `obj` part of the selected `Cross::Robot`.
        sel = fcgui.Selection.getSelection()
        if sel:
            candidate = sel[0]
            if is_link(candidate):
                obj.Link = candidate
                try:
                    obj.Link.Proxy.get_robot().addObject(obj)
                except AttributeError:
                    pass

    return obj
