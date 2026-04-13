from __future__ import annotations

from math import sin
from typing import NewType
from typing import Optional
from typing import TYPE_CHECKING

import FreeCAD as fc

from pivy import coin

from .coin_utils import transform_from_placement
from .coin_utils import cone_between_points
from .freecad_utils import warn
from .wb_utils import ICON_PATH
from .wb_utils import is_link
from freecad.cross.vendor.fcapi import fpo  # Cf. https://github.com/mnesarco/fcapi

# Typing hints.
from .ultrasound import Ultrasound as CrossUltrasound  # A Cross::Ultrasound, i.e. a DocumentObject with Proxy "UltrasoundProxy". # noqa: E501

if TYPE_CHECKING and hasattr(fc, 'GuiUp') and fc.GuiUp:
    from FreeCADGui import ViewProviderDocumentObject as VPDO
else:
    VPDO = NewType('VPDO', object)


@fpo.view_proxy(
        icon=str(ICON_PATH / 'ultrasound.svg'),
)
class UltrasoundViewProxy:
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

        if (
                (obj.RangeMax < obj.RangeMin)
                or (obj.RangeMax <= 0)
                or (obj.DetectionAngle < 0.0)
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

        # Represent the envelope with a truncated cone.
        # TODO: truncate.
        # Implementation float(obj.Range) is in millimeters.
        cone = cone_between_points(
            [float(obj.RangeMax), 0.0, 0.0],
            [0.0, 0.0, 0.0],
            sin(obj.DetectionAngle.getValueAs('rad') / 2.0) * float(obj.RangeMax),
            color=self.color[:3],
        )
        sep.addChild(cone)

        root.addChild(sep)


@fpo.proxy(
    object_type='App::FeaturePython',
    subtype='Cross::Ultrasound',
    view_proxy=UltrasoundViewProxy,
)
class UltrasoundProxy:

    detection_angle = fpo.PropertyAngle(
            name='DetectionAngle',
            section='Sensor Options',
            description='Aperture angle of the sensor',
            default=10.0,
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
            default=0.0,
    )

    range_max = fpo.PropertyLength(
            name='RangeMax',
            section='Sensor Options',
            description='Maximum range value',
            default=1_000,
    )


def make_ultrasound(
        name: str,
        doc: Optional[fc.Document] = None,
) -> CrossUltrasound:
    """Add a CROSS::Ultrasound to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj: CrossUltrasound = UltrasoundProxy.create(name=name, doc=doc)

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
