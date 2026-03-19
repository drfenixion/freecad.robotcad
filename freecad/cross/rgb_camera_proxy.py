from __future__ import annotations

from typing import NewType
from typing import Optional
from typing import TYPE_CHECKING

import FreeCAD as fc

from pivy import coin

from .coin_utils import transform_from_placement
from .freecad_utils import warn
from .wb_utils import ICON_PATH
from .wb_utils import is_link
from freecad.cross.vendor.fcapi import fpo  # Cf. https://github.com/mnesarco/fcapi

# Typing hints.
from .rgb_camera import RgbCamera as CrossRgbCamera  # A Cross::RgbCamera, i.e. a DocumentObject with Proxy "RgbCamera". # noqa: E501

if TYPE_CHECKING and hasattr(fc, 'GuiUp') and fc.GuiUp:
    from FreeCADGui import ViewProviderDocumentObject as VPDO
else:
    VPDO = NewType('VPDO', object)


@fpo.view_proxy(
        icon=str(ICON_PATH / 'camera-photo-symbolic.svg'),
)
class RgbCameraViewProxy:
    frustum_display_mode = fpo.DisplayMode(name='Frustum', is_default=True)

    frustum_length = fpo.PropertyLength(
            name='FrustumLength',
            section='Display Options',
            default=5000,
            description=(
                'Length of the frustum representation (relative to the camera center)'
            ),
    )

    frustum_start = fpo.PropertyLength(
            name='FrustumStart',
            section='Display Options',
            default=0,
            description=(
                'Start of the frustum representation'
            ),
    )

    color = fpo.PropertyColor(
            name='Color',
            section='Display Options',
            default=(0.0, 0.5, 0.0),
            description=(
                'Color of the frustum representation'
            ),
    )

    transparency = fpo.PropertyIntegerConstraint(
            name='Transparency',
            section='Display Options',
            default=0,
            description=(
                'Transparency of the frustum representation'
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
        """Draw the frustum."""
        import math
        obj = self.Object
        view = self.ViewObject
        if not hasattr(view, 'RootNode'):
            return
        root = view.RootNode
        root.removeAllChildren()

        if not view.Visibility:
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

        # Represent the frustum as a truncated pyramid because
        # `SoFrustumCamera` is not supported by the coin3D version used in
        # FreeCAD.
        # Calculate the vertices of the truncated pyramid
        x_angle = math.radians(obj.HFov / 2)
        y_angle = math.radians(obj.VFov / 2)
        start = max(0.0, self.frustum_start)
        height = max(start, self.frustum_length)
        start_half_width = start * math.tan(x_angle)
        start_half_height = start * math.tan(y_angle)
        base_half_width = height * math.tan(x_angle)
        base_half_height = height * math.tan(y_angle)

        vertices = [
            ( start_half_width,  start_half_height, start),  # Vertices near summit.
            (-start_half_width,  start_half_height, start),
            (-start_half_width, -start_half_height, start),
            ( start_half_width, -start_half_height, start),
            ( base_half_width,  base_half_height, height),  # Vertices far.
            (-base_half_width,  base_half_height, height),
            (-base_half_width, -base_half_height, height),
            ( base_half_width, -base_half_height, height),
        ]

        # Create the faces of the frustum
        coord = coin.SoCoordinate3()
        coord.point.setValues(0, len(vertices), vertices)

        # Define the indices for the faces
        indices = [
            [0, 1, 2, 3, -1],  # Face near summit
            [0, 1, 5, 4, -1],  # Face 1
            [1, 2, 6, 5, -1],  # Face 2
            [2, 3, 7, 6, -1],  # Face 3
            [3, 0, 4, 7, -1],  # Face 4
            [4, 5, 6, 7, -1],  # Far face
        ]

        face_set = coin.SoIndexedFaceSet()
        face_set.coordIndex.setValues(0, len(indices) * 5, [i for face in indices for i in face])

        sep.addChild(coord)
        sep.addChild(face_set)
        root.addChild(sep)


@fpo.proxy(
    object_type='App::FeaturePython',
    subtype='Cross::RgbCamera',
    view_proxy=RgbCameraViewProxy,
)
class RgbCameraProxy:

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

    hfov = fpo.PropertyAngle(
            name='HFov',
            section='Sensor Options',
            default=70,
            description='Horizontal field of view of the camera',
    )

    vfov = fpo.PropertyAngle(
            name='VFov',
            section='Sensor Options',
            default=70,
            description='Vertical field of view of the camera',
    )


def make_rgb_camera(
        name: str,
        doc: Optional[fc.Document] = None,
) -> CrossRgbCamera:
    """Add a CROSS::::RgbCamera to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj: CrossRgbCamera = RgbCameraProxy.create(name=name, doc=doc)

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
