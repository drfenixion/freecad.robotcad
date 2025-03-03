from __future__ import annotations

from typing import NewType
from typing import Optional
from typing import TYPE_CHECKING

import FreeCAD as fc

from pivy import coin

from .coin_utils import text_2d
from .fpo import DisplayMode
from .fpo import PropertyInteger
from .fpo import PropertyVector
from .fpo import proxy  # Cf. https://github.com/mnesarco/fcapi
from .fpo import view_proxy
from .freecad_utils import warn
from .wb_utils import ICON_PATH

# Typing hints.
from .observer import Observer as CrossObserver, ViewProviderObserver  # A Cross::Observer, i.e. a DocumentObject with Proxy "Observer". # noqa: E501

if TYPE_CHECKING and hasattr(fc, 'GuiUp') and fc.GuiUp:
    from FreeCADGui import ViewProviderDocumentObject as VPDO
else:
    VPDO = NewType('VPDO', object)


@view_proxy(
        icon=str(ICON_PATH / 'observer.svg'),
)
class ObserverViewProxy:
    dot_display_mode = DisplayMode(name='Dot', is_default=True)

    # Unknown units.
    # Tune depending on `dot_character`.
    default_position = fc.Vector(-2.0, 0.75, 0.002)

    # Dot character.
    # Choose depending on the font support.
    # ●: Black circle, U+25CF
    # •: Bullet, U+2022
    # ⬤: Big black circle, U+2B24
    dot_character = '●'  # Black circle, U+25CF

    position = PropertyVector(
            name='Position',
            section='Display Options',
            default=default_position,
            description=(
                'position on the screen, unknown units.'
                ' (-2.0, 0.75, 0.002) corresponds approximately'
                ' to top-left (depends on screen geometry)'
            ),
    )

    def on_attach(self, vp: ViewProviderObserver) -> None:
        # `self.position` is not yet initialized, use `self.ViewObject.Position`.
        (
            self.root_node,
            self.text_node,
            self.matrix_transform_node,
            self.material_node,
        ) = text_2d(
                '',
                self.default_position,
        )

    def on_object_change(self, vp: ViewProviderObserver, prop_name: str) -> None:
        if not self.Object.ExpressionEngine:
            self.text_node.string = ''
        else:
            # TODO: check  that 'Formula' has an expression engine, no other property.
            self.text_node.string = self.dot_character
        self._set_color(self.material_node)

    @dot_display_mode.builder
    def dot_display_mode_builder(self, vp: ViewProviderObserver):
        return self.root_node

    def _set_color(self, material_node: coin.SoMaterial) -> None:
        if self.Object.Formula != 0:
            material_node.emissiveColor = (0.0, 0.5, 0.0)
        else:
            material_node.emissiveColor = (1.0, 0.0, 0.0)

    @position.observer
    def on_position_changed(
            self,
            vp: ViewProviderObserver,
    ) -> None:
        self.matrix_transform_node.matrix = coin.SbMatrix(
               0.0097401002,             0.0,             0.0, 0.0,
                        0.0,      0.00974009,     1.16877e-05, 0.0,
                        0.0,    -2.33755e-05,       0.0194802, 0.0,
            self.position.x, self.position.y, self.position.z, 1.0,
        )


@proxy(
    object_type='App::FeaturePython',
    view_proxy=ObserverViewProxy,
)
class ObserverProxy:

    # The formula is defined as integer because FreeCAD
    # doesn't support expression engine for booleans.
    formula = PropertyInteger(
            name='Formula',
            description='Use (x ? 1 : 0)',
    )


def make_observer(
        name: str,
        doc: Optional[fc.Document] = None,
) -> CrossObserver:
    """Add a CrCrossObserver to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj: CrossObserver = ObserverProxy.create(name=name, doc=doc)

    return obj
