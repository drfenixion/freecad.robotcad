''' Add inertia calculator override options to selected objects. '''

import FreeCAD as fc
import FreeCADGui as fcgui

from ..freecad_utils import add_property
from ..freecad_utils import error
from ..freecad_utils import warn
from ..gui_utils import tr
from ..wb_utils import is_joint
from ..wb_utils import is_link
from ..wb_utils import is_robot

from .inertia_calculator import PROPERTY_GROUP
from .inertia_calculator import PROP_OVERRIDE_MODE, PROP_CENTER_OF_MASS, PROP_DENSITY, PROP_IXX, PROP_IXY, PROP_IXZ, PROP_IYY, PROP_IYZ, PROP_IZZ, PROP_MASS
from .inertia_calculator import OVERRIDE_MODES
from .inertia_calculator import OVERRIDE_MODE_DENSITY, OVERRIDE_MODE_IGNORE, OVERRIDE_MODE_MANUAL, OVERRIDE_MODE_MASS, OVERRIDE_MODE_NONE

DO = fc.DocumentObject

# Tooltips of the override properties (property names are imported from
# inertia_calculator.py).
# Override modes

_OVERRIDE_MODE_TOOLTIP = (
    'Inertia override mode for this object.\n'
    f'"{OVERRIDE_MODE_NONE}": no override, use the object\'s own shape and material'
    ' as usual (default).\n'
    f'"{OVERRIDE_MODE_DENSITY}": ignore materials, use "{PROP_DENSITY}" for the object'
    ' and its whole subtree.\n'
    f'"{OVERRIDE_MODE_MASS}": ignore materials, distribute "{PROP_MASS}" over the shape'
    ' of the object and its whole subtree (uniform density is assumed).\n'
    f'"{OVERRIDE_MODE_MANUAL}": ignore shape, materials and subtree entirely, use'
    f' "{PROP_MASS}",\n "{PROP_CENTER_OF_MASS}" and "{PROP_IXX}".."{PROP_IZZ}" as supplied.\n'
    f'"{OVERRIDE_MODE_IGNORE}": ignore the object and its subtree, assume zero inertia and mass.\n'
    '\n'
    f'Inside a "{OVERRIDE_MODE_DENSITY}" or "{OVERRIDE_MODE_MASS}" object, only the shapes of'
    ' the children are used:\n'
    f'their override modes are not applied, except "{OVERRIDE_MODE_IGNORE}", which still'
    ' excludes a child.'
)

# Other override properties.
_OVERRIDE_VALUE_PROPERTIES = [
    (
        PROP_MASS, 'App::PropertyMass',
        f'Mass of the object (including its subtree). Used when "{PROP_OVERRIDE_MODE}" is "{OVERRIDE_MODE_MASS}"'
        f' or "{OVERRIDE_MODE_MANUAL}". Ignored otherwise.',
    ),
    (
        PROP_DENSITY, 'App::PropertyDensity',
        f'Density of the object and of its subtree. Used when "{PROP_OVERRIDE_MODE}" is'
        f' "{OVERRIDE_MODE_DENSITY}". Ignored otherwise.',
    ),
    (
        PROP_CENTER_OF_MASS, 'App::PropertyPlacement',
        'Center of mass in the local frame of the object. Its rotation gives the axes in which'
        f' "{PROP_IXX}".."{PROP_IZZ}" are expressed (e.g. the principal axes of inertia).'
        f' Used when "{PROP_OVERRIDE_MODE}" is "{OVERRIDE_MODE_MANUAL}". Ignored'
        ' otherwise.',
    ),
    (
        PROP_IXX, 'App::PropertyFloat',
        'Moment of inertia around the x axis, in kg*mm^2 (FreeCAD\'s native inertia unit,'
        ' same as Shape.MatrixOfInertia).'
        f' Used when "{PROP_OVERRIDE_MODE}" is "{OVERRIDE_MODE_MANUAL}". Ignored otherwise.',
    ),
    (
        PROP_IXY, 'App::PropertyFloat',
        'Moment of inertia around the y axis when rotating around the x axis, in kg*mm^2'
        ' (FreeCAD\'s native inertia unit, same as Shape.MatrixOfInertia).'
        f' Used when "{PROP_OVERRIDE_MODE}" is "{OVERRIDE_MODE_MANUAL}". Ignored otherwise.',
    ),
    (
        PROP_IXZ, 'App::PropertyFloat',
        'Moment of inertia around the z axis when rotating around the x axis, in kg*mm^2'
        ' (FreeCAD\'s native inertia unit, same as Shape.MatrixOfInertia).'
        f' Used when "{PROP_OVERRIDE_MODE}" is "{OVERRIDE_MODE_MANUAL}". Ignored otherwise.',
    ),
    (
        PROP_IYY, 'App::PropertyFloat',
        'Moment of inertia around the y axis, in kg*mm^2 (FreeCAD\'s native inertia unit,'
        ' same as Shape.MatrixOfInertia).'
        f' Used when "{PROP_OVERRIDE_MODE}" is "{OVERRIDE_MODE_MANUAL}". Ignored otherwise.',
    ),
    (
        PROP_IYZ, 'App::PropertyFloat',
        'Moment of inertia around the z axis when rotating around the y axis, in kg*mm^2'
        ' (FreeCAD\'s native inertia unit, same as Shape.MatrixOfInertia).'
        f' Used when "{PROP_OVERRIDE_MODE}" is "{OVERRIDE_MODE_MANUAL}". Ignored otherwise.',
    ),
    (
        PROP_IZZ, 'App::PropertyFloat',
        'Moment of inertia around the z axis, in kg*mm^2 (FreeCAD\'s native inertia unit,'
        ' same as Shape.MatrixOfInertia).'
        f' Used when "{PROP_OVERRIDE_MODE}" is "{OVERRIDE_MODE_MANUAL}". Ignored otherwise.',
    ),
]


class _OverrideInertialPropertiesCommand:
    """Add the "RobotCad" inertia-override properties to the selected object(s).

    This command only adds properties; it does not calculate or read
    anything. The added properties are meant to be filled in by hand in the
    Property panel afterwards and are then read by the advanced inertia
    calculation method (see `inertia_calculator.py`).

    """

    def GetResources(self) -> dict:
        return {
            'Pixmap': 'override_inertial_properties.svg',
            'MenuText': tr('Override inertial properties'),
            'ToolTip': tr(
                'Add RobotCad Override Inertial Properties to selected objects.\n'
                '\n'
                'Override properties are used in "advanced" inertia and\n'
                'mass calculation mode (see Robot\'s "InertiaCalculationMethod"\n'
                'property). These properties allow overriding the mass, the density or\n'
                'the total inertia of the selected object, or of its subtree if a\n'
                'container is selected.\n'
                '\n'
                'After the properties are added, set\n'
                f' "{PROP_OVERRIDE_MODE}" and the relevant value(s) directly in\n'
                ' the Property panel.'
            ),
        }

    def IsActive(self) -> bool:
        return bool(fcgui.Selection.getSelection())

    def Activated(self) -> None:
        objs = fcgui.Selection.getSelection()
        if not objs:
            return

        doc = fc.activeDocument()
        doc.openTransaction(tr('Override inertial properties'))

        for obj in objs:
            if is_link(obj) or is_robot(obj) or is_joint(obj):
                error(
                    f'Object "{obj.Label}" is a CROSS Link, Robot or Joint and'
                    ' already has its own Inertial Properties; skipped.',
                    gui=True,
                )
                continue
            self._add_override_properties(obj)

        doc.recompute()
        doc.commitTransaction()

    def _add_override_properties(self, obj: DO) -> None:
        """Add the override properties to `obj`, without changing any that already exist."""
        # list of existing properties
        all_names = [PROP_OVERRIDE_MODE] + [name for name, _, _ in _OVERRIDE_VALUE_PROPERTIES]
        pre_existing = [name for name in all_names if name in obj.PropertiesList]

        # freecad_utils.add_property does not replace existing properties.
        add_property(
            obj, 'App::PropertyEnumeration', PROP_OVERRIDE_MODE, PROPERTY_GROUP,
            _OVERRIDE_MODE_TOOLTIP, default = OVERRIDE_MODES
        )
        for name, type_, tooltip in _OVERRIDE_VALUE_PROPERTIES:
            add_property(obj, type_, name, PROPERTY_GROUP, tooltip)

        if pre_existing:
            label = 'property' if len(pre_existing) == 1 else 'properties'
            warn(
                f'Object "{obj.Label}" already had the {label}'
                f' {", ".join(pre_existing)}; existing value(s) left unchanged.',
                gui=True,
            )


fcgui.addCommand('OverrideInertialProperties', _OverrideInertialPropertiesCommand())
