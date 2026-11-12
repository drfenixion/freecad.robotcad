"""Robot properties `InertiaCalculationMethod` and `InertiaCalculatorLogLevel`.

On document restore FreeCAD calls `RobotProxy.onDocumentRestored()`, which
re-runs `RobotProxy.__init__()` and thus `_init_properties()`. A robot saved
by an older version lacks the two properties; they must get their default
values, and a robot that already has them must keep the saved values.
Re-running `__init__()` on a live robot is used here to simulate the restore.

These tests need the whole workbench (`robot_proxy` imports PySide), so they
are skipped when it cannot be imported.
"""
from __future__ import annotations

import unittest

from ... import wb_constants
from .. import inertia_calculator as ic
from .base import InertiaCalculatorTestCase

METHOD = 'InertiaCalculationMethod'
LOG_LEVEL = 'InertiaCalculatorLogLevel'


class RobotInertiaPropertiesTests(InertiaCalculatorTestCase):

    def setUp(self) -> None:
        super().setUp()
        try:
            from ...robot_proxy import make_robot
        except ImportError as e:  # pragma: no cover - depends on the environment.
            raise unittest.SkipTest(f'robot_proxy cannot be imported: {e}')
        self.robot = make_robot('Robot', self.doc)

    def test_new_robot_has_defaults(self):
        self.assertEqual(self.robot.InertiaCalculationMethod, wb_constants.INERTIA_METHOD_BASIC)
        self.assertEqual(self.robot.InertiaCalculatorLogLevel, ic.LOG_LEVEL_WARN)

    def test_enumerations_mirror_calculator_constants(self):
        self.assertEqual(self.robot.getEnumerationsOfProperty(METHOD), wb_constants.INERTIA_METHODS)
        self.assertEqual(self.robot.getEnumerationsOfProperty(LOG_LEVEL), ic.LOG_LEVELS)

    def test_properties_are_in_inertial_parameters_group(self):
        self.assertEqual(self.robot.getGroupOfProperty(METHOD), 'Inertial Parameters')
        self.assertEqual(self.robot.getGroupOfProperty(LOG_LEVEL), 'Inertial Parameters')

    def test_robot_without_properties_gets_defaults_on_restore(self):
        self.robot.removeProperty(METHOD)
        self.robot.removeProperty(LOG_LEVEL)
        self.assertNotIn(METHOD, self.robot.PropertiesList)
        self.assertNotIn(LOG_LEVEL, self.robot.PropertiesList)

        self.robot.Proxy.__init__(self.robot)  # What onDocumentRestored() does.

        self.assertEqual(self.robot.InertiaCalculationMethod, wb_constants.INERTIA_METHOD_BASIC)
        self.assertEqual(self.robot.InertiaCalculatorLogLevel, ic.LOG_LEVEL_WARN)

    def test_restore_keeps_existing_values(self):
        self.robot.InertiaCalculationMethod = wb_constants.INERTIA_METHOD_ADVANCED
        self.robot.InertiaCalculatorLogLevel = ic.LOG_LEVEL_INFO

        self.robot.Proxy.__init__(self.robot)  # What onDocumentRestored() does.

        self.assertEqual(self.robot.InertiaCalculationMethod, wb_constants.INERTIA_METHOD_ADVANCED)
        self.assertEqual(self.robot.InertiaCalculatorLogLevel, ic.LOG_LEVEL_INFO)
