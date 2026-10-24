"""Command to generate a robot from text description using LLM."""

from __future__ import annotations

import FreeCADGui as fcgui

from ..gui_utils import tr
from .generate_robot_by_text import show_generation_dialog


class _GenerateRobotByTextCommand:
    """The command definition to generate a robot from text description."""

    def GetResources(self):
        return {
            'Pixmap': 'generate_robot_by_text.svg',
            'MenuText': tr('Generate primitive robot by text'),
            'Accel': 'G, R',
            'ToolTip': tr('Allows you to generate a robot built from primitives based on a text description.'),
        }

    def IsActive(self):
        return True

    def Activated(self):
        show_generation_dialog(fcgui.getMainWindow())


fcgui.addCommand('GenerateRobotByText', _GenerateRobotByTextCommand())
