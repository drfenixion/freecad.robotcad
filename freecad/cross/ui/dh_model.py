"""
The model for Denavit-Hartenberg (DH) robot representation

The model for Denavit-Hartenberg (DH) robot representation
for Qt's Model/View architecture.

"""

from PySide import QtCore  # FreeCAD's PySide!

from ..kk_robot import KKRobot
from .dh_kk_model import DHKKModel


class DHModel(DHKKModel):

    columns = (
            'theta',
            'r',
            'd',
            'alpha',
            'prismatic',
    )
    column_headers = (
            'θ (rad)',
            'r (m)',
            'd (m)',
            'α (rad)',
            'Prismatic?',
    )

    column_tooltips = (
            'Θi: angle between X(i-1) and Xi about Z(i-1), in radians.',
            'ri: distance between Oi and X(i-1), in meters.',
            'di: distance between O(i-1) and Zi, in meters.',
            'ɑi: angle between Z(i-1) and Zi about X(i-1), in radians.',
            'True if the joint is prismatic, False if revolute.',
    )

    def __init__(
        self,
        kk_robot: KKRobot,
        parent=None,
    ) -> None:
        super().__init__(kk_robot, parent)

    def data(self, index, role):
        if role == QtCore.Qt.DisplayRole:
            if self.kk_robot.is_dh_compatible:
                return super().data(index, role)
            else:
                return 'N/A'
