"""
The model for Khalil-Kleinfinger (KK) robot representation

The model for Khalil-Kleinfinger (KK) robot representation
for Qt's Model/View architecture.

"""

from ..kk_robot import KKRobot
from .dh_kk_model import DHKKModel


class KKModel(DHKKModel):

    columns = (
            'theta',
            'r',
            'd',
            'alpha',
            'gamma',
            'epsilon',
            'prismatic',
    )
    column_headers = (
            'θ (rad)',
            'r (m)',
            'd (m)',
            'α (rad)',
            'ɣ (rad)',
            'ε (m)',
            'Prismatic?',
    )

    column_tooltips = (
            'Θi: angle between X(i-1) and Xi about Zi, in radians.',
            'ri: distance between Oi and X(i-1), in meters.',
            'di: distance between O(i-1) and Zi, in meters.',
            'ɑi: angle between Z(i-1) and Zi about X(i-1), in radians.',
            "γi: angle between Xi and X'i about Zi, in radians.",
            "εi: distance between Oi and O'i, in meters.",
            'True if the joint is prismatic, False if revolute.',
    )

    def __init__(
        self,
        kk_robot: KKRobot,
        parent=None,
    ) -> None:
        super().__init__(kk_robot, parent)
