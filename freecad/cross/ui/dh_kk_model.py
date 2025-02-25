"""
The model for Denavit-Hartenberg (DH) or Khalil-Kleinfinger (KK) representation

The abstract class for models for Denavit-Hartenberg (DH) or
Khalil-Kleinfinger (KK) robot representation
for Qt's Model/View architecture.

"""

from PySide import QtCore  # FreeCAD's PySide!

from ..kk_robot import KKFrame
from ..kk_robot import KKRobot


class DHKKModel(QtCore.QAbstractTableModel):

    # Overwritten by subclasses.
    columns = tuple()
    column_headers = tuple()
    column_tooltips = tuple()

    def __init__(
        self,
        kk_robot: KKRobot,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.kk_robot = kk_robot

    def flags(self, index):
        return QtCore.Qt.ItemIsEditable | super().flags(index)

    def rowCount(self, index):
        # We add a row so that the user can add a joint.
        # The last row is not added to the robot until the row is complete.
        return len(self.kk_robot.kk_frames)

    def columnCount(self, index):
        return len(self.columns)

    def headerData(self, section, orientation, role):
        # section is the index of the column/row.
        if role == QtCore.Qt.DisplayRole:
            if orientation == QtCore.Qt.Horizontal:
                try:
                    return str(self.column_headers[section])
                except IndexError:
                    return '?'

            if orientation == QtCore.Qt.Vertical:
                # 1-based.
                return str(section + 1)
        if role == QtCore.Qt.ToolTipRole:
            if orientation == QtCore.Qt.Horizontal:
                try:
                    return self.column_tooltips[section]
                except IndexError:
                    return ''
            else:
                return ''

    def data(self, index, role):
        if role == QtCore.Qt.DisplayRole:
            return self._get_cell_value(index)

    def setData(self, index, value, role):
        # TODO:  The role will always be set to Qt::EditRole because our cells
        # only contain text. If a checkbox were present and user permissions
        # are set to allow the checkbox to be selected, calls would also
        # be made with the role set to Qt::CheckStateRole.
        if role == QtCore.Qt.EditRole:
            if (not self.checkIndex(index)):
                return False
            return self._set_cell_value(index, value)

    def _get_edited_joint(
        self,
        index,
    ) -> KKFrame:
        """Return `self.kk_robot.joints[i]`."""
        return self.kk_robot.kk_frames[index.row()]

    def _get_joint_value(
        self,
        kk_joint: KKFrame,
        index,
    ) -> str:
        """Return the appropriate field."""
        attr_str = self.columns[index.column()]
        value = getattr(kk_joint, attr_str)
        return value

    def _get_joint_data(
        self,
        kk_joint: KKFrame,
        index,
    ) -> str:
        """Return the appropriate field as string."""
        return str(self._get_joint_value(kk_joint, index))

    def _set_joint_data(
        self,
        kk_joint: KKFrame,
        index,
        value,
    ) -> bool:
        """Set the appropriate field."""
        kk_joint = self._get_edited_joint(index)
        attr_str = self.columns[index.column()]
        if attr_str == 'prismatic':
            try:
                setattr(
                    kk_joint,
                    attr_str,
                    value.casefold() == 'true',
                )
            except IndexError:
                # DHModel has less columns than KKModel.
                pass
            except ValueError:
                return False
        else:
            try:
                setattr(
                    kk_joint,
                    attr_str,
                    float(value),
                )
            except IndexError:
                # DHModel has less columns than KKModel.
                pass
            except ValueError:
                return False
        return True

    def _get_cell_value(
        self,
        index,
    ) -> str:
        """Return the cell content."""
        kk_joint = self._get_edited_joint(index)
        return self._get_joint_data(kk_joint, index)

    def _set_cell_value(
        self,
        index,
        value,
    ) -> bool:
        kk_joint = self._get_edited_joint(index)
        return self._set_joint_data(kk_joint, index, value)

    def add_joint(self) -> None:
        """Add a joint to the robot."""
        self.kk_robot.kk_frames.append(KKFrame(0.0, 0.0, 0.0, 0.0))
