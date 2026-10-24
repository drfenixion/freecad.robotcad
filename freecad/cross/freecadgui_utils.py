"""Functions that could have belonged to FreeCADGui."""

from __future__ import annotations

import FreeCAD as fc
import FreeCADGui as fcgui
from copy import deepcopy
import re
import traceback
from typing import Optional

try:
    from PySide import QtWidgets, QtGui, QtCore
    from PySide.QtWidgets import QMessageBox
except:
    from PySide6 import QtWidgets, QtGui, QtCore
    from PySide6.QtWidgets import QMessageBox

from .freecad_utils import copy_obj_geometry, get_subobjects_by_full_name
from .freecad_utils import first_object_with_volume
from .freecad_utils import is_lcs
from .freecad_utils import is_part
from .freecad_utils import message
from .freecad_utils import is_link as is_fc_link
from .placement_utils import get_global_placement

# Typing hints.
DO = fc.DocumentObject
SO = 'FreeCADGui.SelectionObject'  # Could not get the class from Python.


def get_subobjects_and_placements(
        selection: list[SO],
) -> list[tuple[DO, fc.Placement]]:
    """Return the list of selected subobjects and their placement."""
    outlist: list[tuple[DO, fc.Placement]] = []
    for selection_object in selection:
        root_obj = selection_object.Object
        sub_fullpaths = selection_object.SubElementNames
        if not sub_fullpaths:
            # An object is selected, not a face, edge, vertex.
            sub_fullpaths = ('',)
        for sub_fullpath in sub_fullpaths:
            subobjects = get_subobjects_by_full_name(root_obj, sub_fullpath)
            if subobjects:
                obj = subobjects[-1]
            else:
                obj = root_obj
            # One or more subelements are selected.
            placement = get_global_placement(root_obj, sub_fullpath)
            outlist.append((obj, placement))
    return outlist


def createBoundBox(obj):
    return createBoundAbstract(obj, createPrimitive = createBox)


def createBoundXAlignedCylinder(obj):
    return createBoundAbstract(obj, createPrimitive = createXAlignedCylinder)


def createBoundYAlignedCylinder(obj):
    return createBoundAbstract(obj, createPrimitive = createYAlignedCylinder)


def createBoundZAlignedCylinder(obj):
    return createBoundAbstract(obj, createPrimitive = createZAlignedCylinder)


def createBoundSphere(obj):
    return createBoundAbstract(obj, createPrimitive = createSphere)


def createBox(boundBox_: DO, nameLabel: str) -> DO:
    boundObj = fc.ActiveDocument.addObject("Part::Box", nameLabel + "_BoundBox")
    boundObj.Length.Value = boundBox_.XLength
    boundObj.Width.Value  = boundBox_.YLength
    boundObj.Height.Value = boundBox_.ZLength

    boundObj.Placement.Base = fc.Vector(boundBox_.XMin,boundBox_.YMin,boundBox_.ZMin)

    return boundObj


def createXAlignedCylinder(boundBox_: DO, nameLabel: str):
    return createCylinder(boundBox_, nameLabel, alignAxis = 'x')


def createYAlignedCylinder(boundBox_: DO, nameLabel: str):
    return createCylinder(boundBox_, nameLabel, alignAxis = 'y')


def createZAlignedCylinder(boundBox_: DO, nameLabel: str):
    return createCylinder(boundBox_, nameLabel, alignAxis = 'z')


def createCylinder(boundBox_: DO, nameLabel: str, alignAxis: str = 'x') -> DO:

    def createAlignedCylinder(boundBox_: DO, nameLabel: str, alignAxis: str = 'z') -> DO:
        "Create aligned cynlinder by selectable axis"
        if alignAxis == 'z':
            firstAxisLength = boundBox_.ZLength
            secondAxisLength = boundBox_.XLength
            thirdAxisLength = boundBox_.YLength
            secondAxisCenter = boundBox_.Center.x
            thirdAxisCenter = boundBox_.Center.y
            firstAxisMin = boundBox_.ZMin
        elif alignAxis == 'x':
            firstAxisLength = boundBox_.XLength
            secondAxisLength = boundBox_.YLength
            thirdAxisLength = boundBox_.ZLength
            secondAxisCenter = boundBox_.XMin
            thirdAxisCenter = boundBox_.YMin - (boundBox_.YMin - boundBox_.Center.y)
            firstAxisMin = boundBox_.ZMin - (boundBox_.ZMin - boundBox_.Center.z)
        elif alignAxis == 'y':
            firstAxisLength = boundBox_.YLength
            secondAxisLength = boundBox_.ZLength
            thirdAxisLength = boundBox_.XLength
            secondAxisCenter = boundBox_.XMin - (boundBox_.XMin - boundBox_.Center.x)
            thirdAxisCenter = boundBox_.YMin - (boundBox_.YMin - boundBox_.Center.y) * 2
            firstAxisMin = boundBox_.ZMin - (boundBox_.ZMin - boundBox_.Center.z)
        else:
            raise RuntimeError('Wrong cynlinder alignAxis (' + alignAxis + ')')



        boundObj = fc.ActiveDocument.addObject('Part::Cylinder', nameLabel + "_BoundCylinder")
        boundObj.Height = firstAxisLength
        boundObj.Radius = ((secondAxisLength ** 2 + thirdAxisLength ** 2) ** 0.5) / 2.0
        boundObj.Placement.Base = fc.Vector(secondAxisCenter, thirdAxisCenter, firstAxisMin)
        # z is default orientation
        if alignAxis == 'x':
            # rotate by y
            boundObj.Placement.rotate(fc.Vector(0,0,0), fc.Vector(0,1,0), 90)
        if alignAxis == 'y':
            # rotate by x
            boundObj.Placement.rotate(fc.Vector(0,0,0), fc.Vector(1,0,0), 90)

        return boundObj

    boundObj = createAlignedCylinder(boundBox_, nameLabel, alignAxis)

    return boundObj


def createSphere(boundBox_: DO, nameLabel: str) -> DO:
    boundObj = fc.ActiveDocument.addObject('Part::Sphere', nameLabel + "_BoundSphere")
    boundObj.Radius = boundBox_.DiagonalLength / 2.0
    boundObj.Placement.Base = boundBox_.Center

    return boundObj


def createCollisionCopyObj(obj: DO) -> DO:
    """Create copy of object for collision purpose"""
    colObj = fc.ActiveDocument.addObject("Part::Feature", obj.Label + "_col_obj")
    colObj = copy_obj_geometry(obj, colObj)

    colObj = set_collision_appearance(colObj)

    return colObj


def createBoundAbstract(obj, createPrimitive = createBox):

    obj = first_object_with_volume(obj)

    if hasattr(obj, "Shape"):
        s = obj.Shape
    elif hasattr(obj, "Mesh"):      # upgrade with wmayer thanks #http://forum.freecadweb.org/viewtopic.php?f=13&t=22331
        s = obj.Mesh
    elif hasattr(obj, "Points"):
        s = obj.Points

    boundObj = False
    try:
        # boundBox
        boundBox_    = s.BoundBox
        boundBoxLX   = boundBox_.XLength
        boundBoxLY   = boundBox_.YLength
        boundBoxLZ   = boundBox_.ZLength
        boundBoxXMin = boundBox_.XMin
        boundBoxYMin = boundBox_.YMin
        boundBoxZMin = boundBox_.ZMin

        nameLabel  = obj.Label

        try:
            import unicodedata
            nameLabel = str(unicodedata.normalize('NFKD', nameLabel).encode('ascii','ignore'))[2:]
        except Exception:
            None

        fc.Console.PrintMessage(str(boundBox_)+"\r\n")
        fc.Console.PrintMessage("Rectangle      : "+str(boundBoxLX)+" x "+str(boundBoxLY)+" x "+str(boundBoxLZ)+"\r\n")

        if (boundBoxLX > 0) and (boundBoxLY > 0) and (boundBoxLZ > 0):  # Create Volume

            boundObj = createPrimitive(boundBox_, nameLabel)

            boundObj = set_collision_appearance(boundObj)

    except Exception:
        fc.Console.PrintError("Bad selection"+"\n")

    return boundObj


def set_collision_appearance(boundObj):
    boundObjGui = fcgui.ActiveDocument.getObject(boundObj.Name)
    if is_fc_link(boundObj):
        boundObjGui.ShapeMaterial = fc.Material(
            DiffuseColor=(1.00,1.00,0.4),
            AmbientColor=(1.00,1.00,0.4),
            SpecularColor=(1.00,1.00,0.4),
            EmissiveColor=(1.00,1.00,0.4),
            Shininess=(0.90),
            Transparency=(0.70),
        )
        boundObjGui.OverrideMaterial = True
    else:
        # LineColor
        red   = 1.0  # 1 = 255
        green = 1.0  #
        blue  = 0.4  #

        boundObjGui.LineColor  = (red, green, blue)
        boundObjGui.PointColor = (red, green, blue)
        boundObjGui.ShapeColor = (red, green, blue)
        boundObjGui.LineWidth = 1
        boundObjGui.Transparency = 90
    return boundObj


def get_placement(
        orienteer1: DO,
) -> fc.Placement:
    """Return absolute coordinates of orienteer."""
    resolve_mode_resolve = 0 # 0 - absolute, 1 relative
    selection = fcgui.Selection.getSelectionEx('', resolve_mode_resolve)
    objects_placements = get_subobjects_and_placements(selection)
    objects, placements = zip(*objects_placements)
    orienteer1_placement = placements[objects.index(orienteer1)]

    return orienteer1_placement


def get_placements(
        orienteer1: DO,
        orienteer2: DO,
) -> fc.Placement:
    """Return the transform from `lcs` to `obj`."""
    resolve_mode_resolve = 0 # 0 - absolute, 1 relative
    selection = fcgui.Selection.getSelectionEx('', resolve_mode_resolve)
    objects_placements = get_subobjects_and_placements(selection)
    objects, placements = zip(*objects_placements)
    orienteer1_placement = placements[objects.index(orienteer1)]
    orienteer2_placement = placements[objects.index(orienteer2)]

    return orienteer1_placement, orienteer2_placement


def getSelectedPropertiesAndObjectsInTreeView() -> tuple[list, list]:
    """Get selected properties of treeView in order of selection.

    Return selected properties and objects."""

    def has_parent(index):
        return index.parent().isValid()

    mw = fcgui.getMainWindow()
    trees = mw.findChildren(QtWidgets.QTreeView)

    props=[]
    for tree in trees:
        prop_default = {'type': 'property', 'name': None, 'value': None, 'description': None}
        prop = deepcopy(prop_default)
        n= 0
        for index in tree.selectedIndexes():

            n=n+1
            itemData = index.model().itemData(index)
            if itemData!={} :
                if 0 in itemData  :
                    if 1 in itemData :
                        object = fc.ActiveDocument.getObjectsByLabel(itemData[0])[0]
                        props.append({'type': 'object', 'name': itemData[0], 'object': object})
                        prop = deepcopy(prop_default)
                        n = 0
                    else:
                        if n==1 :
                            tabProperty=[itemData[0]]
                            parent = index.parent()
                            tabProperty.append(parent.data())
                            while has_parent(parent):
                                parent=parent.parent()
                                tabProperty.append(parent.data())
                            # if name have category with '___' in last it can miss one '_' in name
                            # same as displayed name
                            prop['name'] = tabProperty # name consists of segments of name
                            if 3 in itemData :
                                # use full name instead of name
                                prop['full_name'] = re.findall('\\nName: (.+)\\n\\n', itemData[3])[0]

                        elif n==2 :
                            prop['value'] = itemData[0]  # value

                            if 3 in itemData : # tip
                                    prop['description'] = itemData[3]

            if n == 2 :
                props.append(prop)
                prop = deepcopy(prop_default)
                n = 0

    # separate props and objects
    properties = []
    objects = []
    for el in props:
        if 'type' in el:
            if el['type'] == 'property':
                properties.append(el)
            elif el['type'] == 'object':
                objects.append(el)

    return properties, objects


def get_sorted_concated_names(objs: list[DO]) -> str:
    """Get sorted concated names of objects as string.
    Usefull for check equal objects groups"""
    objs_names = []
    for link_or_obj in objs:
        try:
            obj = link_or_obj.getLinkedObject(True)
            name = obj.Name
            if obj.Name is None:
                name = 'None'
            objs_names.append(name)
        except ReferenceError:
            pass
    objs_names.sort()
    objs_names_concated = '_'.join(objs_names)
    return objs_names_concated


def refresh_objects_trees(objects: list[DO]) -> None:
    """Rebuild the construction-tree nodes that hold the given objects.

    FreeCAD's tree does not always rebuild a branch after the generated
    children of an `App::Part` are deleted and recreated (for example when the
    Real/Visual/Collision level of detail is toggled). The children then show
    up at the root of the construction tree until the branch is expanded by
    hand, even though they are still children of the part.

    Expanding a branch makes FreeCAD re-query its children and show the objects
    under their actual parent. Only the nodes that match `objects` are expanded
    and then collapsed again, so the rest of the tree and the other documents
    are left untouched.

    Parameters
    ----------
    - objects: the document objects whose tree nodes hold the level-of-detail
      objects that may fall to the root (typically the `App::Part`s referenced
      by `Link.Real` / `Link.Visual` / `Link.Collision`).

    The tree is iterated through its model, which works for a `QTreeWidget` as
    well as for a model-based `QTreeView`. Only the matched node itself is
    expanded (to reveal its direct children, which are the level-of-detail
    objects that may have fallen to the root) and then collapsed again; its
    descendants and the rest of the tree are left untouched. It does nothing
    without GUI.

    """
    if not (hasattr(fc, 'GuiUp') and fc.GuiUp):
        return
    try:
        main_window = fcgui.getMainWindow()
    except Exception:
        fc.Console.PrintError('refresh_objects_trees: cannot get main window\n')
        fc.Console.PrintError(traceback.format_exc())
        return

    object_names: set[str] = set()
    if objects:
        for o in objects:
            try:
                if getattr(o, 'Name', None):
                    object_names.add(o.Name)
                if getattr(o, 'Label', None):
                    object_names.add(o.Label)
            except Exception:
                pass

    def _iter_indices(model):
        """Yield the model index of every item, depth-first."""
        stack = [QtCore.QModelIndex()]
        while stack:
            parent = stack.pop()
            try:
                rows = model.rowCount(parent)
            except Exception:
                continue
            for row in range(rows):
                idx = model.index(row, 0, parent)
                if not idx.isValid():
                    continue
                yield idx
                stack.append(idx)

    def _index_name(model, idx) -> Optional[str]:
        """Return the object name stored in a tree index, if any."""
        for role in (QtCore.Qt.UserRole, QtCore.Qt.UserRole + 1,
                     QtCore.Qt.UserRole + 2):
            try:
                data = model.data(idx, role)
            except Exception:
                continue
            if isinstance(data, str) and data:
                return data
        return None

    def _index_text(model, idx) -> Optional[str]:
        """Return the display text of a tree index, if any."""
        try:
            data = model.data(idx, QtCore.Qt.DisplayRole)
        except Exception:
            return None
        return data if isinstance(data, str) else None

    def _refresh_index(tree, idx) -> None:
        """Re-query the children of `idx` and restore `idx`'s own state.

        Only `idx` itself is expanded (revealing its direct children, which are
        the level-of-detail objects that may have fallen to the root) and then
        collapsed again, so its descendants and the rest of the tree are left
        untouched. `idx` is finally put back in the state it had before.
        """
        try:
            was_expanded = tree.isExpanded(idx)
        except Exception:
            was_expanded = False
        try:
            tree.expand(idx)
        except Exception:
            return
        try:
            tree.collapse(idx)
        except Exception:
            pass
        if was_expanded:
            try:
                tree.expand(idx)
            except Exception:
                pass

    def _rebuild_trees() -> None:
        try:
            # `findChildren(QTreeView)` also returns the `QTreeWidget`
            # subclasses FreeCAD uses for its document tree.
            trees = main_window.findChildren(QtWidgets.QTreeView)
        except Exception:
            fc.Console.PrintError('refresh_objects_trees: cannot find tree views\n')
            fc.Console.PrintError(traceback.format_exc())
            return
        for tree in trees:
            try:
                model = tree.model()
                if model is None:
                    # The document tree has no items yet.
                    continue
                if object_names:
                    # Refresh only the nodes of the given objects (the
                    # level-of-detail sources). The tree is iterated through its
                    # model, which works for a `QTreeWidget` as well as for a
                    # model-based `QTreeView` such as `DocumentTreeItems`.
                    for idx in _iter_indices(model):
                        name = _index_name(model, idx)
                        text = _index_text(model, idx)
                        if ((name in object_names)
                                or (text in object_names)):
                            _refresh_index(tree, idx)
            except Exception:
                fc.Console.PrintError('refresh_objects_trees: rebuild failed\n')
                fc.Console.PrintError(traceback.format_exc())

    _rebuild_trees()


def get_progress_bar(title:str = '', min: int = 0, max: int = 0, show_percents:bool = True):
    # Create a progress bar
    progressBar = QtGui.QProgressBar()
    # progressBar.setGeometry(10, 10, 280, 30)
    progressBar.setRange(min, max)
    percents = ''
    if show_percents:
        percents = " %p%"
    progressBar.setFormat(title + percents)
    mw = fcgui.getMainWindow()
    mw.statusBar().setStyleSheet("QStatusBar::item { border: none; }")
    mw.statusBar().addWidget(progressBar, stretch=1)

    return progressBar


def get_report_view_text() -> str:
    """Get all console messages currently in the FreeCAD report view."""
    try:
        mw = fcgui.getMainWindow()
        report_view = mw.findChild(QtWidgets.QTextEdit, "Report view")
        if report_view:
            return report_view.toPlainText()
    except Exception:
        pass
    return ""


def gui_process_events():
    """Use for unblock gui between ticks when sothing hard calculating"""
    QtGui.QApplication.processEvents()


def ask_confirmation(title:str = '', text:str = '') -> bool:
    """Display confirmation popup to user."""
    msg_box = QMessageBox()
    msg_box.setWindowTitle(title)
    msg_box.setText(text)
    msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
    
    result = msg_box.exec_()
    
    return result == QMessageBox.Yes


def show_notification(title, message):
    # Получаем главное окно FreeCAD
    mw = fcgui.getMainWindow()
    
    # Создаем окно сообщения
    msg_box = QtWidgets.QMessageBox(mw)
    msg_box.setIcon(QtWidgets.QMessageBox.Information) # Или Warning, Critical
    msg_box.setWindowTitle(title)
    msg_box.setText(message)
    msg_box.setStandardButtons(QtWidgets.QMessageBox.Ok)
    
    # Показываем окно
    msg_box.exec_()
    