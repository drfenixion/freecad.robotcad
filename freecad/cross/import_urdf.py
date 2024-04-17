import FreeCAD as fc

import FreeCADGui as fcgui

from .freecad_utils import warn
from .gui_utils import tr
from .robot_from_urdf import robot_from_urdf

try:
    from .urdf_loader import UrdfLoader
    imports_ok = True
except ImportError as e:
    # TODO: Warn the user more nicely.
    warn(str(e), gui=False)
    imports_ok = False


def open(filename: str):
    """Called when FreeCAD wants to open a file."""
    doc = fc.newDocument()
    if not doc:
        return
    load_urdf(filename, doc)


def insert(filename: str, docname: str):
    """Called when freecad wants to import a file."""
    try:
        doc = fc.getDocument(docname)
    except NameError:
        doc = fc.newDocument(docname)
    load_urdf(filename, doc)


def load_urdf(filename: str, doc: fc.Document) -> None:
    if doc is None:
        return
    urdf_robot = UrdfLoader.load_from_file(filename)
    doc.openTransaction(tr('Robot from URDF'))
    robot_from_urdf(doc, urdf_robot)
    doc.commitTransaction()
    doc.recompute()
    fcgui.SendMsgToActiveView('ViewFit')

