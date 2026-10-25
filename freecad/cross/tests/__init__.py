"""Tests of RobotCAD workbench utilities (`freecad.cross`).

Run with a FreeCAD-aware Python interpreter, for example::

    freecadcmd -m unittest discover -s freecad/cross/tests -t .

or from the FreeCAD Python console::

    import unittest, freecad.cross.tests.test_make_lcs_at_link_body as t
    unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(t))

Each `test_*.py` module describes in its docstring what it covers.
"""
