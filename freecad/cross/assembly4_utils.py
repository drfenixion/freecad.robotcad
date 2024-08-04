# Emulate some functions from module Asm4_libs v0.12.5.

from typing import List, Optional, Tuple

import FreeCAD as fc

from .freecad_utils import add_property

# Typing hints.
DO = fc.DocumentObject
LabelAndExpression = Tuple[str, str]
ExpressionEngine = List[LabelAndExpression]


def add_asm4_properties(obj: DO):
    """Render `obj` compatible with Assembly4."""
    # Adapted from Asm4_libs.makeAsmProperties.
    # add_property(obj, 'App::PropertyString', 'AssemblyType', 'Assembly', '')
    add_property(obj, 'App::PropertyString', 'AttachedBy', 'Assembly', '')
    add_property(obj, 'App::PropertyString', 'AttachedTo', 'Assembly', '')
    add_property(
        obj, 'App::PropertyPlacement', 'AttachmentOffset', 'Assembly',
        '',
    )
    add_property(obj, 'App::PropertyString', 'SolverId', 'Assembly', '')
    # obj.AssemblyType = 'Part::Link'
    obj.SolverId = 'Asm4EE'


# TODO: Use `from Asm4_libs import createVariables as _new_variable_container`
#       (but fallback if not importable)
# From Asm4_libs
def new_variable_container(
        assembly: DO,
) -> DO:
    """Return a variable container for Assembly4."""
    if ((not hasattr(assembly, 'TypeId'))
            or (assembly.TypeId != 'App::Part')):
        raise RuntimeError(
                'First argument must be an `App::Part` FreeCAD object',
        )
    # There is no object "Variables", so we create it.
    variables = assembly.Document.addObject('App::FeaturePython', 'Variables')
    if hasattr(variables, 'ViewObject') and variables.ViewObject:
        # TODO
        # variables.ViewObject.Proxy = ViewProviderCustomIcon(obj,
        #                                            path + "FreeCADIco.png")
        # variables.ViewObject.Proxy = setCustomIcon(object,
        #                                    'Asm4_Variables.svg')
        pass
    # Signature of a PropertyContainer.
    variables.addProperty('App::PropertyString', 'Type')
    variables.Type = 'App::PropertyContainer'
    assembly.addObject(variables)
    return variables


def _get_placement_label_and_expression(
        expression_engine: ExpressionEngine,
) -> Optional[LabelAndExpression]:
    if not isinstance(expression_engine, list):
        return
    if not expression_engine:
        return
    for label_and_expr in expression_engine:
        if len(label_and_expr) < 2:
            continue
        if label_and_expr[0] == 'Placement':
            return label_and_expr


def get_placement_expression(
        expression_engine: ExpressionEngine,
) -> Optional[str]:
    # Original name: placementEE.
    label_and_expr = _get_placement_label_and_expression(expression_engine)
    if label_and_expr:
        return label_and_expr[1]


def update_placement_expression(
        obj: DO,
        expression: str,
) -> None:
    if not hasattr(obj, 'setExpression'):
        return
    obj.setExpression('Placement', expression)
    obj.setEditorMode('Placement', ['ReadOnly'])
