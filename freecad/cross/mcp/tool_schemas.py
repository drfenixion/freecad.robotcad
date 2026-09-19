"""Pure-Python tool schemas for the RobotCAD MCP server.

This module must NOT import FreeCAD or any workbench module, so that the
standalone stdio bridge can load it without triggering the heavy workbench
package initialization (``freecad/cross/__init__.py``).

Each entry maps a tool name to a JSON-schema-like dict with ``properties`` and
``required``. The schemas mirror the signatures in ``tools_registry.py``.
Descriptions are kept terse to minimize agent token usage.
"""

from __future__ import annotations

TOOL_SCHEMAS: dict[str, dict] = {
    'create_document': {
        'properties': {
            'name': {'type': 'string', 'description': 'Document name (default "Unnamed")'},
        },
        'required': [],
    },
    'create_robot': {
        'properties': {
            'name': {'type': 'string', 'description': 'Robot name (default "Robot")'},
        },
        'required': [],
    },
    'create_link': {
        'properties': {
            'robot': {'type': 'string', 'description': 'Cross::Robot name or Label'},
            'name': {'type': 'string', 'description': 'Link name (default "Link")'},
            'add_to_robot': {'type': 'boolean', 'description': 'Add to robot (default true)'},
        },
        'required': ['robot'],
    },
    'create_links_filled': {
        'properties': {
            'robot': {'type': 'string', 'description': 'Cross::Robot name or Label'},
            'object_names': {
                'type': 'array',
                'items': {'type': 'string'},
                'description': 'Objects (Part/Body/App::Link) to fill links from, in link order',
            },
        },
        'required': ['robot', 'object_names'],
    },
    'create_joint': {
        'properties': {
            'robot': {'type': 'string', 'description': 'Cross::Robot name or Label'},
            'name': {'type': 'string', 'description': 'Joint name (default "Joint")'},
            'parent_link': {'type': 'string', 'description': 'Parent link name'},
            'child_link': {'type': 'string', 'description': 'Child link name'},
            'type': {'type': 'string', 'description': 'fixed, revolute, prismatic, continuous, ...'},
            'axis': {'type': 'array', 'items': {'type': 'number'}, 'description': 'Joint axis [x, y, z]: local Z is rotated to point along it; revolute joints rotate around Z, prismatic move along Z'},
            'lower': {'type': 'number', 'description': 'Lower limit (deg or mm)'},
            'upper': {'type': 'number', 'description': 'Upper limit (deg or mm)'},
            'effort': {'type': 'number', 'description': 'Max effort (N or Nm)'},
            'velocity': {'type': 'number', 'description': 'Max velocity (deg/s or mm/s)'},
        },
        'required': ['robot'],
    },
    'create_joints_filled': {
        'properties': {
            'robot': {'type': 'string', 'description': 'Cross::Robot name or Label'},
            'link_names_in_order': {
                'type': 'array',
                'items': {'type': 'string'},
                'description': 'Links in connection order; first is the spider hub',
            },
            'connect_type': {
                'type': 'string',
                'enum': ['chain', 'spider'],
                'description': 'chain = consecutive, spider = all to first',
            },
        },
        'required': ['robot', 'link_names_in_order'],
    },
    'create_collision': {
        'properties': {
            'link_or_robot': {'type': 'string', 'description': 'Link or robot name or Label'},
            'type': {
                'type': 'string',
                'enum': ['copy', 'box', 'sphere', 'cylinder_z', 'cylinder_x', 'cylinder_y'],
                'description': (
                    'Default "copy" (exact geometry copy). Primitives (from the '
                    'bounding box) ONLY on explicit user request.'
                ),
            },
        },
        'required': ['link_or_robot'],
    },
    # set_object_placement is disabled: set_placement_between is the default
    # positioning method. Re-add the schema only on explicit request.
    # 'set_object_placement': { ... },
    'set_placement_between': {
        'properties': {
            'target': {'type': 'string', 'description': 'Link or joint to position'},
            'ref1': {
                'type': 'string',
                'description': (
                    'Ref 1 (PRIMARY method): face/edge/vertex/circle of a link '
                    "Real element as '<real_link>.<body>.<subelement>' (e.g. "
                    "'real_l_chassis001_.chassis001.Box.Face3') or an already "
                    'existing LCS. A robot link (l_...) cannot be a ref. Must '
                    'lie on the parent link. Do NOT create LCS objects — plain '
                    'subelement references are enough.'
                ),
            },
            'ref2': {
                'type': 'string',
                'description': 'Ref 2: same format; must lie on the child link.',
            },
            'move': {
                'type': 'string',
                'enum': ['leaf', 'child_branch', 'parent_tree'],
                'description': "'leaf' (default) = final chain element only; others are advanced",
            },
        },
        'required': ['target', 'ref1', 'ref2'],
    },
    'rotate_object': {
        'properties': {
            'object_name': {'type': 'string', 'description': 'Joint, link or LCS name'},
            'axis': {'type': 'string', 'enum': ['x', 'y', 'z']},
            'angle_deg': {'type': 'number', 'description': 'Angle in degrees'},
        },
        'required': ['object_name'],
    },
    'create_lcs': {
        'properties': {
            'link': {'type': 'string', 'description': 'Cross::Link name or Label'},
            'subelement': {
                'type': 'string',
                'description': (
                    "Subelement of the link Real element: '<real_link>.<body>."
                    "<subelement>' (e.g. 'real_l_chassis001_.chassis001.Box."
                    "Face3'); short forms 'chassis001.Box.Face3' / 'Box.Face3' "
                    'accepted; empty = Real origin. ONLY on explicit user '
                    'request: the default positioning workflow never needs an '
                    'LCS — use plain subelement references in '
                    'set_placement_between instead.'
                ),
            },
        },
        'required': ['link'],
    },
    'set_placement_vision_mode': {
        'properties': {
            'robot': {
                'type': 'string',
                'description': 'Cross::Robot name; empty = all robots',
            },
        },
        'required': [],
    },
    'set_material': {
        'properties': {
            'object_name': {'type': 'string', 'description': 'Link or robot name'},
            'material_card_path': {
                'type': 'string',
                'description': '.FCMat path or material name (e.g. "ABS-Generic")',
            },
            'density': {
                'type': 'number',
                'description': 'Density kg/m^3 (from the card if omitted)',
            },
            'card_name': {'type': 'string', 'description': 'Material card name'},
        },
        'required': ['object_name'],
    },
    'calculate_mass_and_inertia': {
        'properties': {
            'robot_or_link_names': {
                'type': 'array',
                'items': {'type': 'string'},
                'description': 'Robot name or list of link names',
            },
        },
        'required': ['robot_or_link_names'],
    },
    'set_joint_values': {
        'properties': {
            'robot': {'type': 'string', 'description': 'Robot name'},
            'values': {
                'type': 'object',
                'additionalProperties': {'type': 'number'},
                'description': 'Joint name -> value (deg or mm)',
            },
        },
        'required': ['robot', 'values'],
    },
    'list_scene_objects': {
        'properties': {},
        'required': [],
    },
    'get_object_info': {
        'properties': {
            'object_name': {'type': 'string', 'description': 'Object name or label'},
        },
        'required': ['object_name'],
    },
    'get_snapshot': {
        'properties': {
            'path': {'type': 'string', 'description': 'Output path (temp file if omitted)'},
            'width': {'type': 'integer', 'description': 'Width (default 1024)'},
            'height': {'type': 'integer', 'description': 'Height (default 768)'},
            'format': {'type': 'string', 'enum': ['png', 'jpg', 'bmp'], 'description': 'Image format'},
        },
        'required': [],
    },
    'instructions_to_work_with_tools': {
        'properties': {
            'topic': {
                'type': 'string',
                'enum': [
                    'all',
                    'general_algorithm',
                    'create_robot',
                    'positioning',
                    'joints',
                    'collisions',
                    'materials',
                    'inspection',
                    'best_practices',
                ],
                'description': 'Section to return (default "all"); "general_algorithm" is the source of truth',
            },
        },
        'required': [],
    },
}


def get_tool_schemas() -> dict[str, dict]:
    """Return the tool schemas dict."""
    return TOOL_SCHEMAS
