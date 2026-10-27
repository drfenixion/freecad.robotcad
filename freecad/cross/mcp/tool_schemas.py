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
    'create_collisions': {
        'properties': {
            'link_or_robot': {
                'type': 'array',
                'items': {'type': 'string'},
                'description': (
                    'Link/robot names or Labels. A robot covers all its links; '
                    'its links passed explicitly are skipped (no duplicates).'
                ),
            },
            'type': {
                'oneOf': [
                    {
                        'type': 'string',
                        'enum': ['copy', 'box', 'sphere', 'cylinder_z', 'cylinder_x', 'cylinder_y'],
                    },
                    {
                        'type': 'array',
                        'items': {
                            'type': 'string',
                            'enum': ['copy', 'box', 'sphere', 'cylinder_z', 'cylinder_x', 'cylinder_y'],
                        },
                    },
                ],
                'description': (
                    'One type for every element, or a list parallel to '
                    'link_or_robot. Default "copy" (exact geometry). Primitives '
                    '(from the bounding box) ONLY on explicit user request.'
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
            'target': {
                'type': 'string',
                'description': (
                    'Link or joint to position. NEVER on a sensor link hosting a '
                    'camera or similar forward-looking sensor (use '
                    'set_placement_between_sensor_link_and_parent). EXCEPTION: a '
                    'link hosting only a horizontal lidar IS positioned with this '
                    'tool, then its parent joint is rotated about Z.'
                ),
            },
            'ref1': {
                'type': 'string',
                'description': (
                    'Reference on the PARENT link: face/edge/vertex/circle of its '
                    "Real element as '<real_link>.<inner_link_name>.<feature>."
                    "<subelement>' (e.g. 'real_l_chassis001_.chassis001.Box."
                    "Face3'), or an existing LCS. <inner_link_name> is the Name "
                    'of the App::Link inside the Real element (e.g. '
                    "'chassis001'), NOT the source body name. A robot link "
                    '(l_...) cannot be a ref. Do NOT create LCS objects.'
                ),
            },
            'ref2': {
                'type': 'string',
                'description': (
                    'Reference on the CHILD link, same format. If the call '
                    "succeeds but the target did NOT move, both references "
                    'resolved to the same point — verify with '
                    'get_objects_info([target]) and pick different ones.'
                ),
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
            'axis': {
                'type': 'string',
                'enum': ['x', 'y', 'z'],
                'description': (
                    'Rotation axis. Orient a jointed link by rotating the JOINT '
                    '(its local Z is the functional axis). MIRROR a link to the '
                    'other side of the parent by rotating the LINK 180° about X '
                    '(NOT Z: that only spins it around its own functional axis).'
                ),
            },
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
                    "Subelement of the link Real element: '<real_link>."
                    "<inner_link_name>.<feature>.<subelement>' (e.g. "
                    "'real_l_chassis001_.chassis001.Box.Face3'); short forms "
                    "'chassis001.Box.Face3' / 'Box.Face3' accepted; empty = Real "
                    'origin. ONLY on explicit user request — the default '
                    'workflow uses plain references in set_placement_between.'
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
    'get_objects_info': {
        'properties': {
            'object_names': {
                'type': 'array',
                'items': {'type': 'string'},
                'description': 'List of object names or labels to describe in one call',
            },
            'max_internal_geometry_items': {
                'type': 'integer',
                'minimum': 100,
                'description': 'Max faces/vertices returned per object (default 1000, minimum 100)',
            },
        },
        'required': ['object_names'],
    },
    'check_overlap': {
        'properties': {
            'parent': {'type': 'string', 'description': 'Parent object/link name or Label'},
            'child': {'type': 'string', 'description': 'Child object/link name or Label'},
            'tolerance': {
                'type': 'number',
                'description': 'Touch tolerance in mm (default 1e-6); a shared face is not an overlap',
            },
        },
        'required': ['parent', 'child'],
    },
    'get_snapshot': {
        'properties': {
            'width': {'type': 'integer', 'description': 'Width (default 1024)'},
            'height': {'type': 'integer', 'description': 'Height (default 768)'},
            'format': {'type': 'string', 'enum': ['png', 'jpg', 'bmp'], 'description': 'Image format'},
            'object_name': {
                'type': 'string',
                'description': (
                    'Object/joint name or Label: select it and fit the view onto '
                    'it so its axes are shown up close (e.g. a sensor\'s joint to '
                    'check its frame). Empty = fit the whole scene.'
                ),
            },
        },
        'required': [],
    },
    'list_controller_types': {
        'properties': {},
        'required': [],
    },
    'create_controller': {
        'properties': {
            'robot': {'type': 'string', 'description': 'Cross::Robot name or Label'},
            'type': {
                'type': 'string',
                'description': (
                    'ROS controller name (e.g. "diff_drive_controller") or plugin '
                    'class full name; see list_controller_types()'
                ),
            },
            'name': {'type': 'string', 'description': 'Optional Label for the created object'},
            'kind': {
                'type': 'string',
                'enum': ['controller', 'broadcaster'],
                'description': "Kind to create (default 'controller')",
            },
        },
        'required': ['robot', 'type'],
    },
    'get_robot_controllers': {
        'properties': {
            'robot': {'type': 'string', 'description': 'Cross::Robot name or Label'},
        },
        'required': ['robot'],
    },
    'list_controller_parameters': {
        'properties': {
            'controller': {'type': 'string', 'description': 'Controller/broadcaster name or Label'},
        },
        'required': ['controller'],
    },
    'set_controller_parameter': {
        'properties': {
            'controller': {'type': 'string', 'description': 'Controller/broadcaster name or Label'},
            'parameter': {
                'type': 'string',
                'description': 'Full parameter name (nested names joined with "___")',
            },
            'value': {
                'description': (
                    'New value: scalar for plain parameters, object name(s) for '
                    'link/joint parameters'
                ),
            },
        },
        'required': ['controller', 'parameter', 'value'],
    },
    'set_controller_parameters': {
        'properties': {
            'controller': {'type': 'string', 'description': 'Controller/broadcaster name or Label'},
            'values': {
                'type': 'object',
                'description': 'Parameter full name -> value',
            },
        },
        'required': ['controller', 'values'],
    },
    'list_sensor_types': {
        'properties': {},
        'required': [],
    },
    'create_sensor': {
        'properties': {
            'element': {
                'type': 'string',
                'description': 'Parent Cross::Link, Cross::Joint or Cross::Robot name or Label',
            },
            'type': {
                'type': 'string',
                'description': (
                    'Sensor name (e.g. "camera", "gpu_lidar", "imu", '
                    '"force_torque"); see list_sensor_types()'
                ),
            },
            'name': {'type': 'string', 'description': 'Optional Label for the created object'},
        },
        'required': ['element', 'type'],
    },
    'set_placement_between_sensor_link_and_parent': {
        'properties': {
            'sensor': {
                'type': 'string',
                'description': (
                    'Link-attached forward-looking sensor (camera or vertical '
                    'lidar): the sensor object OR the sensor link that hosts '
                    'it (e.g. "l_camera001") — both are accepted. A HORIZONTAL '
                    'lidar does NOT use this tool — use set_placement_between '
                    '+ a Z rotation of its parent joint.'
                ),
            },
            'ref_parent': {
                'type': 'string',
                'description': (
                    'Reference on the parent link: '
                    '"<real_link>.<inner_link_name>.<feature>.<Face|Edge|VertexN>" or an LCS name'
                ),
            },
            'ref_sensor': {
                'type': 'string',
                'description': (
                    'Matching reference on the sensor link, same format'
                ),
            },
        },
        'required': ['sensor', 'ref_parent', 'ref_sensor'],
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
                    'controllers',
                    'sensors',
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
