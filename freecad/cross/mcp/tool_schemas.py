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
            'object_labels': {
                'type': 'array',
                'items': {'type': 'string'},
                'description': (
                    'Labels of the objects (Part/Body/App::Link) to fill links '
                    'from, in link order (a Name is accepted as a fallback)'
                ),
            },
        },
        'required': ['robot', 'object_labels'],
    },
    'create_joint': {
        'properties': {
            'robot': {'type': 'string', 'description': 'Cross::Robot name or Label'},
            'name': {
                'type': 'string',
                'description': (
                    'Joint name (default "Joint"). Used ONLY when no links are '
                    'given (a joint without links). When parent_link and '
                    'child_link are given, the joint is named automatically.'
                ),
            },
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
            'link_labels_in_order': {
                'type': 'array',
                'items': {'type': 'string'},
                'description': (
                    'Link Labels in connection order (a Name is accepted as a '
                    'fallback); first is the spider hub'
                ),
            },
            'connect_type': {
                'type': 'string',
                'enum': ['chain', 'spider'],
                'description': 'chain = consecutive, spider = all to first',
            },
        },
        'required': ['robot', 'link_labels_in_order'],
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
                    'Link or joint to position. NOT on a camera-like sensor link '
                    '(use set_placement_between_sensor_link_and_parent). A link '
                    'with only a HORIZONTAL lidar IS positioned here, then its '
                    'parent joint is rotated about Z.'
                ),
            },
            'ref1': {
                'type': 'string',
                'description': (
                    "Reference on the PARENT link: a Real-element subelement "
                    "('real_l_chassis001_.chassis001.Box.Face3') or an LCS. A "
                    'robot link (l_...) cannot be a ref.'
                ),
            },
            'ref2': {
                'type': 'string',
                'description': (
                    'Reference on the CHILD link, same format. Joined to ref1 by '
                    'their centers.'
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
                    'Orienting a link: rotate its JOINT (local Z = functional '
                    'axis). Mirroring a link to the parent\'s other side: rotate '
                    'the LINK 180 about X (not Z).'
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
                    "Real-element subelement ('real_l_chassis001_.chassis001.Box."
                    "Face3'); short forms 'chassis001.Box.Face3' / 'Box.Face3' "
                    'accepted; empty = Real origin. ONLY on explicit request.'
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
            'robot_or_link_labels': {
                'type': 'array',
                'items': {'type': 'string'},
                'description': (
                    'Robot Label, or a list of link Labels (a Name is accepted '
                    'as a fallback)'
                ),
            },
        },
        'required': ['robot_or_link_labels'],
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
            'object_labels': {
                'type': 'array',
                'items': {'type': 'string'},
                'description': (
                    'List of object Labels to describe in one call (a Name is '
                    'accepted as a fallback when the Label is ambiguous)'
                ),
            },
            'max_internal_geometry_items': {
                'type': 'integer',
                'minimum': 100,
                'description': 'Max faces/vertices returned per object (default 1000, minimum 100)',
            },
        },
        'required': ['object_labels'],
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
    'measure_geometry': {
        'properties': {
            'ref1': {
                'type': 'string',
                'description': (
                    "REQUIRED full subelement path "
                    "('real_l_wheel001_.wheel001.Cylinder.Face2') or an LCS "
                    'name. A bare object name is rejected.'
                ),
            },
            'ref2': {
                'type': 'string',
                'description': (
                    'Second reference, same format. Omit it to measure the RADIUS '
                    'of a circular edge or a cylindrical/spherical/toroidal/'
                    'conical face (or a flat cylinder end).'
                ),
            },
            'mode': {
                'type': 'string',
                'enum': ['auto', 'distance', 'radius'],
                'description': (
                    "'auto' (default): distance when ref2 is given, radius "
                    "otherwise. 'distance' = min gap + center distance + per-axis "
                    "delta; 'radius' = radius/diameter/center/axis + 'circular'."
                ),
            },
        },
        'required': ['ref1'],
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
            'view': {
                'type': 'string',
                'enum': [
                    'isometric',
                    'axonometric',
                    'top',
                    'bottom',
                    'front',
                    'rear',
                    'left',
                    'right',
                ],
                'description': (
                    "Side the camera looks from (default 'isometric'). Call the "
                    'tool once per side to inspect a part from several '
                    'directions: top/bottom, front/rear (back), left/right. '
                    "Aliases: 'iso', 'axo', 'back', 'side', 'default'."
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
                    'Forward-looking link sensor (camera or vertical lidar): the '
                    'sensor object OR its sensor link. A HORIZONTAL lidar does not '
                    'use this tool.'
                ),
            },
            'ref_parent': {
                'type': 'string',
                'description': 'Reference on the parent link (Real-element subelement) or an LCS name',
            },
            'ref_sensor': {
                'type': 'string',
                'description': 'Matching reference on the sensor link, same format',
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
                'description': (
                    'Section to return (default "all"); "general_algorithm" is '
                    'the source of truth.'
                ),
            },
        },
        'required': [],
    },
}


def get_tool_schemas() -> dict[str, dict]:
    """Return the tool schemas dict."""
    return TOOL_SCHEMAS
