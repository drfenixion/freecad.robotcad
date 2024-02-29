from __future__ import annotations

from typing import ForwardRef, Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    imports_ok = True
except ImportError:
    Node = ForwardRef('Node')
    MultiThreadedExecutor = ForwardRef('MultiThreadedExecutor')
    imports_ok = False


def get_node() -> Optional[Node]:
    if not imports_ok:
        return None

    node = Node('cross')
    return node


def get_node_and_executor() -> Tuple[Optional[MultiThreadedExecutor], Optional[Node]]:
    if not imports_ok:
        return None, None

    rclpy.init()
    node = get_node()
    if node is None:
        return None, None
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    return node, executor
