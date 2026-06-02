from dataclasses import dataclass, field
from typing import Any, List, Optional, Tuple, Dict
import xml.etree.ElementTree as ET
import numpy as np 

# Data Structure Definitions.
@dataclass
class URDFElement:
    """Base class for any URDF component"""
    name: str
    origin_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)

@dataclass
class Link(URDFElement):
    """Represents a physical link with inertia and visual properties"""
    name: str
    mass: float = 0.0
    inertia: Dict[str, float] = field(default_factory=dict)
    collision_mesh_path: str = ""
    visual_mesh_path: str = ""
    center_of_mass: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    material: Dict[str, Any] = field(default_factory=dict)

@dataclass
class Joint(URDFElement):
    """Represents a joint connecting two links."""
    joint_type: str = "fixed"  # 'revolute', 'prismatic', 'continuous', 'fixed'
    parent: str = ""
    child: str = ""
    axis: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    limit_lower: Optional[float] = None
    limit_upper: Optional[float] = None
    effort_limit: Optional[float] = None
    velocity_limit: Optional[float] = None

@dataclass
class Robot:
    name: str
    links: List[Link] = field(default_factory=list)
    joints: List[Joint] = field(default_factory=list)

