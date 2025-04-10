"""A robot represented with the Khalil-Kleinfinger (KK) model.

Notations from Khalil, W. & Kleinfinger, J., "A new geometric notation for
open and closed-loop robots", in Proceedings of the IEEE International
Conference on Robotics and Automation, 1986, pp. 1174 - 1179.

"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import accumulate
from itertools import pairwise
import math
from typing import Any, Optional
from typing import TYPE_CHECKING

import FreeCAD as fc

from .freecad_utils import is_same_placement
from .freecad_utils import warn
from .joint_proxy import make_joint
from .link_proxy import make_link
from .wb_utils import is_joint
from .wb_utils import is_link
from .wb_utils import ros_name

try:
    import numpy as np
    from . import geometry_helpers as gh
except ImportError as e:
    warn(
        f'numpy not available, some functionalities will not work, error: {e}',
        False,
    )

# Stubs and type hints.
if TYPE_CHECKING:
    try:
        from numpy.typing import ArrayLike
    except ImportError:
        ArrayLike = Any
    from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501


@dataclass
class KKFrame:
    """A frame of a robot.

    The parameters of the Khalil-Kleinfinger notation correspond to the order
    of the applied transforms and are:
    - pre_rz (gamma): angle between Xi and X'i about Zi.
    - pre_tz (epsilon): distance between Oi and O'i along Zi.
    - rx (alpha): angle between Zi and Zj about X'i.
    - tx (d): distance between O'i and Zj along X'i.
    - rz (theta): angle between X'i and Xj about Zj.
    - tz (r): distance between Oj and X'i along Zj.

    The Denavit-Hartenberg convention from
    [Wikipedia](https://en.wikipedia.org/wiki/Denavit–Hartenberg_parameters)
    can be converted to the KK notation as follows:
        KK  DH
        Θ   γ
        d   ε
        α   α
        r   d
        0   Θ
        0   r

    The modified DH parameters (also from Wikipedia) can be converted to the
    KK notation as follows:
        modified-DH  KK
        0            γ
        0            ε
        α            α
        r            d
        Θ            Θ
        d            r

    """
    # Notation:
    # The axis of joint (i) will be Zi.
    # The coordinate frame Ri (Oi,Xi,Yi,Zi) is fixed with respect to link (i).
    # The parameters which lead to define frame (i) will have (i) as subscript.

    # TODO: Use FreeCAD's quantities instead of floats.

    # ɑi: angle between Z(i-1) and Zi about X(i-1), in radians.
    rx: float

    # di: distance between O(i-1) and Zi, in meters.
    tx: float

    # Θi: angle between X(i-1) and Xi about Zi, in radians.
    rz: float

    # ri: distance between Oi and X(i-1), in meters.
    tz: float

    # The variable of joint (i) denoted by qi is Θ if (i) is rotational and
    # ri if (i) is prismatic. Hence qi = Θ * (1 - σi) + ri * σi, where σi is
    # the joint type (0 for rotational and 1 for prismatic).
    # We rather use a boolean to represent the joint type:
    # False for rotational, True for prismatic.
    prismatic: bool = False

    # In the case of a link with more than two joints, Xi is the common
    # perpendicular on Zi and Zj, where (i) is the antecedent of (j).
    # Among all possible directions for Xi, the one which is on the longest
    # branch is chosen. Other directions are denoted X'i, X''i, etc.
    # Thus, some other auxiliary frames R'(O',X',Y',Z') will be defined fixed
    # with respect to link(i).

    # γi: angle between Xi and X'i about Zi, in radians.
    # Default to 0.0, i.e. pure modified DH convention.
    pre_rz: float = 0.0

    # εi: distance between Oi and O'i, in meters.
    # Default to 0.0, i.e. pure modified DH convention.
    pre_tz: float = 0.0

    @property
    def sigma(self) -> int:
        """Joint type: 0 for rotational, 1 for prismatic."""
        return int(self.prismatic)

    @property
    def is_dh_compatible(self):
        """Check if the joint is compatible with the DH convention.

        Check if the joint is compatible with the Denavit-Hartenberg
        convention, i.e. not a tree structure.

        """
        return (self.pre_rz == 0.0) and (self.pre_tz == 0.0)

    def set_dh_from_placement(
        self,
        placement: fc.Placement,
    ) -> None:
        """Set the joint parameters from a placement.

        Set the joint parameters from the joint's Origin (in the
        RobotCAD workbench).
        Joints in RobotCAD are along/about the z-axis, no need for the `axis`
        parameter as in `set_dh_from_matrix`.

        """
        placement = placement.copy()
        placement.Base = placement.Base / 1000.0  # Convert to meters.
        matrix = placement.toMatrix()

        self.set_dh_from_matrix([
            [matrix.A11, matrix.A12, matrix.A13, matrix.A14],
            [matrix.A21, matrix.A22, matrix.A23, matrix.A24],
            [matrix.A31, matrix.A32, matrix.A33, matrix.A34],
            [matrix.A41, matrix.A42, matrix.A43, matrix.A44],
        ])

    def set_dh_from_matrix(
        self,
        matrix: ArrayLike,
        axis: Optional[ArrayLike] = None,
    ) -> None:
        """Set the joint parameters from a transformation matrix.

        Set the joint parameters from a transformation matrix between two
        joints.

        The transformation matrix must be a homogeneous transformation in the
        form:
            | cos(Θ)         -sin(Θ)       0        d         |
            | cos(ɑ)sin(Θ)   cos(ɑ)cos(Θ)  -sin(ɑ)  -r*sin(ɑ) |
            | sin(ɑ)sin(Θ)   sin(ɑ)cos(Θ)  cos(ɑ)   r*cos(ɑ)  |
            | 0              0             0        1         |
        Inspired from https://github.com/mcevoyandy/urdf_to_dh,
        urdf_to_dh.generate_dh.py (MIT License).

        Set only the Denavit-Hartenberg parameters, i.e. gamma and epsilon are
        set to 0.0.

        """
        raise NotImplementedError('Not implemented yet.')

        matrix = np.atleast_2d(matrix)

        # Default to the z-axis.
        if axis is None:
            z_axis = np.array([0.0, 0.0, 1.0, 0.0])
            axis = (matrix @ z_axis)[:3]

        # Khalil-Kleinfinger parameters.
        self.pre_rz = 0.0
        self.pre_tz = 0.0

        dh_params = np.zeros(4)

        origin_xyz = matrix[:3, 3]
        origin = np.zeros(3)
        z_axis = np.array([0.0, 0.0, 1.0])

        if gh.are_collinear(origin, z_axis, origin_xyz, axis):
            # Collinear case.
            dh_params[:] = self._dh_params_collinear_case(origin_xyz)
        elif gh.are_parallel(z_axis, axis):
            # Parallel case.
            dh_params[:] = self._dh_params_parallel_case(origin_xyz)
        elif gh.lines_intersect(origin, z_axis, origin_xyz, axis)[0]:
            # Intersect case.
            dh_params[:] = self._dh_params_intersection_case(origin_xyz, axis)
        else:
            # Skew case.
            dh_params[:] = self._dh_params_skew_case(origin_xyz, axis)

        self.rz, self.tz, self.tx, self.rx = dh_params

    def to_placement(self) -> fc.Placement:
        """Return the origin of the joint as Placement.

        Return the (URDF) origin of the joint as Placement, i.e. the
        placement of the joint frame with respect to the parent link frame.

        The transformation matrix of KK is
        ⎡-sγ*sθ*cα + cγ*cθ, -sγ*cα*cθ - sθ*cγ,  sα*sγ, d*cγ + r*sα*sγ⎤
        ⎢ sγ*cθ + sθ*cα*cγ, -sγ*sθ + cα*cγ*cθ, -sα*cγ, d*sγ - r*sα*cγ⎥
        ⎢            sα*sθ,             sα*cθ,     cα,       ε + r*cα⎥
        ⎣                0,                 0,      0,              1⎦

        The transformation matrix of modified-DH is T(i-1)(i) =
            | cos(Θ)         -sin(Θ)       0        d         |
            | cos(ɑ)sin(Θ)   cos(ɑ)cos(Θ)  -sin(ɑ)  -r*sin(ɑ) |
            | sin(ɑ)sin(Θ)   sin(ɑ)cos(Θ)  cos(ɑ)   r*cos(ɑ)  |
            | 0              0             0        1         |

        """
        cθ = math.cos(self.rz)
        sθ = math.sin(self.rz)
        cα = math.cos(self.rx)
        sα = math.sin(self.rx)
        cγ = math.cos(self.pre_rz)
        sγ = math.sin(self.pre_rz)
        r = self.tz
        d = self.tx
        ε = self.pre_tz
        placement = fc.Placement(
            fc.Matrix(
                  -sγ*sθ*cα + cγ*cθ, -sγ*cα*cθ - sθ*cγ,  sα*sγ, d*cγ + r*sα*sγ,
                   sγ*cθ + sθ*cα*cγ, -sγ*sθ + cα*cγ*cθ, -sα*cγ, d*sγ - r*sα*cγ,
                              sα*sθ,             sα*cθ,     cα,       ε + r*cα,
                                  0,                 0,      0,              1,
            ),
        )
        placement.Base *= 1000.0  # Convert to mm.
        return placement

    def to_placements(self) -> (fc.Placement, fc.Placement):
        """Return the two transforms for the joint origin.

        Return the (URDF) origin of the joint as Placement, i.e. the
        placement of the joint frame with respect to the parent link frame.

        The pre-transform matrix of KK is
        ⎡cγ, -sγ, 0, 0⎤
        ⎢sγ,  cγ, 0, 0⎥
        ⎢ 0,   0, 1, ε⎥
        ⎣ 0,   0, 0, 1⎦

        The transformation matrix of modified-DH is T(i-1)(i) =
            | cos(Θ)         -sin(Θ)       0        d         |
            | cos(ɑ)sin(Θ)   cos(ɑ)cos(Θ)  -sin(ɑ)  -r*sin(ɑ) |
            | sin(ɑ)sin(Θ)   sin(ɑ)cos(Θ)  cos(ɑ)   r*cos(ɑ)  |
            | 0              0             0        1         |

        """
        cθ = math.cos(self.rz)
        sθ = math.sin(self.rz)
        cα = math.cos(self.rx)
        sα = math.sin(self.rx)
        cγ = math.cos(self.pre_rz)
        sγ = math.sin(self.pre_rz)
        r = self.tz
        d = self.tx
        ε = self.pre_tz
        pre_placement = fc.Placement(
            fc.Matrix(
                cγ, -sγ, 0, 0,
                sγ,  cγ, 0, 0,
                 0,   0, 1, ε,
                 0,   0, 0, 1,
            ),
        )
        pre_placement.Base *= 1000.0  # Convert to mm.
        placement = fc.Placement(
            fc.Matrix(
                cθ,   -sθ,   0,     d,
             sθ*cα, cα*cθ, -sα, -r*sα,
             sα*sθ, sα*cθ,  cα,  r*cα,
                 0,     0,   0,     1,
            ),
        )
        placement.Base *= 1000.0  # Convert to mm.
        return pre_placement, placement

    def _dh_params_collinear_case(self, origin) -> list[float]:
        """Return (theta, r, d, ɑ) = (r, 0, 0, 0) for collinear joints.

        (theta, r, d, ɑ) is the KK notation, corresponding to the DH notation
        (d, Θ, r, ɑ).
        """
        dh_params: list[float] = [0.0] * 4
        dh_params[1] = origin[2]  # r.
        print(f'_dh_params_collinear_case, {dh_params}')  # DEBUG
        return dh_params

    def _dh_params_parallel_case(self, origin) -> list[float]:
        """Return (theta, r, d, ɑ) = (theta, r, d, 0) for parallel joints.

        (theta, r, d, ɑ) is the KK notation, corresponding to the DH notation
        (d, Θ, r, ɑ).

        """
        dh_params: list[float] = [0.0] * 4
        dh_params[1] = origin[2]  # r.
        dh_params[0] = math.atan2(origin[1], origin[0])  # θ.
        dh_params[2] = math.sqrt(origin[0]**2 + origin[1]**2)  # d.
        print(f'_dh_params_parallel_case, {dh_params}')  # DEBUG
        return dh_params

    def _dh_params_intersection_case(self, origin, axis) -> list[float]:
        """Return (theta, r, d, ɑ) = (theta, r, 0, ɑ) for intersecting joints.

        (theta, r, d, ɑ) is the KK notation, corresponding to the DH notation
        (d, Θ, r, ɑ).

        """
        zaxis = np.array([0.0, 0.0, 1.0])

        dh_params: list[float] = [0.0] * 4
        print(f'{gh.lines_intersect(np.zeros(3), zaxis, origin, axis,)=}')  # DEBUG
        dh_params[1] = gh.lines_intersect(
                np.zeros(3),
                zaxis,
                origin,
                axis,
        )[1][0]  # r.

        # Round small values to 0.0.
        axis[abs(axis) < 1.0e-5] = 0.0

        cn = np.cross(zaxis, axis)
        cn[abs(cn) < 1.0e-5] = 0.0
        if cn[0] < 0.0:
            cn = -cn

        dh_params[0] = math.atan2(cn[1], cn[0])  # θ.
        dh_params[2] = 0.0

        vn = cn / np.linalg.norm(cn)
        dh_params[3] = math.atan2(
            np.dot(np.cross(zaxis, axis), vn),
            np.dot(zaxis, axis),
        )  # ɑ.

        print(f'_dh_params_intersection_case, {dh_params}')  # DEBUG
        return dh_params

    def _dh_params_skew_case(self, origin, direction):
        """Return (theta, r, d, ɑ) for the general case.

        (theta, r, d, ɑ) is the KK notation, corresponding to the DH notation
        (d, Θ, r, ɑ).

        """
        # Late import to load the workbench faster and allow to use the rest
        # without numpy.
        import numpy as np

        point_a = np.zeros(3)
        point_b = np.zeros(3)
        dh_params: list[float] = [0.0] * 4

        # Find closest points along parent z-axis (point_a) and
        # joint axis (point_b)
        dir_2 = (direction[0]**2 + direction[1]**2)
        t = -(origin[0] * direction[0] + origin[1] * direction[1]) / dir_2
        point_b = origin + t * direction
        point_a[2] = point_b[2]

        dh_params[1] = point_a[2]  # r.

        dh_params[2] = np.linalg.norm(point_b - point_a)  # d.

        dh_params[0] = math.atan2(point_b[1], point_b[0])  # θ.

        # Awesome way to get signed angle:
        # https://stackoverflow.com/questions/5188561/signed-angle-between-two-3d-vectors-with-same-origin-within-the-same-plane/33920320#33920320
        cn = point_b - point_a
        vn = cn / np.linalg.norm(cn)
        zaxis = np.array([0.0, 0.0, 1.0])
        dh_params[3] = math.atan2(
            np.dot(np.cross(zaxis, direction), vn),
            np.dot(zaxis, direction),
        )  # ɑ.

        print(f'_dh_params_skew_case, {dh_params}')  # DEBUG
        return dh_params


@dataclass
class DHFrame:
    """A Denavit-Hartenberg frame of a robot.

    The parameters of the Denavit-Hartenberg notation correspond to the order
    of the applied transforms and are:
    - rz (theta): angle between X'i and Xj about Zj.
    - tz (r): distance between Oj and X'i along Zj.
    - rx (alpha): angle between Zi and Zj about X'i.
    - tx (d): distance between O'i and Zj along X'i.

    The Denavit-Hartenberg convention from
    [Wikipedia](https://en.wikipedia.org/wiki/Denavit–Hartenberg_parameters)
    can be converted to the KK notation as follows:
        KK  DH
        Θ   γ
        d   ε
        α   α
        r   d
        0   Θ
        0   r

    """
    # TODO: Use FreeCAD's quantities instead of floats.

    # Θi: angle between X(i-1) and Xi about Zi, in radians.
    rz: float

    # ri: distance between Oi and X(i-1), in meters.
    tz: float

    # ɑi: angle between Z(i-1) and Zi about X(i-1), in radians.
    rx: float

    # di: distance between O(i-1) and Zi, in meters.
    tx: float

    # The variable of joint (i) denoted by qi is Θ if (i) is rotational and
    # ri if (i) is prismatic. Hence qi = Θ * (1 - σi) + ri * σi, where σi is
    # the joint type (0 for rotational and 1 for prismatic).
    # We rather use a boolean to represent the joint type:
    # False for rotational, True for prismatic.
    prismatic: bool = False


@dataclass
class KKRobot:
    """A robot represented with the Khalil-Kleinfinger (KK) model.

    - The base will beconsidered as link 0.
    - The numbers of links and joints are increasing
      at each branch when traversing from the base to
      an end effector
    - link (i) is articulated on joint (i), i.e. joint (i)
      connects the link (a(i)) and link(i), where a(i) is
      the number of the link antecedent to link (i)
      when coming from the base
    - frame(i) is defined fixed with respect to link (i),
      and Z is the axis of joint(i).
    - Xi is the common perpendicular on Zi and Zj, where (i) is
      the antecedent of (j). Among all possible directions
      for Xi, the one which is on the longest branch is
      chosen. Other directions are denoted X'i, X''i, etc.
      Thus, some other auxiliary frames R'(O',X',Y',Z') will be defined fixed
      with respect to link(i).

    """
    kk_frames: list[KKFrame] = field(default_factory=list)

    @property
    def dof(self) -> int:
        """Degree of freedom, i.e. number of non-fixed joints."""
        return len(self.kk_frames)

    @property
    def is_dh_compatible(self):
        """Check if the robot is compatible with the DH convention.

        Check if the robot is compatible with the Denavit-Hartenberg
        convention, i.e. not a tree structure.

        Could have been called `is_open_chain`.

        """
        return all(j.is_dh_compatible for j in self.kk_frames)

    def set_from_robot(
        self,
        robot: CrossRobot,
    ) -> bool:
        """Set all parameters from a CROSS::Robot."""
        self.kk_frames.clear()

        chains = robot.Proxy.get_chains()
        if len(chains) > 1:
            return False

        for joint in robot.Proxy.get_joints():
            if joint.Type not in ['revolute', 'prismatic']:
                warn(f'Joint type {joint.Type} of {joint.Label} not supported', True)
                return False

        for joint in robot.Proxy.get_joints():
            prismatic = (joint.Type == 'prismatic')
            kk_joint = KKFrame(0.0, 0.0, 0.0, 0.0)  # Irrelevant values.
            kk_joint.set_dh_from_placement(joint.Origin)
            kk_joint.prismatic = prismatic
            self.kk_frames.append(kk_joint)
        return True

    def transfer_to_robot(
        self,
        robot: CrossRobot,
    ) -> bool:
        """Set all parameters to a CROSS::Robot.

        Does NOT open any transaction.

        """
        if not self.is_dh_compatible:
            warn(
                f'Robot "{robot.Label}" is not compatible with the'
                ' DH convention, not implemented yet',
                True,
            )
            return False
        cross_joints = robot.Proxy.get_joints()
        cross_links = robot.Proxy.get_links()

        if len(cross_joints) > len(self.kk_frames):
            warn(
                f'Robot "{robot.Label}" has a larger number of joints than'
                ' in the DH parameter table, not implemented yet',
                True,
            )
            return False

        if (
            cross_joints and cross_links
            and (len(cross_joints) != (len(cross_links) - 1))
        ):
            warn(
                f'Robot "{robot.Label}" has {len(cross_joints)} joints and'
                f' {len(cross_links)} links, should have'
                f' {len(cross_joints) + 1} links, not implemented yet',
                True,
            )
            return False

        if ((not cross_joints) and (len(cross_links) == 1)):
            warn(
                f'Robot "{robot.Label}" has 1 link and no'
                ' joint, not implemented yet',
                True,
            )
            return False

        self._add_missing_to_robot(robot)

        # We need to update the list of CROSS joints and links, this time
        # respecting the kinematic chain.
        # Implementation note: no need to check non-empty get_chains() because
        # of the check for DH compatibility above.
        chain = list(accumulate(robot.Proxy.get_chains(), initial=[]))[-1]
        cross_joints = [o for o in chain if is_joint(o)]
        cross_links = [o for o in chain if is_link(o)]
        link_pair_iterator = pairwise(cross_links)
        for i in range(len(self.kk_frames)):
            kk_joint = self.kk_frames[i]
            cross_joint = cross_joints[i]
            parent_link, child_link = next(link_pair_iterator)
            parent_name = ros_name(parent_link)
            if cross_joint.Parent != parent_name:
                # Implementation note: avoid recursion.
                cross_joint.Parent = parent_name
            child_name = ros_name(child_link)
            if cross_joint.Child != child_name:
                # Implementation note: avoid recursion.
                cross_joint.Child = ros_name(child_link)
            type_ = 'prismatic' if kk_joint.prismatic else 'revolute'
            if cross_joint.Type != type_:
                # Implementation note: avoid recursion.
                cross_joint.Type = type_
            if not is_same_placement(
                cross_joint.Origin, kk_joint.to_placement(),
                1e-5, 1e-6,
            ):
                # Implementation note: avoid recursion.
                cross_joint.Origin = kk_joint.to_placement()
            if cross_joint.Mimic:
                # Implementation note: avoid recursion.
                cross_joint.Mimic = False
        return True

    def _add_missing_to_robot(
        self,
        robot: CrossRobot,
    ) -> None:
        """Add the missing joints and links to the robot.

        Only DH parameters supported for now.

        No checks are done because they are in self.transfer_to_robot().

        """
        cross_joints = robot.Proxy.get_joints()
        chains = robot.Proxy.get_chains()

        doc = robot.Document
        if not cross_joints:
            base_link = make_link(f'{robot.Label}_base_link', doc)
            base_link.Label2 = 'base_link'
            base_link.adjustRelativeLinks(robot)
            robot.addObject(base_link)
            parent_link = base_link
        else:
            # Implementation note: no need to check for indexes.
            # Implementation note: no need to check that this is a link
            # because n(links) = n(joints) + 1.
            parent_link = chains[0][-1]

        for i in range(len(self.kk_frames) - len(cross_joints)):
            n = i + len(cross_joints)
            cross_joint = make_joint(f'{robot.Label}_joint_{n + 1}', doc)
            cross_joint.adjustRelativeLinks(robot)
            robot.addObject(cross_joint)
            # cross_joints.append(cross_joint)

            child_link = make_link(f'{robot.Label}_link_{n + 1}', doc)
            child_link.adjustRelativeLinks(robot)
            robot.addObject(child_link)

            cross_joint.Parent = ros_name(parent_link)
            cross_joint.Child = ros_name(child_link)


def kk_from_dh(
        dh_frames: list[DHFrame],
) -> list[KKFrame]:
    if not dh_frames:
        return []

    kk_frames: list[KKFrame] = []
    # First joint.
    kk_frames.append(
        KKFrame(
                rx=0.0,
                tx=0.0,
                rz=dh_frames[0].rz,
                tz=dh_frames[0].tz,
                prismatic=dh_frames[0].prismatic,
        ),
    )

    # Middle joints.
    for i in range(1, len(dh_frames)):
        kk_frames.append(
            KKFrame(
                        tx=dh_frames[i-1].tx,
                        rx=dh_frames[i-1].rx,
                        tz=dh_frames[i].tz,
                        rz=dh_frames[i].rz,
                        prismatic=dh_frames[i].prismatic,
            ),
        )

    # Last joint.
    kk_frames.append(
        KKFrame(
                tx=dh_frames[-1].tx,
                rx=dh_frames[-1].rx,
                tz=0.0,
                rz=0.0,
                prismatic=False,  # Fixed joint. Irrelevant value.
        ),
    )

    return kk_frames


def dh_from_kk(
        kk_frames: list[KKFrame],
) -> (list[DHFrame], fc.Placement):
    if not kk_frames:
        return []

    dh_frames: list[DHFrame] = []

    # Base transform.
    kk_frame = KKFrame(
                tx=kk_frames[0].tx,
                rx=kk_frames[0].rx,
                tz=0.0,
                rz=0.0,
                prismatic=True,  # Irrelevant value.
    )
    base = kk_frame.to_placement()

    # middle joints
    for i in range(len(kk_frames) - 1):
        dh_frames.append(
            DHFrame(
                    tx=dh_frames[i + 1].tx,
                    rx=dh_frames[i + 1].rx,
                    tz=dh_frames[i].tz,
                    rz=dh_frames[i].rz,
                    prismatic=dh_frames[i].prismatic,
            ),
        )

    # last joint
    dh_frames.append(
        DHFrame(
            rz=kk_frames[-1].rz,
            tz=kk_frames[-1].tz,
            rx=0.0,
            tx=0.0,
        ),
    )

    return dh_frames, base
