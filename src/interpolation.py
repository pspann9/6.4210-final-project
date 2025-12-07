from pydrake.all import ModelVisualizer, Simulator, StartMeshcat
from manipulation import ConfigureParser, running_as_notebook
from manipulation.station import LoadScenario, MakeHardwareStation

from typing import Callable

import numpy as np
from pydrake.all import (
    BasicVector,
    Box,
    ConstantVectorSource,
    Context,
    DiagramBuilder,
    Integrator,
    JacobianWrtVariable,
    LeafSystem,
    MathematicalProgram,
    MultibodyPlant,
    Rgba,
    RigidTransform,
    RotationMatrix,
    SnoptSolver,
    PiecewisePolynomial,
    ModelInstanceIndex,
    TrajectorySource,
    AddFrameTriadIllustration,
    InverseKinematics,
    Trajectory,
    PiecewisePose,
    PiecewiseQuaternionSlerp,
    Meshcat,
    ge,
    le,
    Solve,
    Quaternion
)

from manipulation import running_as_notebook
from manipulation.exercises.grader import Grader
from manipulation.exercises.pick.test_differential_ik import TestDifferentialIK
from manipulation.station import LoadScenario, MakeHardwareStation
from manipulation.utils import RenderDiagram


def orientation_slerp(r_A, r_B):
    """
    r_A, r_B: RotationMatrix
    Returns a PiecewiseQuaternionSlerp over [0, 1].
    """
    # Convert RotationMatrix -> Quaternion using the 3x3 rotation matrix
    q_A = Quaternion(r_A.matrix())
    q_B = Quaternion(r_B.matrix())

    traj = PiecewiseQuaternionSlerp()
    traj.Append(0.0, q_A)
    traj.Append(1.0, q_B)
    return traj


def interpolatePosesLinear(T_world_poseA, T_world_poseB, t):
    # assume t is between 0 and 1; responsibility of caller to scale time
    p_A, r_A = T_world_poseA.translation(), T_world_poseA.rotation()
    p_B, r_B = T_world_poseB.translation(), T_world_poseB.rotation()

    # rotation is a clean slerp
    r_slerp = orientation_slerp(r_A, r_B)

    # In this Drake build, value(t) returns a 4x1 array of [w, x, y, z],
    # not a Quaternion object. Wrap it explicitly.
    q_vec = np.asarray(r_slerp.value(t)).reshape(4,)   # shape (4,)
    q_curr = Quaternion(q_vec)                         # make a Quaternion
    r_curr = RotationMatrix(q_curr)                    # RotationMatrix from Quaternion

    # position is a straight line
    p_curr = p_A + t * (p_B - p_A)

    T_world_currGripper = RigidTransform(r_curr, p_curr)
    return T_world_currGripper


def interpolatePosesArcMotion(T_world_poseA, T_world_poseB, t):
    # assume t is between 0 and 1; responsibility of caller to scale time
    p_A, r_A = T_world_poseA.translation(), T_world_poseA.rotation()
    p_B, r_B = T_world_poseB.translation(), T_world_poseB.rotation()

    # rotation is a clean slerp
    r_slerp = orientation_slerp(r_A, r_B)
    r_curr = RotationMatrix(r_slerp.value(t))

    # position is an arc that isn't necessarily axis aligned
    arc_radius = p_B[2] - p_A[2]
    phi = np.arctan2(p_B[1] - p_A[1], p_B[0] - p_A[0]) #xy direction heading
    theta = (t - 1) * np.pi / 2 # -90 to 0 degrees
    p_curr = p_A + np.array([
        arc_radius * np.cos(theta) * np.cos(phi),
        arc_radius * np.cos(theta) * np.sin(phi),
        arc_radius * np.sin(theta) + arc_radius
    ])

    T_world_currGripper = RigidTransform(r_curr, p_curr)
    return T_world_currGripper

def interpolatePosesArcMotion_pdot(T_world_poseA, T_world_poseB, t):
    # assume t is between 0 and 1; responsibility of caller to scale time
    p_A, r_A = T_world_poseA.translation(), T_world_poseA.rotation()
    p_B, r_B = T_world_poseB.translation(), T_world_poseB.rotation()

    # position is an arc that isn't necessarily axis aligned
    arc_radius = p_B[2] - p_A[2]
    phi = np.arctan2(p_B[1] - p_A[1], p_B[0] - p_A[0]) #xy direction heading
    theta = (t - 1) * np.pi / 2 # -90 to 0 degrees
    pdot_curr = (np.pi / 2) * np.array([
      - arc_radius * np.sin(theta) * np.cos(phi),
      - arc_radius * np.sin(theta) * np.sin(phi),
        arc_radius * np.cos(theta)
    ])

    return pdot_curr


def interpolate_joint_angle(ja1, ja2, time_interval, num_samples, include_end=False):
    # constant joint velocity over the time interval
    # naive - let's try not to deal with wrap around and joint velocity limits
    delta = ja2 - ja1
    joint_velocities = delta / time_interval

    ja_list = []
    t_lst = np.linspace(0, time_interval, num_samples, endpoint=include_end)
    for t in t_lst:
        ja_list.append(ja1 + t * joint_velocities)

    return t_lst, ja_list


def interpolate_keyframe_poses(
    poses: list[RigidTransform],
    times: list[float],
    num_samples: int,
) -> tuple[np.ndarray, list[RigidTransform]]:
    """
    Given keyframe gripper poses X_WG[i] at times times[i],
    produce a dense list of interpolated poses and times.

    - poses: list of RigidTransform, length K
    - times: list of floats, same length K, strictly increasing
    - num_samples: total number of time samples in [times[0], times[-1]]

    Returns:
        t_samples: (num_samples,) array of times
        pose_samples: list of RigidTransform, length num_samples
    """
    assert len(poses) == len(times), "poses and times must be same length"
    assert len(poses) >= 2, "need at least two keyframes"
    times = np.asarray(times, dtype=float)
    assert np.all(np.diff(times) > 0), "times must be strictly increasing"

    t_start = times[0]
    t_end   = times[-1]

    # Uniform sampling along the whole motion
    t_samples = np.linspace(t_start, t_end, num_samples)

    pose_samples: list[RigidTransform] = []

    for t in t_samples:
        # Find which segment [times[i], times[i+1]] we are in
        # searchsorted returns index where t would be inserted to keep order
        idx = np.searchsorted(times, t) - 1
        if idx < 0:
            idx = 0
        if idx >= len(poses) - 1:
            idx = len(poses) - 2

        t0 = times[idx]
        t1 = times[idx + 1]

        # Normalize local parameter alpha ∈ [0,1]
        if t1 == t0:
            alpha = 0.0
        else:
            alpha = (t - t0) / (t1 - t0)

        X0 = poses[idx]
        X1 = poses[idx + 1]

        # Use your existing SE(3) linear interpolation
        X_interp = interpolatePosesLinear(X0, X1, alpha)
        pose_samples.append(X_interp)

    return t_samples, pose_samples