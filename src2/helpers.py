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
    InverseKinematics,
    AddFrameTriadIllustration,
    Trajectory,
    PiecewisePose,
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

from interpolation import interpolatePosesLinear


def approach_pose(X_WG: RigidTransform) -> RigidTransform:
    """
    fill in our code below
    """
    X_GGapproach = RigidTransform(p=np.array([0.0, -0.1, 0.0]))
    X_WGApproach = X_WG @ X_GGapproach
    return X_WGApproach


def design_grasp_pose(X_WO: RigidTransform) -> tuple[RigidTransform, RigidTransform]:
    """
    fill in our code below
    """
    R_OG = RotationMatrix.MakeXRotation(-np.pi/2)

    p_WO = X_WO.translation()
    R_WO = X_WO.rotation()
    p_WG = p_WO + np.array([0.0, 0.0, 0.05])
    p_OG = R_WO.inverse() @ (p_WG - p_WO)

    X_OG = RigidTransform(R_OG, p_OG)
    X_WG = X_WO @X_OG
    return X_OG, X_WG

def make_trajectory(
    X_Gs: list[RigidTransform], finger_values: np.ndarray, sample_times: list[float]
) -> tuple[Trajectory, PiecewisePolynomial]:
    robot_velocity_trajectory = None
    traj_wsg_command = None
    # TODO: define a PiecewisePose out of the X_Gs
    pose_traj = PiecewisePose.MakeLinear(sample_times, X_Gs)


    # TODO: set robot_velocity_trajectory to the derivative of the pose trajectory you just defined
    robot_velocity_trajectory: Trajectory = pose_traj.MakeDerivative()



    # TODO: set traj_wsg_command to a PiecewisePolynomial that commands the fingers
    knots = np.asarray(finger_values, dtype=float).reshape(1, -1)  # (1, L)
    traj_wsg_command = PiecewisePolynomial.ZeroOrderHold(sample_times, knots)

    return robot_velocity_trajectory, traj_wsg_command

def get_initial_pose(
    plant: MultibodyPlant,
    body_name: str,
    model_instance: ModelInstanceIndex,
    plant_context: Context,
) -> RigidTransform:
    body = plant.GetBodyByName(body_name, model_instance)
    X_WS = plant.EvalBodyPoseInWorld(plant_context, body)
    X_SO = RigidTransform(body.default_spatial_inertia().get_com())
    return X_WS @ X_SO



def plan_pickup(
    poses: list[RigidTransform],
    total_pickup_time: float,
    num_samples: int = 200,
):
    t_lst = np.linspace(0.0, total_pickup_time, num_samples)
    pose_lst = []

    num_segments = len(poses) - 1
    segment_duration = total_pickup_time / num_segments

    for t in t_lst:
        # which segment are we in?
        seg_idx = min(int(t // segment_duration), num_segments - 1)
        t_seg_start = seg_idx * segment_duration
        t_seg = t - t_seg_start
        alpha = t_seg / segment_duration  # in [0, 1]

        X_start = poses[seg_idx]
        X_end   = poses[seg_idx + 1]

        pose = interpolatePosesLinear(X_start, X_end, alpha)
        pose_lst.append(pose)

    # IK for each pose to get joint-space trajectory
    q_knots = create_q_knots(pose_lst)   # this should give you shape (N, 7) or list of 7-D vectors

    return t_lst, q_knots




def q_from_pose(
    plant: MultibodyPlant,
    context: Context,
    X_WG_target: RigidTransform,
    q_seed_full: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Solve IK to put the gripper at X_WG_target.

    Returns:
        q_iiwa: 7-dim iiwa joint vector
        q_full: full plant position vector (for re-use as next seed)
    """
    ik = InverseKinematics(plant, context)
    q_decision = ik.q()  # length = plant.num_positions()

    world = plant.world_frame()
    G = plant.GetFrameByName("body", plant.GetModelInstanceByName("wsg"))

    p_WG = X_WG_target.translation()
    eps = 1e-3

    ik.AddPositionConstraint(
        frameB=G,
        p_BQ=np.zeros(3),
        frameA=world,
        p_AQ_lower=p_WG - eps,
        p_AQ_upper=p_WG + eps,
    )

    prog = ik.prog()

    if q_seed_full is None:
        q_seed_full = plant.GetPositions(context)
    q_seed_full = np.asarray(q_seed_full).reshape(-1)
    assert q_seed_full.shape[0] == plant.num_positions()

    prog.SetInitialGuess(q_decision, q_seed_full)

    result = Solve(prog)
    if not result.is_success():
        raise RuntimeError(f"IK failed for pose:\n{X_WG_target}")

    q_full_sol = result.GetSolution(q_decision)
    q_iiwa_sol = iiwa_from_full(q_full_sol)
    return q_iiwa_sol, q_full_sol
