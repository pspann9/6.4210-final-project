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