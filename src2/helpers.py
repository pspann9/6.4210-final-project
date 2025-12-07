
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



def approach_pose(X_WG: RigidTransform) -> RigidTransform:
    """
    fill in our code below
    """
    X_GGapproach = RigidTransform(p=np.array([0.0, -0.2, 0.0]))
    X_WGApproach = X_WG @ X_GGapproach
    return X_WGApproach


def design_grasp_pose(X_WO: RigidTransform) -> tuple[RigidTransform, RigidTransform]:
    """
    fill in our code below
    """
    R_OG = RotationMatrix.MakeXRotation(-np.pi/2)

    p_WO = X_WO.translation()
    R_WO = X_WO.rotation()
    p_WG = p_WO + np.array([0.0, -0.005, 0.08])
    p_OG = R_WO.inverse() @ (p_WG - p_WO)

    X_OG = RigidTransform(R_OG, p_OG)
    X_WG = X_WO @ X_OG
    return X_OG, X_WG

