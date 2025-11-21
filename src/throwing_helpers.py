

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
    ge,
    le,
    Solve,
)

from manipulation import running_as_notebook
from manipulation.exercises.grader import Grader
from manipulation.exercises.pick.test_differential_ik import TestDifferentialIK
from manipulation.station import LoadScenario, MakeHardwareStation
from manipulation.utils import RenderDiagram
from pydrake.multibody.tree import JacobianWrtVariable
import numpy as np


def calculate_ball_release_pose(X_H, X_B, entry_angle, s_hit, T_bounds):
    prog = MathematicalProgram()
    v_rel = prog.NewContinuousVariables(3, "v_rel")
    T = prog.NewContinuousVariables(1, "T")[0]
    g = [0, 0, -9.81]
    
    # Get entry direction for a given angle
    p_hit = X_H.translation()
    R_WH = X_H.rotation()
    z_H = R_WH.col(2)
    
    down = -z_H / np.linalg.norm(z_H)
    
    world_up = np.array([0., 0., 1.])
    horizontal = np.cross(down, world_up)
    
    v_dir = np.cos(entry_angle) * down + np.sin(entry_angle) * horizontal
    v_dir = v_dir / np.linalg.norm(v_dir)
    
    v_hit = s_hit * v_dir
    
    # projectile constraints
    T_min, T_max = T_bounds
    prog.AddBoundingBoxConstraint(T_min, T_max, T)
    
    p0 = X_B.translation()
    
    for i in range(3):
        prog.AddConstraint(v_rel[i] + g[i] * T == v_hit[i])
        prog.AddConstraint(p0[i] + v_rel[i] * T + 0.5 * g[i] * T**2 == p_hit[i])
        
    # Solve for v_rel
    prog.AddQuadraticCost(v_rel @ v_rel)
    result = Solve(prog)
    
    p_rel_val = p0.copy()
    v_rel_val = result.GetSolution(v_rel)
    T_val = result.GetSolution(T)

    return p_rel_val, v_rel_val, T_val

def calculate_ball_release_pose2(X_H, X_B, T_bounds):
    prog = MathematicalProgram()
    v_rel = prog.NewContinuousVariables(3, "v_rel")
    T = prog.NewContinuousVariables(1, "T")[0]

    # positions
    p_hit = X_H.translation()      # hoop position
    p0 = X_B.translation()         # release position (gripper/ball)

    # gravity
    g = np.array([0., 0., -9.81])

    # time-of-flight bounds
    T_min, T_max = T_bounds
    prog.AddBoundingBoxConstraint(T_min, T_max, T)

    # ballistic position constraints: p(T) = p0 + v_rel*T + 0.5*g*T^2 = p_hit
    for i in range(3):
        prog.AddConstraint(p0[i] + v_rel[i] * T + 0.5 * g[i] * T**2 == p_hit[i])

    # choose the solution with smallest launch speed
    prog.AddQuadraticCost(v_rel @ v_rel)

    result = Solve(prog)

    v_rel_val = result.GetSolution(v_rel)
    T_val = result.GetSolution(T)
    p_rel_val = p0.copy()  # world release position

    return p_rel_val, v_rel_val, T_val

def calculate_ball_release_state(
    X_H,
    X_WG_align,
    X_GO,
    T_bounds,
    r_nom=0.8,      # nominal radial distance of release (m)
    z_nom=2.5,      # nominal height of release (m)
    w_pos=1.0       # weight on staying near nominal release position
):
    """
    Solve for ball release position, velocity, and time of flight.

    X_H        : world -> hoop RigidTransform
    X_WG_align : world -> gripper (aligned for throw; orientation we like)
    X_GO       : gripper -> object (ball) RigidTransform
    T_bounds   : (T_min, T_max)
    """

    g = np.array([0., 0., -9.81])

    prog = MathematicalProgram()
    p_rel = prog.NewContinuousVariables(3, "p_rel")   # ball position at release (world)
    v_rel = prog.NewContinuousVariables(3, "v_rel")   # ball velocity at release (world)
    T     = prog.NewContinuousVariables(1, "T")[0]    # time of flight

    p_hit = X_H.translation()
    T_min, T_max = T_bounds
    prog.AddBoundingBoxConstraint(T_min, T_max, T)

    # --- Ballistic position constraint: p_rel + v_rel*T + 0.5*g*T^2 = p_hit ---
    for i in range(3):
        prog.AddConstraint(p_rel[i] + v_rel[i] * T + 0.5 * g[i] * T**2 == p_hit[i])

    # --- Geometry: keep p_rel roughly on the ray from origin toward hoop in XY plane ---

    # Enforce that p_rel_xy is colinear with p_hit_xy:
    # p_hit_x * p_rel_y - p_hit_y * p_rel_x = 0
    # (this is linear since p_hit is constant)
    if np.linalg.norm(p_hit[:2]) > 1e-6:
        prog.AddConstraint(p_hit[0] * p_rel[1] - p_hit[1] * p_rel[0] == 0)

    # Nominal release point along that ray (used as a soft cost, not a hard constraint)
    if np.linalg.norm(p_hit[:2]) > 1e-6:
        dir_xy = p_hit[:2] / np.linalg.norm(p_hit[:2])
        p_rel_nom = np.array([r_nom * dir_xy[0], r_nom * dir_xy[1], z_nom])
    else:
        # if hoop is directly above origin, just pick some nominal
        p_rel_nom = np.array([r_nom, 0., z_nom])

    # --- Costs: prefer small launch speed and stay near nominal release position ---
    prog.AddQuadraticCost(v_rel @ v_rel)
    prog.AddQuadraticCost(w_pos * ((p_rel - p_rel_nom) @ (p_rel - p_rel_nom)))

    result = Solve(prog)
    if not result.is_success():
        raise RuntimeError("Ballistic program infeasible; no release found")

    p_rel_val = result.GetSolution(p_rel)
    v_rel_val = result.GetSolution(v_rel)
    T_val     = result.GetSolution(T)

    # Map ball release position to *gripper* pose at release using X_GO
    R_WG_align = X_WG_align.rotation()
    p_GO = X_GO.translation()
    # ball COM: p_rel = p_WG_release + R_WG_align @ p_GO  =>  p_WG_release = p_rel - R_WG_align @ p_GO
    p_WG_release = p_rel_val - R_WG_align @ p_GO
    X_WG_release = RigidTransform(R_WG_align, p_WG_release)

    return p_rel_val, v_rel_val, T_val, X_WG_release


def test_ballistics():
    g = np.array([0, 0, -9.81])
    X_H = RigidTransform(
        p=[0., 5., 3.048],
        R=RotationMatrix()  # identity
    )
    X_B = RigidTransform(
        p=[0., 0., 2.0],   # ball starts at height 2m
        R=RotationMatrix()
    )

    p_rel, v_rel, T = calculate_ball_release_pose2(X_H, X_B, [0.5, 4.0])
    
    p_hit = X_H.translation()

    p_T = p_rel + v_rel * T + 0.5 * g * (T**2)
    v_T = v_rel + g * T

    pos_error = p_T - p_hit
    return pos_error
    
if __name__ == "__main__":
    test_error = test_ballistics()
    print(test_error)
