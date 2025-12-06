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
    InverseKinematics,
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

def pose_from_q(plant, q, context):
    iiwa = plant.GetModelInstanceByName("iiwa")
    wsg  = plant.GetModelInstanceByName("wsg")
    gripper_body = plant.GetBodyByName("body", wsg)
    plant.SetPositions(context, iiwa, q)
    return plant.EvalBodyPoseInWorld(context, gripper_body)

def V_spatial_from_q(plant, context, q, d_G_Ocom=0.11):
    iiwa = plant.GetModelInstanceByName("iiwa")
    wsg  = plant.GetModelInstanceByName("wsg")
    G    = plant.GetBodyByName("body", wsg).body_frame()
    W    = plant.world_frame()

    plant.SetPositions(context, iiwa, q)

    # point at which we want the velocity (ball COM, expressed in G)
    p_GQ = np.array([0, d_G_Ocom, 0])

    J_full = plant.CalcJacobianSpatialVelocity(
        context,
        JacobianWrtVariable.kQDot,
        G,
        p_GQ,
        W,
        W,
    )
    return J_full[:, :7]


def throw_objective(inp, plant, plant_context, p_WB, PRETHROW_ANGLES, THROWEND_ANGLES, g=9.81, d_G_Ocom=0.11, return_other=None):
    throw_motion_time, release_frac = inp

    release_ja = PRETHROW_ANGLES + release_frac * (THROWEND_ANGLES - PRETHROW_ANGLES)

    T_world_releasePose = pose_from_q(plant, release_ja, plant_context)
    p_release = (
        T_world_releasePose.translation()
      + T_world_releasePose.rotation().multiply([0, d_G_Ocom, 0])
    )

    J_release = V_spatial_from_q(
        plant=plant,
        context=plant_context,
        q=release_ja[:7],
        d_G_Ocom=d_G_Ocom,
    )[3:6]
    v_release = J_release @ ((THROWEND_ANGLES - PRETHROW_ANGLES) / throw_motion_time)[:7]

    # ensure we're throwing in the general direction of the target
    if v_release[:2] @ (p_WB - p_release)[:2] <= 0:
        return 1000

    x = np.linalg.norm((p_WB - p_release)[:2])
    y = (p_WB - p_release)[2]
    vx = np.linalg.norm(v_release[:2])
    vy = v_release[2]

    tta = x / vx
    y_hat = vy * tta - 0.5 * g * tta ** 2
    phi_hat = np.arctan((vy - g * tta) / vx)

    objective = (y_hat - y) ** 2

    # you can add the approach-angle term if you care:
    # objective += np.maximum(phi_hat - MAX_APPROACH_ANGLE, 0) ** 2

    if objective < 1e-6:
        objective -= throw_motion_time ** 2  # prefer slower throw if perfect


    return objective

def plan_throw_ik_trajectory(plant, pose_list, t_list):
    iiwa = plant.GetModelInstanceByName("iiwa")
    wsg  = plant.GetModelInstanceByName("wsg")
    G    = plant.GetBodyByName("body", wsg).body_frame()
    W    = plant.world_frame()

    # One shared context we keep updating
    context = plant.CreateDefaultContext()

    # Start from the plant's default iiwa positions
    q_prev_iiwa = plant.GetPositions(context, iiwa).copy()

    q_knots = []
    pos_tol   = 0.005
    theta_tol = 5.0 * np.pi / 180.0

    for X_WG in pose_list:
        p_des = X_WG.translation()
        R_des = X_WG.rotation()

        # Warm-start the context's iiwa positions from previous solution
        plant.SetPositions(context, iiwa, q_prev_iiwa)

        ik = InverseKinematics(plant, context)
        q_dec = ik.q()

        # Position constraint on gripper
        ik.AddPositionConstraint(
            frameB=G,
            p_BQ=np.zeros(3),
            frameA=W,
            p_AQ_lower=p_des - pos_tol,
            p_AQ_upper=p_des + pos_tol,
        )

        # Orientation constraint on gripper
        ik.AddOrientationConstraint(
            frameAbar=W,
            R_AbarA=R_des,
            frameBbar=G,
            R_BbarB=RotationMatrix(),
            theta_bound=theta_tol,
        )

        prog = ik.prog()

        # IMPORTANT: initial guess must have the same size as q_dec
        q_full_init = plant.GetPositions(context)   # all positions, correct length
        prog.SetInitialGuess(q_dec, q_full_init)

        result = Solve(prog)

        if not result.is_success():
            print("Warning: throw IK failed at a sample; reusing previous iiwa q.")
            q_sol_iiwa = q_prev_iiwa
        else:
            # Get full solution, then extract iiwa joints from it
            q_sol_full = result.GetSolution(q_dec)
            plant.SetPositions(context, q_sol_full)
            q_sol_iiwa = plant.GetPositions(context, iiwa)

        q_knots.append(q_sol_iiwa)
        q_prev_iiwa = q_sol_iiwa

    q_knots = np.asarray(q_knots)  # (N, 7)
    return np.asarray(t_list), q_knots

    
    