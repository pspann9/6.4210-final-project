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
    Sphere,
    AddFrameTriadIllustration,
    Multiplexer,
    InverseKinematics,
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

import scipy.optimize


from helpers import design_grasp_pose, approach_pose
from throw import pose_from_q, throw_objective
from pid import PDController
from perception_helpers import perceive_ball_and_bin

# Start the visualizer.
meshcat = StartMeshcat()

scenario_data = """
        directives:
        - add_model:
            name: iiwa
            file: package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf
            default_joint_positions:
                iiwa_joint_1: [-1.57]
                iiwa_joint_2: [0.1]
                iiwa_joint_3: [0]
                iiwa_joint_4: [-1.2]
                iiwa_joint_5: [0]
                iiwa_joint_6: [1.6]
                iiwa_joint_7: [0]

        - add_model:
            name: wsg
            file: package://manipulation/hydro/schunk_wsg_50_with_tip.sdf
            
        - add_weld:
            parent: world
            child:  iiwa::iiwa_link_0
            X_PC:
                translation: [0, 0, 0]
                rotation: !Rpy { deg: [0, 0, 0]}

        - add_weld:
            parent: iiwa::iiwa_link_7
            child: wsg::body
            X_PC:
                translation: [0, 0, 0.09]
                rotation: !Rpy { deg: [90, 0, 90]}


        - add_model:
            name: ball
            file: file:///workspaces/6.4210-final-project/sdfs/sphere_red.sdf
            default_free_body_pose:
                body_link:
                    translation: [0.55, 0, 0.0]
                    rotation: !Rpy { deg: [0, 0, 0] }

        - add_model:
            name: table
            file: file:///workspaces/6.4210-final-project/sdfs/table.sdf
        - add_weld:
            parent: world
            child: table::table_link
            X_PC:
                translation: [0.0, 0.0, -0.05]
                rotation: !Rpy { deg: [0, 0, 22.02] }
                
        - add_model:
            name: bin_red
            file: file:///workspaces/6.4210-final-project/sdfs/bin_red.sdf
            
        - add_weld:
            parent: world
            child: bin_red::bin_base
            X_PC:
                translation: [.5, -1.854106, 0]
                rotation: !Rpy { deg: [0, 0, 22.02] }

        - add_model:
            name: bin_green
            file: file:///workspaces/6.4210-final-project/sdfs/bin_green.sdf
            
        - add_weld:
            parent: world
            child: bin_green::bin_base
            X_PC:
                translation: [0, -2, 0]
                rotation: !Rpy { deg: [0, 0, 22.02] }

        - add_model:
            name: bin_blue
            file: file:///workspaces/6.4210-final-project/sdfs/bin_blue.sdf
            
        - add_weld:
            parent: world
            child: bin_blue::bin_base
            X_PC:
                translation: [1, -1.732050, 0]
                rotation: !Rpy { deg: [0, 0, 22.02] }
                
                
        - add_frame:
            name: camera0_origin
            X_PF:
                base_frame: world
                rotation: !Rpy { deg: [-140.0, 0.0, 180.0]}
                translation: [0, 4, 2] # [0, 0.8, 0.5]

        - add_model:
            name: camera0
            file: package://manipulation/camera_box.sdf

        - add_weld:
            parent: camera0_origin
            child: camera0::base

        - add_frame:
            name: camera1_origin
            X_PF:
                base_frame: world
                rotation: !Rpy { deg: [-140, 0.0, 90.0]}
                translation: [4, 0, 2] # [0.8, 0.1, 0.5]

        - add_model:
            name: camera1
            file: package://manipulation/camera_box.sdf

        - add_weld:
            parent: camera1_origin
            child: camera1::base

        - add_frame:
            name: camera2_origin
            X_PF:
                base_frame: world
                rotation: !Rpy { deg: [-140.0, 0.0, -90.0]}
                translation: [-4, 0, 2] # [-0.8, 0.1, 0.5]

        - add_model:
            name: camera2
            file: package://manipulation/camera_box.sdf

        - add_weld:
            parent: camera2_origin
            child: camera2::base

        - add_frame:
            name: camera3_origin
            X_PF:
                base_frame: world
                rotation: !Rpy { deg: [-140.0, 0.0, -180.0]}
                translation: [1, -4, 2] # [-0.8, 0.1, 0.5]

        - add_model:
            name: camera3
            file: package://manipulation/camera_box.sdf

        - add_weld:
            parent: camera3_origin
            child: camera3::base

        - add_frame:
            name: camera4_origin
            X_PF:
                base_frame: world
                rotation: !Rpy { deg: [-140.0, 0.0, -180.0]}
                translation: [-1, -4, 2] # [-0.8, 0.1, 0.5]

        - add_model:
            name: camera4
            file: package://manipulation/camera_box.sdf

        - add_weld:
            parent: camera4_origin
            child: camera4::base

        model_drivers:
            iiwa: !IiwaDriver
                hand_model_name: wsg
            wsg: !SchunkWsgDriver {}
            
        cameras:
            camera0:
                name: camera0
                depth: True
                rgb: True
                X_PB:
                    base_frame: camera0::base
            camera1:
                name: camera1
                depth: True
                rgb: True
                X_PB:
                    base_frame: camera1::base
            camera2:
                name: camera2
                depth: True
                rgb: True
                X_PB:
                    base_frame: camera2::base
            camera3:
                name: camera3
                depth: True
                rgb: True
                X_PB:
                    base_frame: camera3::base
            camera4:
                name: camera4
                depth: True
                rgb: True
                X_PB:
                    base_frame: camera4::base
    """

scenario = LoadScenario(data=scenario_data)

from pydrake.all import LeafSystem, BasicVector

pause_duration = 1.0

class TimeVaryingGains(LeafSystem):
    def __init__(self):
        super().__init__()
        self.DeclareVectorOutputPort("gains", BasicVector(2), self.Calc)

    def Calc(self, context, output):
        t = context.get_time()

        t_switch = 10.0 + 0.5 * pause_duration  # halfway through prethrow pause

        if t < t_switch:
            # pickup / move / early prethrow pause -> gentle
            kp = 300.0
            kd = 100.0
        else:
            # late prethrow pause + throw -> aggressive
            kp = 3000.0
            kd = 1.0

        output.SetFromVector([kp, kd])

# Define the builder we will use to specify the full diagram.
# Add the hardware station to the diagram
builder = DiagramBuilder()
station = MakeHardwareStation(scenario, meshcat=meshcat, hardware=False)
builder.AddSystem(station)
plant = station.GetSubsystemByName("plant")


# get initial poses of gripper and objects
temp_context = station.CreateDefaultContext()
temp_plant_context = plant.GetMyContextFromRoot(temp_context)
X_WGinitial = plant.EvalBodyPoseInWorld(temp_plant_context, plant.GetBodyByName("body"))
model_instance_ball = plant.GetModelInstanceByName("ball")
# X_WOball_initial = get_initial_pose(
#     plant, "body_link", model_instance_ball, temp_plant_context
# )
X_WOball_initial, X_WB_bin = perceive_ball_and_bin(scenario, meshcat)

# print("COLOR", ball_color, bin_color)

# Build trajectory keyframes
X_OG, X_WG_pick = design_grasp_pose(X_WOball_initial)
X_WG_prepick = approach_pose(X_WG_pick)

# === New: compute hoop pose (world → hoop) ===
# model_instance_hoop = plant.GetModelInstanceByName("hoop_model")
# hoop_body = plant.GetBodyByName("base_link_hoop", model_instance_hoop)
# X_WH = plant.EvalBodyPoseInWorld(temp_plant_context, hoop_body)


# Ball-in-gripper transform from design_grasp_pose
X_GO = X_OG.inverse()

X_WG_hold = X_WGinitial
X_WO_hold = X_WG_hold @ X_GO
p_WB = X_WB_bin.translation()


heading = np.arctan2(p_WB[1], p_WB[0])  # angle from base to hoop in world XY
q0 = heading + np.pi

PRETHROW_ANGLES = np.array([
    q0,        # base heading
    0.0,       # shoulder pan
    0.0,       # shoulder lift
    2.1,       # big bend
    0.0,       # wrist 1
    -1.95,      # big opposite bend
    0.0,       # wrist 2
])
THROWEND_ANGLES = np.array([
    q0,
    0.0,
    0.0,
    0.6,       # extend
    0.0,
    -0.3,      # extend
    0.0,
])

MIDTHROW_PRE_ANGLES = np.array([
    q0,        # base heading
    0.0,       # shoulder pan
    0.0,       # shoulder lift
    2.0,       # big bend
    0.0,       # wrist 1
    -0.2,      # big opposite bend
    0.0,       # wrist 2
])

MIDTHROW_ANGLES = np.array([
    q0,        # base heading
    0.0,       # shoulder pan
    0.0,       # shoulder lift
    1.9,       # big bend
    0.0,       # wrist 1
    -0.3,      # big opposite bend
    0.0,       # wrist 2
])


iiwa = plant.GetModelInstanceByName("iiwa")
iiwa_start = plant.GetJointByName("iiwa_joint_1").position_start()
iiwa_nq = plant.num_positions(iiwa)
wsg  = plant.GetModelInstanceByName("wsg")
gripper_body = plant.GetBodyByName("body", wsg)  # <- this is your EE


def iiwa_from_full(q_full: np.ndarray) -> np.ndarray:
    """Extract the 7 iiwa positions from the full q."""
    return q_full[iiwa_start : iiwa_start + iiwa_nq]

def replace_iiwa_in_full(q_full: np.ndarray, q_iiwa: np.ndarray) -> np.ndarray:
    """Return a copy of q_full with its iiwa segment replaced."""
    q_full = np.asarray(q_full).copy()
    q_full[iiwa_start : iiwa_start + iiwa_nq] = q_iiwa
    return q_full

def q_from_pose(plant: MultibodyPlant,
                context: Context,
                X_WG_target: RigidTransform,
                q_seed: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    IK: find joint angles that put the *gripper body* at X_WG_target
    with both position and orientation constrained.

    Returns:
        (q_iiwa, q_full)
    """
    ik = InverseKinematics(plant, context)
    q_decision = ik.q()

    world = plant.world_frame()
    wsg_instance = plant.GetModelInstanceByName("wsg")
    G = plant.GetFrameByName("body", wsg_instance)

    p_WG = X_WG_target.translation()
    R_WG = X_WG_target.rotation()

    pos_eps = 1e-3

    # Position: keep gripper origin near target
    ik.AddPositionConstraint(
        frameB=G,
        p_BQ=np.zeros(3),
        frameA=world,
        p_AQ_lower=p_WG - pos_eps,
        p_AQ_upper=p_WG + pos_eps,
    )

    # Orientation: align G with R_WG (small angle tolerance)
    ik.AddOrientationConstraint(
        frameAbar=world,
        R_AbarA=R_WG,
        frameBbar=G,
        R_BbarB=RotationMatrix.Identity(),
        theta_bound=0.01  # ~0.5 degrees, can relax if IK fails
    )

    prog = ik.prog()
    prog.SetInitialGuess(q_decision, q_seed)

    result = Solve(prog)
    if not result.is_success():
        raise RuntimeError(f"IK failed for pose:\n{X_WG_target}")

    q_full = result.GetSolution(q_decision)

    iiwa = plant.GetModelInstanceByName("iiwa")
    nq_iiwa = plant.num_positions(iiwa)
    q_iiwa = q_full[:nq_iiwa]

    return q_iiwa, q_full

X_WG_prethrow = pose_from_q(plant, PRETHROW_ANGLES, temp_plant_context)
X_WG_release  = pose_from_q(plant, THROWEND_ANGLES, temp_plant_context)

    
R_WG = X_WG_hold.rotation()
p_hold = X_WG_hold.translation()
p_WB = X_WB_bin.translation()

res = scipy.optimize.differential_evolution(
    lambda inp: throw_objective(
        inp,
        plant=plant,
        plant_context=temp_plant_context,
        p_WB=p_WB,
        PRETHROW_ANGLES=PRETHROW_ANGLES,
        THROWEND_ANGLES=THROWEND_ANGLES,
    ),
    bounds=[(1e-3, 3.0), (0.1, 0.9)],  # time, release_frac
    seed=43,
)

throw_motion_time, release_frac = res.x
print("opt throw_time", throw_motion_time, "release_frac", release_frac)
    
# Ignore the tiny optimized time; use a slower throw the PD can track
# You can tune this, but 0.4–0.6 is a reasonable starting point.
# throw_duration = 0.2
tune_throw_param = -.05
throw_duration = throw_motion_time + tune_throw_param
# or, if you want to keep a lower bound:
# throw_duration = max(0.4, float(throw_motion_time))


# Fixed times for the overall sequence
t_prethrow_arrive    = 10.0
t_prethrow_pause_end = t_prethrow_arrive + pause_duration
t_throw_start        = t_prethrow_pause_end
t_throw_end          = t_throw_start + throw_duration


q_seed_full = plant.GetPositions(temp_plant_context)

# iiwa q at initial config
q_initial = iiwa_from_full(q_seed_full)

# IK for the pickup / hold poses:
q_prepick, q_seed_full = q_from_pose(plant, temp_plant_context, X_WG_prepick, q_seed_full)
q_pick,    q_seed_full = q_from_pose(plant, temp_plant_context, X_WG_pick,    q_seed_full)
q_hold = q_initial
q_prethrow = PRETHROW_ANGLES
q_midthrow_pre = MIDTHROW_PRE_ANGLES
q_midthrow = MIDTHROW_ANGLES
q_release  = THROWEND_ANGLES

# build iiwa joint trajectory using the timing we defined above
times = [
    0.0,               # initial
    2.0,               # move above ball
    5.0,               # move down to grasp
    6.0,               # move above ball
    8.0,               # lift / hold
    t_prethrow_arrive, # move to prethrow
    t_throw_start,     # pause at prethrow
    (t_throw_start + (t_throw_start + t_throw_end) / 2) / 2,
    (t_throw_start + t_throw_end) / 2,     # pause at prethrow
    t_throw_end,       # finish throw (reach release)
]

q_knots = np.vstack([
    q_initial,   # 0.0
    q_prepick,   # 2.0
    q_pick,      # 5.0
    q_prepick,      # 6.0
    q_hold,      # 8.0
    q_prethrow,  # t_prethrow_arrive
    q_prethrow,  # t_throw_start (hold)
    q_midthrow_pre,
    q_midthrow,  # t_throw_start (hold)
    q_release,   # t_throw_end
]).T

traj_q_des = PiecewisePolynomial.CubicShapePreserving(times, q_knots)
q_des_source = builder.AddSystem(TrajectorySource(traj_q_des))


opened = 0.107
closed = 0.0

# Decide when (as a fraction of the *slower* throw) you want to release.
# 0.7–0.85 is usually “late in the swing”.
# release_frac_for_wsg = 1.2
release_frac_for_wsg = release_frac # 1.2  # try 0.8 first, then tweak
tune_param = 0.16

t_release_wsg = t_throw_start + release_frac_for_wsg * throw_duration + tune_param


times_wsg = [
    0.0,
    2.0,
    5.0,
    8.0,
    t_prethrow_arrive,  # closed while moving to prethrow
    t_release_wsg,      # open during the throw
    12.0,
]

print("=== timing debug ===")
print("throw_duration:", throw_duration)
print("release_frac:", release_frac)
print("t_throw_start:", t_throw_start)
print("t_throw_end:", t_throw_end)
print("t_release_wsg:", t_release_wsg)
print("times_wsg:", times_wsg)


# 1 x N array of finger positions
finger_knots = np.array([[ 
    opened,   # 0.0
    opened,   # 2.0
    closed,   # 5.0 (grab ball)
    closed,   # 8.0
    closed,   # at prethrow
    opened,   # release
    opened,   # stay open
]])

traj_wsg_command = PiecewisePolynomial.ZeroOrderHold(times_wsg, finger_knots)
wsg_source = builder.AddSystem(TrajectorySource(traj_wsg_command))
builder.Connect(
    wsg_source.get_output_port(),
    station.GetInputPort("wsg.position"),
)




# Make the PD (now with time-varying gains)
pd = builder.AddSystem(PDController())

# Time-varying gains → [kp, kd]
gain_sched = builder.AddSystem(TimeVaryingGains())
builder.Connect(gain_sched.get_output_port(0), pd.gains_port)


# Measured [q; qdot]
mux = builder.AddSystem(Multiplexer([7, 7]))
builder.Connect(
    station.GetOutputPort("iiwa.position_measured"),
    mux.get_input_port(0)
)

builder.Connect(
    station.GetOutputPort("iiwa.velocity_estimated"),
    mux.get_input_port(1)
)

builder.Connect(mux.get_output_port(), pd.state_port)
builder.Connect(q_des_source.get_output_port(), pd.q_des_port)
builder.Connect(pd.output_port, station.GetInputPort("iiwa.torque"))

builder.Connect(
    station.GetOutputPort("iiwa.position_measured"),
    station.GetInputPort("iiwa.position"),
)

# visualize axes (useful for debugging)
scenegraph = station.GetSubsystemByName("scene_graph")
AddFrameTriadIllustration(
    scene_graph=scenegraph,
    body=plant.GetBodyByName("body_link", model_instance_ball),
    length=0.1,
)

AddFrameTriadIllustration(
    scene_graph=scenegraph, body=plant.GetBodyByName("body"), length=0.1
)

diagram = builder.Build()

T_final = 13.0

# Define the simulator.
simulator = Simulator(diagram)
context = simulator.get_mutable_context()
station_context = station.GetMyContextFromRoot(context)
diagram.ForcedPublish(context)


# run simulation!
meshcat.StartRecording()
if running_as_notebook:
    simulator.set_target_realtime_rate(1.0)
simulator.AdvanceTo(T_final)

meshcat.StopRecording()
meshcat.PublishRecording()

