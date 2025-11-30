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

from helpers import get_initial_pose, design_grasp_pose, approach_pose, make_trajectory
from ik import PseudoInverseController
from throw import pose_from_q
# from interpolation import make_trajectory

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

        - add_weld:
            parent: iiwa::iiwa_link_7
            child: wsg::body
            X_PC:
                translation: [0, 0, 0.09]
                rotation: !Rpy { deg: [90, 0, 90]}

        - add_model:
            name: hoop_model
            file: file:///workspaces/6.4210-final-project/sdfs/basketball_hoop.sdf

        - add_weld:
            parent: world
            child: hoop_model::base_link_hoop  
            X_PC:
                translation: [0, -5, 3.048]
                rotation: !Rpy { deg: [0, 0, 90] }

        - add_model:
            name: ball
            file: package://drake_models/manipulation_station/sphere.sdf
            default_free_body_pose:
                base_link:
                    translation: [0.55, 0, 0]
                    rotation: !Rpy { deg: [0, 0, 0] }

        - add_model:
            name: table
            file: file:///workspaces/6.4210-final-project/sdfs/table.sdf
        - add_weld:
            parent: world
            child: table::table_link
            X_PC:
                translation: [1.45, 0.0, -0.05]
                rotation: !Rpy { deg: [0, 0, -90] }

        model_drivers:
            iiwa: !IiwaDriver
                hand_model_name: wsg
            wsg: !SchunkWsgDriver {}
    """

scenario = LoadScenario(data=scenario_data)



# Define the builder we will use to specify the full diagram.
# Add the hardware station to the diagram
builder = DiagramBuilder()
station = MakeHardwareStation(scenario, meshcat=meshcat)
builder.AddSystem(station)
plant = station.GetSubsystemByName("plant")

# get initial poses of gripper and objects
temp_context = station.CreateDefaultContext()
temp_plant_context = plant.GetMyContextFromRoot(temp_context)
X_WGinitial = plant.EvalBodyPoseInWorld(temp_plant_context, plant.GetBodyByName("body"))
model_instance_ball = plant.GetModelInstanceByName("ball")
X_WOball_initial = get_initial_pose(
    plant, "base_link", model_instance_ball, temp_plant_context
)


# Build trajectory keyframes
X_OG, X_WG_pick = design_grasp_pose(X_WOball_initial)
X_WG_prepick = approach_pose(X_WG_pick)

# === New: compute hoop pose (world → hoop) ===
model_instance_hoop = plant.GetModelInstanceByName("hoop_model")
hoop_body = plant.GetBodyByName("base_link_hoop", model_instance_hoop)
X_WH = plant.EvalBodyPoseInWorld(temp_plant_context, hoop_body)


# Ball-in-gripper transform from design_grasp_pose
X_GO = X_OG.inverse()

X_WG_hold = X_WGinitial
X_WO_hold = X_WG_hold @ X_GO

heading = np.arctan2(X_WH.translation()[1], X_WH.translation()[0])
q0 = heading - np.pi

PRETHROW_JA = np.array([
    q0,        # base heading
    0.0,       # shoulder pan
    0.0,       # shoulder lift
    1.8,       # big bend
    0.0,       # wrist 1
    -1.8,      # big opposite bend
    0.0,       # wrist 2
])
THROWEND_JA = np.array([
    q0,
    0.0,
    0.0,
    0.4,       # extend
    0.0,
    -0.4,      # extend
    0.0,
])
FOLLOW_JA = np.array([
    q0,
    0.0,
    0.0,
    0.1,       # slight extra follow-through
    0.0,
    -0.1,
    0.0,
])

iiwa = plant.GetModelInstanceByName("iiwa")
wsg  = plant.GetModelInstanceByName("wsg")
gripper_body = plant.GetBodyByName("body", wsg)  # <- this is your EE



X_WG_prethrow = pose_from_q(plant, PRETHROW_JA, temp_plant_context)
X_WG_release  = pose_from_q(plant, THROWEND_JA, temp_plant_context)
X_WG_follow   = pose_from_q(plant, FOLLOW_JA, temp_plant_context)

    
R_WG = X_WG_hold.rotation()
p_hold = X_WG_hold.translation()



opened = 0.107
closed = 0.0
keyframes = [
    (X_WGinitial,  opened),  # start at home, gripper open
    (X_WG_prepick, opened),  # move above/behind the ball
    (X_WG_pick,    opened),  # descend onto the ball
    (X_WG_pick,    closed),  # close on the ball
    (X_WG_prepick, closed),  # lift back up with ball grasped
    (X_WG_hold,  closed),  
]


keyframes.append((X_WG_prethrow, closed))
keyframes.append((X_WG_release,  closed)) 
keyframes.append((X_WG_release,  opened))
keyframes.append((X_WG_follow,  opened))
    


# unpack the keyframes and use them to build `Trajectory` objects
# note: we specify each keyframe as occuring 2 seconds after the last.
gripper_poses = [kf[0] for kf in keyframes]
finger_states = np.asarray([kf[1] for kf in keyframes]).reshape(1, -1)

sample_times = []
t = 0.0
for i in range(len(keyframes)):
    sample_times.append(t)

    if i < 5:
        t += 1.0   # pickup stuff
    elif i == 5:
        t += 1.0   # hold -> prethrow
    elif i == 6:
        t += 0.3   # prethrow -> release: throw motion
    elif i == 7:
        t += 0.05  # closed release -> open release: quick open
    else:
        t += 0.5   # follow-through

traj_V_G, traj_wsg_command = make_trajectory(gripper_poses, finger_states, sample_times)

# V_G_source defines a trajectory over gripper velocities. Add it to the system.
V_G_source = builder.AddSystem(TrajectorySource(traj_V_G))
# Add the DiffIK controller we just defined to the system
controller = builder.AddSystem(PseudoInverseController(plant))
# The HardwareStation expects robot commands in terms of joint angles.
# We define the `integrator` system to map from joint_velocities to joint_angles.
integrator = builder.AddSystem(Integrator(7))
# wsg_source defines a trajectory of finger positions. Add it to the system.
wsg_source = builder.AddSystem(TrajectorySource(traj_wsg_command))

# TODO: connect the joint velocity source to the pseudoinverse controller
builder.Connect(
    V_G_source.get_output_port(),
    controller.GetInputPort("V_WG"),
)

# TODO: connect the controller to integrator to get joint angle commands
builder.Connect(
    controller.GetOutputPort("iiwa.velocity"),
    integrator.get_input_port(),
)

# TODO: connect the joint angles computed by the integrateor to the iiwa.position port on the manipulation station
builder.Connect(
    integrator.get_output_port(),
    station.GetInputPort("iiwa.position"),
)

# TODO: connect the "iiwa.position_measured" port on the station back to the relevant input port on the controller
builder.Connect(
    station.GetOutputPort("iiwa.position_measured"),
    controller.GetInputPort("iiwa.position"),
)

zero_torque = builder.AddSystem(ConstantVectorSource(np.zeros(7)))
builder.Connect(
    zero_torque.get_output_port(),
    station.GetInputPort("iiwa.torque"),
)

# TODO: connect the wsg_source to the "wsg.position" input port of the station
builder.Connect(
    wsg_source.get_output_port(),
    station.GetInputPort("wsg.position"),
)


# visualize axes (useful for debugging)
scenegraph = station.GetSubsystemByName("scene_graph")
AddFrameTriadIllustration(
    scene_graph=scenegraph,
    body=plant.GetBodyByName("base_link", model_instance_ball),
    length=0.1,
)

AddFrameTriadIllustration(
    scene_graph=scenegraph, body=plant.GetBodyByName("body"), length=0.1
)

diagram = builder.Build()








# Define the simulator.
simulator = Simulator(diagram)
context = simulator.get_mutable_context()
station_context = station.GetMyContextFromRoot(context)
integrator.set_integral_value(
    integrator.GetMyContextFromRoot(context),
    plant.GetPositions(
        plant.GetMyContextFromRoot(context),
        plant.GetModelInstanceByName("iiwa"),
    ),
)
diagram.ForcedPublish(context)
print(f"sanity check, simulation will run for {traj_V_G.end_time()} seconds")

# run simulation!
meshcat.StartRecording()
if running_as_notebook:
    simulator.set_target_realtime_rate(1.0)
simulator.AdvanceTo(traj_V_G.end_time())

meshcat.StopRecording()
meshcat.PublishRecording()