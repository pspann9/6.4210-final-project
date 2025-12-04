import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import trimesh
from pydrake.all import (
    AddFrameTriadIllustration,
    BasicVector,
    Concatenate,
    Context,
    Diagram,
    DiagramBuilder,
    Integrator,
    JacobianWrtVariable,
    LeafSystem,
    MultibodyPlant,
    PiecewisePolynomial,
    PiecewisePose,
    PointCloud,
    Rgba,
    RigidTransform,
    RobotDiagram,
    RollPitchYaw,
    RotationMatrix,
    Simulator,
    StartMeshcat,
    Trajectory,
    TrajectorySource,
)

from manipulation import running_as_notebook
from manipulation.exercises.grader import Grader
from manipulation.icp import IterativeClosestPoint
# from manipulation.ball_generation import create_sdf_asset_from_ball
from manipulation.meshcat_utils import AddMeshcatTriad
from manipulation.station import (
    AddPointClouds,
    LoadScenario,
    MakeHardwareStation,
    RobotDiagram,
)
from manipulation.utils import RenderDiagram

# Start meshcat for visualization
meshcat = StartMeshcat()
print("Click the link above to open Meshcat in your browser!")


ball_sdf = f"{Path.cwd()}/sdfs/sphere.sdf"


# Add the directives for the bimanual IIWA arms, table, and initials


def generate_bimanual_IIWA14_with_assets_directives_file() -> (
    tuple[Diagram, RobotDiagram]
):
    table_sdf = f"{Path.cwd()}/sdfs/table.sdf"
    ball_sdf = f"{Path.cwd()}/sdfs/sphere.sdf"

    directives_yaml = f"""directives:
- add_model:
    name: iiwa
    file: package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf
    default_joint_positions:
        iiwa_joint_1: [-1.57]
        iiwa_joint_2: [0.1]
        iiwa_joint_3: [0]
        iiwa_joint_4: [-1.2]
        iiwa_joint_5: [0]
        iiwa_joint_6: [ 1.6]
        iiwa_joint_7: [0]
- add_weld:
    parent: world
    child: iiwa::iiwa_link_0
    # X_PC:
    #     translation: [0, -0.5, 0]
    #     rotation: !Rpy {{ deg: [0, 0, 180] }}
- add_model:
    name: wsg
    file: package://manipulation/hydro/schunk_wsg_50_with_tip.sdf
- add_weld:
    parent: iiwa::iiwa_link_7
    child: wsg::body
    X_PC:
        translation: [0, 0, 0.09]
        rotation: !Rpy {{ deg: [90, 0, 90]}}
        
        
- add_model:
    name: table
    file: file://{table_sdf}
- add_weld:
    parent: world
    child: table::table_link
    X_PC:
        translation: [0.0, 0.0, -0.05]
        rotation: !Rpy {{ deg: [0, 0, -90] }}
        
- add_model:
    name: ball
    file: file://{ball_sdf}
    default_free_body_pose:
        body_link:
            translation: [0.55, 0, 0.00]
            rotation: !Rpy {{ deg: [0, 0, 0] }}

- add_model:
    name: hoop_model
    file: file:///workspaces/6.4210-final-project/sdfs/basketball_hoop.sdf
- add_weld:
    parent: world
    child: hoop_model::base_link_hoop  
    X_PC:
        translation: [0, -5, 3.048]
        rotation: !Rpy {{ deg: [0, 0, 90] }}

- add_model:
        name: obstacle_back
        file: file:///workspaces/6.4210-final-project/sdfs/obstacle.sdf

- add_weld:
    parent: world
    child: obstacle_back::link  
    X_PC:
        translation: [0, 1, 0]
        rotation: !Rpy {{ deg: [90, 0, 0] }}

- add_model:
    name: obstacle_front
    file: file:///workspaces/6.4210-final-project/sdfs/obstacle.sdf

- add_weld:
    parent: world
    child: obstacle_front::link  
    X_PC:
        translation: [0, -1, 0]
        rotation: !Rpy {{ deg: [90, 0, 0] }}

- add_model:
    name: obstacle_left
    file: file:///workspaces/6.4210-final-project/sdfs/obstacle.sdf

- add_weld:
    parent: world
    child: obstacle_left::link  
    X_PC:
        translation: [1, 0, 0]
        rotation: !Rpy {{ deg: [90, 0, 0] }}

# - add_model:
#     name: obstacle_right
#     file: file:///workspaces/6.4210-final-project/sdfs/obstacle.sdf

# - add_weld:
#     parent: world
#     child: obstacle_right::link  
#     X_PC:
#         translation: [-1, 0, 0]
#         rotation: !Rpy {{ deg: [90, 0, 0] }}
"""
    os.makedirs("directives", exist_ok=True)

    with open(
        "directives/bimanual_IIWA14_with_table_and_initials_and_assets.dmd.yaml", "w"
    ) as f:
        f.write(directives_yaml)


generate_bimanual_IIWA14_with_assets_directives_file()



def create_camera_directives() -> None:
    camera_directives_yaml = """
directives:
- add_frame:
    name: camera0_origin
    X_PF:
        base_frame: world
        rotation: !Rpy { deg: [-140.0, 0.0, 180.0]}
        translation: [0, 2, 2] # [0, 0.8, 0.5]

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
        translation: [2, 0.1, 2] # [0.8, 0.1, 0.5]

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
        translation: [-2, 0.1, 2] # [-0.8, 0.1, 0.5]

- add_model:
    name: camera2
    file: package://manipulation/camera_box.sdf

- add_weld:
    parent: camera2_origin
    child: camera2::base
"""
    with open("directives/camera_directives.dmd.yaml", "w") as f:
        f.write(camera_directives_yaml)


create_camera_directives()


def create_bimanual_IIWA14_with_assets_and_cameras_scenario() -> None:
    # TODO: create a scenario yaml with the directives added with `add_directives`
    robots = Path("directives/bimanual_IIWA14_with_table_and_initials_and_assets.dmd.yaml").resolve()
    cams   = Path("directives/camera_directives.dmd.yaml").resolve()

    scenario_yaml = f"""
directives:
    - add_directives:
        file: file://{robots.as_posix()}
    - add_directives:
        file: file://{cams.as_posix()}

model_drivers:
    iiwa: !IiwaDriver
        control_mode: position_only
        hand_model_name: wsg
    wsg: !SchunkWsgDriver {{}}

cameras:
    camera0:
        name: camera0
        depth: True
        X_PB:
            base_frame: camera0::base
    camera1:
        name: camera1
        depth: True
        X_PB:
            base_frame: camera1::base
    camera2:
        name: camera2
        depth: True
        X_PB:
            base_frame: camera2::base
"""
    # TODO: add the camera configs and iiwa drivers with `add_cameras` and `add_iiwa_drivers`

    os.makedirs("scenarios", exist_ok=True)

    with open(
        "scenarios/bimanual_IIWA14_with_table_and_initials_and_assets_and_cameras.scenario.yaml",
        "w",
    ) as f:
        f.write(scenario_yaml)


create_bimanual_IIWA14_with_assets_and_cameras_scenario()


def create_bimanual_IIWA14_with_table_and_initials_and_assets_and_cameras() -> (
    tuple[DiagramBuilder, RobotDiagram]
):
    # TODO: Load the scenario created above into a Scenario object
    scenario = LoadScenario(
       filename="scenarios/bimanual_IIWA14_with_table_and_initials_and_assets_and_cameras.scenario.yaml"
    )

    # TODO: Create HardwareStation with the scenario and meshcat
    station = MakeHardwareStation(scenario, meshcat)

    # TODO: Make a DiagramBuilder, add the station, and build the diagram
    builder = DiagramBuilder()
    station = builder.AddSystem(station)

    # TODO: Add the point clouds to the diagram with AddPointClouds
    to_point_cloud = AddPointClouds(
        scenario=scenario, station=station, builder=builder, meshcat=meshcat
    )

    # TODO: export the point cloud outputs to the builder
    for name, system in to_point_cloud.items():
        pc_port = system.point_cloud_output_port()
        builder.ExportOutput(pc_port, f"{name}_point_cloud")


    # TODO: Return the builder AND the station (notice that here we will need both)
    
    return builder, station


builder, station = (
    create_bimanual_IIWA14_with_table_and_initials_and_assets_and_cameras()
)

# in order to debug, we will build the diagram once here.
diagram = builder.Build()

# visualize the diagram
RenderDiagram(diagram, max_depth=1)

# publish the diagram with some default context
context = diagram.CreateDefaultContext()
diagram.ForcedPublish(context)


N_SAMPLE_POINTS = 1500

# ball
sdf_path = Path("sdfs/sphere.sdf")
obj_path = (sdf_path.parent / "sphere.obj")
mesh = trimesh.load(obj_path.as_posix(), force='mesh')

pts = mesh.sample(N_SAMPLE_POINTS)

xyzs = pts.T 
cloud = PointCloud(xyzs.shape[1])
cloud.mutable_xyzs()[:] = xyzs



# calculate the actual relative positions of each of the letters to use as a reference
# for cropping the point clouds.
plant = station.plant()
plant_context = diagram.GetSubsystemContext(plant, context)

world_frame = plant.world_frame()

model_ball = plant.GetModelInstanceByName(f"ball")
frame_ball = plant.GetFrameByName(
    f"body_link", model_instance=model_ball
)
X_PC_ball = plant.CalcRelativeTransform(plant_context, world_frame, frame_ball)


pad = np.array([0.15, 0.15, 0.15])
center_W = X_PC_ball.translation()
ball_lower = center_W - pad
ball_upper = center_W + pad

camera0_ball_point_cloud = diagram.GetOutputPort("camera0_point_cloud").Eval(context).Crop(ball_lower, ball_upper)
camera1_ball_point_cloud = diagram.GetOutputPort("camera1_point_cloud").Eval(context).Crop(ball_lower, ball_upper)
camera2_ball_point_cloud = diagram.GetOutputPort("camera2_point_cloud").Eval(context).Crop(ball_lower, ball_upper)

##################
# obstacle
# Example for obstacle_back
model_obstacle = plant.GetModelInstanceByName("obstacle_back")
frame_obstacle = plant.GetFrameByName("link", model_instance=model_obstacle)
X_PC_obstacle = plant.CalcRelativeTransform(plant_context, world_frame, frame_obstacle)

pad = np.array([1.05, 1.05, 1.05])  # adjust based on obstacle size
center_W = X_PC_obstacle.translation()
obstacle_lower = center_W - pad
obstacle_upper = center_W + pad

camera0_obstacle_pc = diagram.GetOutputPort("camera0_point_cloud").Eval(context).Crop(obstacle_lower, obstacle_upper)
camera1_obstacle_pc = diagram.GetOutputPort("camera1_point_cloud").Eval(context).Crop(obstacle_lower, obstacle_upper)
camera2_obstacle_pc = diagram.GetOutputPort("camera2_point_cloud").Eval(context).Crop(obstacle_lower, obstacle_upper)

##################

# This will visualize the bounding boxes for each of the letters in the point clouds
# Use it to adjust your bounding boxes to fit the letters.
if ball_lower is not None and ball_upper is not None:
    meshcat.SetLineSegments(
        "bounding_line",
        np.array(ball_lower).T,
        np.array(ball_upper).T,
        1.0,
        Rgba(0, 0, 0),
    )

# TODO: concatenate the point clouds
merged_point_cloud = Concatenate([camera0_ball_point_cloud, camera1_ball_point_cloud, camera2_ball_point_cloud])

# TODO: downsample the point clouds
# HINT: use the `VoxelizedDownSample` method for PointCloud
downsampled_point_cloud = merged_point_cloud.VoxelizedDownSample(0.005)


# TODO: implement a function that removes all points in the point cloud that lie below z=0.01


def remove_table_points(point_cloud: PointCloud) -> PointCloud:
    xyz = point_cloud.xyzs()                 # 3 x N
    mask = np.isfinite(xyz).all(axis=0) & (xyz[2, :] >= 0.01)
    idx = np.nonzero(mask)[0]

    out = PointCloud(idx.size)      # new cloud with only XYZ
    out.mutable_xyzs()[:] = xyz[:, idx]
    return out

def remove_non_back_points(point_cloud: PointCloud) -> PointCloud:
    xyz = point_cloud.xyzs()                 # 3 x N
    mask = np.isfinite(xyz).all(axis=0) & (xyz[1, :] >= 0.5)
    idx = np.nonzero(mask)[0]

    out = PointCloud(idx.size)      # new cloud with only XYZ
    out.mutable_xyzs()[:] = xyz[:, idx]
    return out



# TODO: remove the table points from the concatenated point clouds
ball_point_cloud = remove_table_points(merged_point_cloud)
ball_point_cloud = ball_point_cloud.VoxelizedDownSample(0.005)

####################
merged_obstacle_pc = Concatenate([camera0_obstacle_pc, camera1_obstacle_pc, camera2_obstacle_pc])
merged_obstacle_pc = merged_obstacle_pc.VoxelizedDownSample(0.005)
merged_obstacle_pc = remove_table_points(merged_obstacle_pc)
merged_obstacle_pc = remove_non_back_points(merged_obstacle_pc)
####################


# visualize the concatenated point clouds
meshcat.SetObject(
    "ball_point_cloud", ball_point_cloud, point_size=0.05, rgba=Rgba(1, 0, 0)
)

#################
meshcat.SetObject(
    "obstacle_point_cloud", merged_obstacle_pc, point_size=0.05, rgba=Rgba(0, 0, 1)
)
#################

MAX_ITERATIONS = 25 

# TODO: set initial guesses for each of the letters along with the maximum number of iterations
# These can be rough guesses as ICP will do the bulk of the work of aligning the point clouds.
initial_guess = RigidTransform(
    RotationMatrix.MakeZRotation(0.0),  
    [ 0.5, 0.0, 0.0 ]            
)

# TODO: convert both the model and generated point clouds to numpy arrays to pass into the ICP function
model_points = cloud.xyzs() 
scene_points = ball_point_cloud.VoxelizedDownSample(0.005).xyzs()

# TODO: register the point clouds with the model_points point clouds using ICP for each of the letters
ball_X_Ohat, error_ball = IterativeClosestPoint(
    p_Om=model_points,          
    p_Ws=scene_points,           
    X_Ohat=initial_guess,     
    max_iterations=MAX_ITERATIONS
)

################
obstacle_mesh = trimesh.load("sdfs/construction_cone.obj", force='mesh')
obstacle_points = obstacle_mesh.sample(N_SAMPLE_POINTS).T
obstacle_cloud = PointCloud(obstacle_points.shape[1])
obstacle_cloud.mutable_xyzs()[:] = obstacle_points

# Run ICP
initial_guess_obstacle = RigidTransform(
    RotationMatrix.MakeXRotation(90.0),  
    [ 0.0, 1.0, 0.0 ]      
)  # rough initial position
obstacle_X_Ohat, error_obstacle = IterativeClosestPoint(
    p_Om=obstacle_points,
    p_Ws=merged_obstacle_pc.xyzs(),
    X_Ohat=initial_guess_obstacle,
    max_iterations=MAX_ITERATIONS,
)
################


print("Obstacle pose:", obstacle_X_Ohat)



# check the error in the registration for each of the letters below:
# if it has converged, all errors should be close to zero
np.set_printoptions(precision=3, suppress=True)
error_ball = ball_X_Ohat.inverse().multiply(X_PC_ball)

rpy = RollPitchYaw(error_ball.rotation()).vector()
xyz = error_ball.translation()
# print(f"{your_initial}_ball error: rpy: {rpy}, xyz: {xyz}")



# TODO: modify the functions below to work with your letters. This will require quite a bit of trial and error.


def design_grasp_pose(X_WO: RigidTransform) -> tuple[RigidTransform, RigidTransform]:
    R_OG = (
        RollPitchYaw(0, 0, np.pi).ToRotationMatrix()
        @ RollPitchYaw(-np.pi / 2, 0, 0).ToRotationMatrix()
    )
    # p_OG = [0.07, 0.08, 0.12]
    p_OG = [0, 0, 0]
    X_OG = RigidTransform(R_OG, p_OG)
    X_WG = X_WO.multiply(X_OG)
    return X_OG, X_WG


def design_pregrasp_pose(
    X_WG: RigidTransform,
) -> tuple[RigidTransform, RigidTransform, RigidTransform]:
    X_GGApproach = RigidTransform([0.0, -0.2, 0.0])
    X_WGApproach = X_WG @ X_GGApproach
    return X_WGApproach


def design_pregoal_pose(
    X_WG: RigidTransform,
) -> tuple[RigidTransform, RigidTransform, RigidTransform]:
    X_GGApproach = RigidTransform([0.0, 0.0, -0.2])
    X_WGApproach = X_WG @ X_GGApproach
    return X_WGApproach


# The goal poses have been modified to include the third initial.
def design_goal_poses(
    X_WO: RigidTransform, X_OG: RigidTransform
) -> tuple[RigidTransform, RigidTransform, RigidTransform]:
    X_WOgoal = X_WO @ RigidTransform(
        R=RotationMatrix.MakeXRotation(np.pi / 2), p=np.array([-0.1, 0.2, 0.0])
    )
    X_WGgoal = X_WOgoal @ X_OG
    return X_WGgoal


def design_postgoal_pose(
    X_WG: RigidTransform,
) -> tuple[RigidTransform, RigidTransform, RigidTransform]:
    X_GGApproach = RigidTransform([0.0, 0.0, -0.2])
    X_WGApproach = X_WG @ X_GGApproach
    return X_WGApproach


def make_trajectory(
    X_Gs: list[RigidTransform], finger_values: np.ndarray, sample_times: list[float]
) -> tuple[Trajectory, PiecewisePolynomial]:
    robot_position_trajectory = PiecewisePose.MakeLinear(sample_times, X_Gs)
    robot_velocity_trajectory = robot_position_trajectory.MakeDerivative()
    traj_wsg_command = PiecewisePolynomial.FirstOrderHold(sample_times, finger_values)
    return robot_velocity_trajectory, traj_wsg_command


class PseudoInverseController(LeafSystem):
    def __init__(self, plant: MultibodyPlant):
        LeafSystem.__init__(self)
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        self._iiwa = plant.GetModelInstanceByName("iiwa")
        self._G = plant.GetBodyByName("body").body_frame()
        self._W = plant.world_frame()

        self.V_G_port = self.DeclareVectorInputPort("V_WG", 6)
        self.q_port = self.DeclareVectorInputPort("iiwa.position", 7)
        self.DeclareVectorOutputPort("iiwa.velocity", 7, self.CalcOutput)
        self.iiwa_start = plant.GetJointByName("iiwa_joint_1").velocity_start()
        self.iiwa_end = plant.GetJointByName("iiwa_joint_7").velocity_start()

    def CalcOutput(self, context: Context, output: BasicVector):
        """
        fill in our code below.
        """
        # evaluate the V_G_port and q_port on the current context to get those values.

        V_WG_des = np.asarray(self.V_G_port.Eval(context)).reshape(6)   # [w; v] expressed in W
        q = np.asarray(self.q_port.Eval(context)).reshape(7)            # iiwa positions

        # update the positions of the internal _plant_context according to `q`.
        # HINT: you can write to a plant context by calling `self._plant.SetPositions`
        self._plant.SetPositions(self._plant_context, self._iiwa, q)

        # Compute the gripper jacobian
        # HINT: the jacobian is 6 x N, with N being the number of DOFs.
        # We only want the 6 x 7 submatrix corresponding to the IIWA
        #    V_WG_W = J_WG * v  (J is 6 x nv).
        J_WG_full = self._plant.CalcJacobianSpatialVelocity(
            self._plant_context,
            JacobianWrtVariable.kV,
            self._G,                   # frame B (gripper)
            np.zeros(3),               # p_BG (point: origin of G)
            self._W,                   # frame A (world)
            self._W                    # expressed in W
        )
        J = J_WG_full[:, self.iiwa_start : self.iiwa_end + 1]  # 6x7


        # compute `v` by mapping the gripper velocity (from the V_G_port) to the joint space
        lam2 = 1e-4
        #    v = Jᵀ (J Jᵀ + λ² I)⁻¹ V
        v = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(6), V_WG_des)

        output.SetFromVector(v)



# we will rebuild the diagram in order to add the controller and integrator systems we need.
builder, station = (
    create_bimanual_IIWA14_with_table_and_initials_and_assets_and_cameras()
)
plant = station.GetSubsystemByName("plant")

station_context = station.CreateDefaultContext()
plant_context = plant.GetMyContextFromRoot(station_context)

# get initial poses of gripper and objects
X_WGinitial = plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("body"))

# TODO: copy over the poses registered from ICP above
X_WOinitial = ball_X_Ohat  # TODO

print("HERE", X_WOinitial)

X_WOinitial = RigidTransform(RollPitchYaw(0, 0, 0).ToRotationMatrix(), X_WOinitial.translation())

print("HERE", X_WOinitial)

# Build trajectory keyframes
X_OG, X_WGpick = design_grasp_pose(X_WOinitial)
X_WGprepick = design_pregrasp_pose(X_WGpick)
X_WGgoal = design_goal_poses(X_WOinitial, X_OG)
X_WGpregoal = design_pregoal_pose(X_WGgoal)
X_WGpostgoal = design_postgoal_pose(X_WGgoal)

# constants for finger distances when the gripper is opened or closed
opened = 0.107
closed = 0.0

# list of keyframes, formatted as (gripper poses, finger states)
# for each object the robot starts in its default pose with its gripper open
# then it goes to the prepick pose, the pick pose, closes the gripper, and then goes
# to the place pose
keyframes = [
    ("X_WGinitial", X_WGinitial, opened),
    ("X_WGprepick", X_WGprepick, opened),
    ("X_WGpick", X_WGpick, opened),
    ("X_WGpick", X_WGpick, closed),
    ("X_WGpregoal", X_WGpregoal, closed),
    ("X_WGgoal", X_WGgoal, closed),
    ("X_WGgoal", X_WGgoal, opened),
    ("X_WGpostgoal", X_WGpostgoal, opened),
    ("X_WGinitial", X_WGinitial, opened),
]


# # TODO: copy over your work from the previous pset
gripper_poses = [keyframe[1] for keyframe in keyframes]
finger_states = np.asarray([keyframe[2] for keyframe in keyframes]).reshape(1, -1)
sample_times = [3 * i for i in range(len(gripper_poses))]
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

# TODO: connect the wsg_source to the "wsg.position" input port of the station
builder.Connect(
    wsg_source.get_output_port(),
    station.GetInputPort("wsg.position"),
)

# visualize axes (useful for debugging)
scenegraph = station.GetSubsystemByName("scene_graph")
AddFrameTriadIllustration(
    scene_graph=scenegraph,
    body=plant.GetBodyByName(f"body_link"),
    length=0.1,
)
AddFrameTriadIllustration(
    scene_graph=scenegraph, body=plant.GetBodyByName("body"), length=0.1
)

diagram = builder.Build()



# Define the simulator

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
simulator.set_target_realtime_rate(1.0)
simulator.AdvanceTo(traj_V_G.end_time() + 2) # if running_as_notebook else 0.01)
meshcat.StopRecording()
meshcat.PublishRecording()
