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


def copy_filtered(point_cloud: PointCloud, idx: np.ndarray) -> PointCloud:
    """Return a new PointCloud containing only indices in idx,
    preserving XYZ and any present per-point fields (RGBs, normals)."""
    out = PointCloud(idx.size, fields=point_cloud.fields())

    # XYZs always exist
    out.mutable_xyzs()[:] = point_cloud.xyzs()[:, idx]

    # Copy RGBs only if the source has them
    if point_cloud.has_rgbs():
        out.mutable_rgbs()[:] = point_cloud.rgbs()[:, idx]

    # Copy normals if present
    if point_cloud.has_normals():
        out.mutable_normals()[:] = point_cloud.normals()[:, idx]

    # Add any other field checks here if you need them (e.g., intensities).
    return out


def remove_table_points(point_cloud: PointCloud) -> PointCloud:
    xyz = point_cloud.xyzs()                 # 3 x N
    mask = np.isfinite(xyz).all(axis=0) & (xyz[2, :] >= 0.01)
    idx = np.nonzero(mask)[0]

    # quick path: no filtering required
    if idx.size == point_cloud.size():
        return point_cloud

    return copy_filtered(point_cloud, idx)

def rgb_at_pose(point_cloud, X_Ohat: RigidTransform, query_point: np.ndarray = None, radius: float = None):
    """
    Get the RGB color from a point cloud at a given world pose.

    Args:
        point_cloud: Drake PointCloud with xyzs() and rgbs().
        X_Ohat: RigidTransform representing the pose of the object in world frame.
        query_point: Optional 3-element array specifying a query position in world frame.
                     If None, defaults to X_Ohat.translation() (object center).
        radius: Optional radius for averaging points around query_point. If None, uses closest point.

    Returns:
        color: 3-element np.ndarray with RGB values in [0,1].
    """
    # Transform points to world frame
    xyzs = point_cloud.xyzs()  # 3 x N
    rgbs = point_cloud.rgbs()  # 3 x N

    R = X_Ohat.rotation().matrix()
    t = X_Ohat.translation()
    xyzs_W = R @ xyzs + t[:, np.newaxis]

    # Default query point is object center
    if query_point is None:
        query_point = t

    if radius is None:
        # Return RGB of closest point
        dists = np.linalg.norm(xyzs_W - query_point[:, np.newaxis], axis=0)
        idx = np.argmin(dists)
        color = rgbs[:, idx]
    else:
        # Average RGB over all points within radius
        mask = np.linalg.norm(xyzs_W - query_point[:, np.newaxis], axis=0) < radius
        if np.sum(mask) == 0:
            # fallback: closest point
            dists = np.linalg.norm(xyzs_W - query_point[:, np.newaxis], axis=0)
            idx = np.argmin(dists)
            color = rgbs[:, idx]
        else:
            color = np.mean(rgbs[:, mask], axis=1)

    return color

def copy_filtered(point_cloud: PointCloud, idx: np.ndarray) -> PointCloud:
    """Return a new PointCloud containing only indices in idx,
    preserving XYZ and any present per-point fields (RGBs, normals)."""
    out = PointCloud(idx.size, fields=point_cloud.fields())

    # XYZs always exist
    out.mutable_xyzs()[:] = point_cloud.xyzs()[:, idx]

    # Copy RGBs only if the source has them
    if point_cloud.has_rgbs():
        out.mutable_rgbs()[:] = point_cloud.rgbs()[:, idx]

    # Copy normals if present
    if point_cloud.has_normals():
        out.mutable_normals()[:] = point_cloud.normals()[:, idx]

    # Add any other field checks here if you need them (e.g., intensities).
    return out

def remove_table_points(point_cloud: PointCloud) -> PointCloud:
    xyz = point_cloud.xyzs()                 # 3 x N
    mask = np.isfinite(xyz).all(axis=0) & (xyz[2, :] >= 0.01)
    idx = np.nonzero(mask)[0]

    # quick path: no filtering required
    if idx.size == point_cloud.size():
        return point_cloud

    return copy_filtered(point_cloud, idx)

def rgb_at_pose(point_cloud, X_Ohat: RigidTransform, query_point: np.ndarray = None, radius: float = None):
    # Transform points to world frame
    xyzs = point_cloud.xyzs()  # 3 x N
    rgbs = point_cloud.rgbs()  # 3 x N

    R = X_Ohat.rotation().matrix()
    t = X_Ohat.translation()
    xyzs_W = R @ xyzs + t[:, np.newaxis]

    # Default query point is object center
    if query_point is None:
        query_point = t

    if radius is None:
        # Return RGB of closest point
        dists = np.linalg.norm(xyzs_W - query_point[:, np.newaxis], axis=0)
        idx = np.argmin(dists)
        color = rgbs[:, idx]
    else:
        # Average RGB over all points within radius
        mask = np.linalg.norm(xyzs_W - query_point[:, np.newaxis], axis=0) < radius
        if np.sum(mask) == 0:
            # fallback: closest point
            dists = np.linalg.norm(xyzs_W - query_point[:, np.newaxis], axis=0)
            idx = np.argmin(dists)
            color = rgbs[:, idx]
        else:
            color = np.mean(rgbs[:, mask], axis=1)

    return color

def perceive_ball_and_bin(scenario, meshcat):
    """
    Returns:
        X_WB_ball: RigidTransform of the ball in world.
        X_WB_bin:  RigidTransform of the bin in world.
    """
    station = MakeHardwareStation(scenario, meshcat=meshcat, hardware=False)
    builder = DiagramBuilder()
    station = builder.AddSystem(station)

    ######################### Ball pose ################################
    to_point_cloud = AddPointClouds(
        scenario=scenario, station=station, builder=builder, meshcat=meshcat
    )

    for name, system in to_point_cloud.items():
        pc_port = system.point_cloud_output_port()
        builder.ExportOutput(pc_port, f"{name}_point_cloud")
        
    
        
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()

        
    N_SAMPLE_POINTS = 1500

    sdf_path = Path("sdfs/sphere.sdf")
    obj_path = (sdf_path.parent / "sphere.obj")
    mesh = trimesh.load(obj_path.as_posix(), force='mesh')

    pts = mesh.sample(N_SAMPLE_POINTS)

    xyzs = pts.T 
    cloud = PointCloud(xyzs.shape[1])
    cloud.mutable_xyzs()[:] = xyzs
    
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
    camera3_ball_point_cloud = diagram.GetOutputPort("camera3_point_cloud").Eval(context).Crop(ball_lower, ball_upper)
    
    
    if ball_lower is not None and ball_upper is not None:
        meshcat.SetLineSegments(
            "bounding_line",
            np.array(ball_lower).T,
            np.array(ball_upper).T,
            1.0,
            Rgba(0, 0, 0),
        )
    merged_point_cloud = Concatenate([camera0_ball_point_cloud, camera1_ball_point_cloud, camera2_ball_point_cloud, camera3_ball_point_cloud])
    downsampled_point_cloud = merged_point_cloud.VoxelizedDownSample(0.005)
    
    ball_point_cloud = remove_table_points(merged_point_cloud)
    ball_point_cloud = ball_point_cloud.VoxelizedDownSample(0.005)
    
    # meshcat.SetObject(
    #     "ball_point_cloud", ball_point_cloud, point_size=0.05, rgba=Rgba(1, 0, 0)
    # )
    
    MAX_ITERATIONS = 25 
    
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
    
    ball_color = rgb_at_pose(ball_point_cloud, ball_X_Ohat)
    ball_color_avg = rgb_at_pose(ball_point_cloud, ball_X_Ohat, radius=0.05)
    print("Average ball color (RGB):", ball_color_avg)
    
    np.set_printoptions(precision=3, suppress=True)
    error_ball = ball_X_Ohat.inverse().multiply(X_PC_ball)

    rpy = RollPitchYaw(error_ball.rotation()).vector()
    xyz = error_ball.translation()
    # rgb = ball_point_cloud.rgbs()
    # mean_rgb = rgb.mean(axis=1)
    # print(mean_rgb)

    p_WB = ball_X_Ohat.translation()

    X_WB_ball = RigidTransform(
        RotationMatrix(),
        p_WB
    )

    ################## Bin pose ###################
    model_bin1 = plant.GetModelInstanceByName("bin_red")
    frame_bin1 = plant.GetFrameByName("bin_base", model_instance=model_bin1)
    X_PC_bin1 = plant.CalcRelativeTransform(plant_context, world_frame, frame_bin1)
    
    pad = np.array([.3, .4, .4])
    center_W = X_PC_bin1.translation()
    bin1_lower = center_W - pad
    bin1_upper = center_W + pad

    camera0_bin1_pc = diagram.GetOutputPort("camera0_point_cloud").Eval(context).Crop(bin1_lower, bin1_upper)
    camera1_bin1_pc = diagram.GetOutputPort("camera1_point_cloud").Eval(context).Crop(bin1_lower, bin1_upper)
    camera2_bin1_pc = diagram.GetOutputPort("camera2_point_cloud").Eval(context).Crop(bin1_lower, bin1_upper)
    camera3_bin1_pc = diagram.GetOutputPort("camera3_point_cloud").Eval(context).Crop(bin1_lower, bin1_upper)


    ####################
    merged_bin1_pc = Concatenate([camera0_bin1_pc, camera1_bin1_pc, camera2_bin1_pc, camera3_bin1_pc])
    merged_bin1_pc = merged_bin1_pc.VoxelizedDownSample(0.005)
    merged_bin1_pc = remove_table_points(merged_bin1_pc)
    # merged_bin1_pc = remove_non_back_points(merged_bin1_pc)
    
    # meshcat.SetObject(
    #     "bin1_point_cloud", merged_bin1_pc, point_size=0.05, rgba=Rgba(1, 1, 1)
    # )

    # p = Concatenate([diagram.GetOutputPort("camera0_point_cloud").Eval(context), diagram.GetOutputPort("camera1_point_cloud").Eval(context), diagram.GetOutputPort("camera2_point_cloud").Eval(context), diagram.GetOutputPort("camera3_point_cloud").Eval(context)])
    # meshcat.SetObject("p", p, point_size=0.01, rgba=Rgba(0,1,0))

    sdf_path_bin = Path("sdfs/bin.sdf")
    obj_path_bin = (sdf_path_bin.parent / "bin.obj")
    bin1_mesh = trimesh.load(obj_path_bin.as_posix(), force='mesh')

    # bin1_mesh = trimesh.load("sdfs/construction_cone.obj", force='mesh')
    bin1_points = bin1_mesh.sample(N_SAMPLE_POINTS).T
    bin1_cloud = PointCloud(bin1_points.shape[1])
    bin1_cloud.mutable_xyzs()[:] = bin1_points

    # Run ICP
    initial_guess_bin1 = RigidTransform(
        RotationMatrix.MakeZRotation(0),  
        [ 0.25, -1.5, 0.0 ]      
    )  # rough initial position
    bin1_X_Ohat, error_bin1 = IterativeClosestPoint(
        p_Om=bin1_points,
        p_Ws=merged_bin1_pc.xyzs(),
        X_Ohat=initial_guess_bin1,
        max_iterations=MAX_ITERATIONS,
    )
    
    X_WB_bin_red = bin1_X_Ohat

    ################## Bin pose ###################

    # sdf_path_bin = Path("sdfs/bin.sdf")
    # obj_path_bin = (sdf_path_bin.parent / "bin.obj")
    # bin1_mesh = trimesh.load(obj_path_bin.as_posix(), force='mesh')

    # # bin1_mesh = trimesh.load("sdfs/construction_cone.obj", force='mesh')
    # bin1_points = bin1_mesh.sample(N_SAMPLE_POINTS).T
    # bin1_cloud = PointCloud(bin1_points.shape[1])
    # bin1_cloud.mutable_xyzs()[:] = bin1_points

    # # Run ICP
    # initial_guess_bin1 = RigidTransform(
    #     RotationMatrix.MakeXRotation(90.0),  
    #     [ 0.0, 1.0, 0.0 ]      
    # )  # rough initial position
    # bin1_X_Ohat, error_bin1 = IterativeClosestPoint(
    #     p_Om=bin1_points,
    #     p_Ws=merged_bin1_pc.xyzs(),
    #     X_Ohat=initial_guess_bin1,
    #     max_iterations=MAX_ITERATIONS,
    # )
    
    # X_WB_bin_green = bin1_X_Ohat


    ################## Bin pose ###################
    model_bin1 = plant.GetModelInstanceByName("bin_blue")
    frame_bin1 = plant.GetFrameByName("bin_base", model_instance=model_bin1)
    X_PC_bin1 = plant.CalcRelativeTransform(plant_context, world_frame, frame_bin1)
    
    pad = np.array([.3, .4, .4])
    center_W = X_PC_bin1.translation()
    bin1_lower = center_W - pad
    bin1_upper = center_W + pad

    camera0_bin1_pc = diagram.GetOutputPort("camera0_point_cloud").Eval(context).Crop(bin1_lower, bin1_upper)
    camera1_bin1_pc = diagram.GetOutputPort("camera1_point_cloud").Eval(context).Crop(bin1_lower, bin1_upper)
    camera2_bin1_pc = diagram.GetOutputPort("camera2_point_cloud").Eval(context).Crop(bin1_lower, bin1_upper)
    camera3_bin1_pc = diagram.GetOutputPort("camera3_point_cloud").Eval(context).Crop(bin1_lower, bin1_upper)


    ####################
    merged_bin1_pc = Concatenate([camera0_bin1_pc, camera1_bin1_pc, camera2_bin1_pc, camera3_bin1_pc])
    merged_bin1_pc = merged_bin1_pc.VoxelizedDownSample(0.005)
    merged_bin1_pc = remove_table_points(merged_bin1_pc)
    # merged_bin1_pc = remove_non_back_points(merged_bin1_pc)
    
    # meshcat.SetObject(
    #     "bin1_point_cloud", merged_bin1_pc, point_size=0.05, rgba=Rgba(0, 0, 1)
    # )

    # p = Concatenate([diagram.GetOutputPort("camera0_point_cloud").Eval(context), diagram.GetOutputPort("camera1_point_cloud").Eval(context), diagram.GetOutputPort("camera2_point_cloud").Eval(context), diagram.GetOutputPort("camera3_point_cloud").Eval(context)])
    # meshcat.SetObject("p", p, point_size=0.01, rgba=Rgba(0,1,0))

    sdf_path_bin = Path("sdfs/bin.sdf")
    obj_path_bin = (sdf_path_bin.parent / "bin.obj")
    bin1_mesh = trimesh.load(obj_path_bin.as_posix(), force='mesh')

    # bin1_mesh = trimesh.load("sdfs/construction_cone.obj", force='mesh')
    bin1_points = bin1_mesh.sample(N_SAMPLE_POINTS).T
    bin1_cloud = PointCloud(bin1_points.shape[1])
    bin1_cloud.mutable_xyzs()[:] = bin1_points

    # Run ICP
    initial_guess_bin1 = RigidTransform(
        RotationMatrix.MakeZRotation(0),  
        [ 0.75, -1.5, 0.0 ]      
    )  # rough initial position
    bin1_X_Ohat, error_bin1 = IterativeClosestPoint(
        p_Om=bin1_points,
        p_Ws=merged_bin1_pc.xyzs(),
        X_Ohat=initial_guess_bin1,
        max_iterations=MAX_ITERATIONS,
    )
    
    X_WB_bin_blue = bin1_X_Ohat


    ball_color_vec = rgb_at_pose(ball_point_cloud, X_WB_ball, radius=0.05)
    # bin_color_vec = rgb_at_pose(merged_bin1_pc, X_WB_bin, radius=0.05)

    if ball_color_vec[0] > ball_color_vec[1] and ball_color_vec[0] > ball_color_vec[2]:
        ball_color = 'red'
        X_WB_bin = X_WB_bin_red
    else:
        ball_color = 'blue'
        X_WB_bin = X_WB_bin_blue

    print("WHAT IS YOUR POSITION??", X_WB_bin)

    # if bin_color_vec[0] > bin_color_vec[1] and bin_color_vec[0] > bin_color_vec[2]:
    #     bin_color = 'red'
    # elif bin_color_vec[1] > bin_color_vec[0] and bin_color_vec[1] > bin_color_vec[2]:
    #     bin_color = 'green'
    # else:
    #     bin_color = 'blue'


    return X_WB_ball, X_WB_bin#, ball_color, bin_color