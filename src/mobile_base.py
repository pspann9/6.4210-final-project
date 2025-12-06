from pydrake.all import (
    DiagramBuilder, Simulator, StartMeshcat,
    RigidTransform, RollPitchYaw
)
from manipulation.station import LoadScenario, MakeHardwareStation
import numpy as np

# ----------------------------
# Start Meshcat
# ----------------------------
meshcat = StartMeshcat()

# ----------------------------
# Scenario data
# ----------------------------
scenario_data = """
directives:
    - add_model:
        name: table
        file: file:///workspaces/6.4210-final-project/sdfs/table.sdf
    - add_weld:
        parent: world
        child: table::table_link
        X_PC:
            translation: [0.0, 0.0, -0.05]
            rotation: !Rpy { deg: [0, 0, -90] }

    - add_model:
        name: mobile_base
        file: file:///workspaces/6.4210-final-project/sdfs/mobile_base.sdf

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

    - add_weld:
        parent: mobile_base::chassis
        child:  iiwa::iiwa_link_0

    - add_model:
        name: wsg
        file: package://manipulation/hydro/schunk_wsg_50_with_tip.sdf
    
    - add_weld:
        parent: iiwa::iiwa_link_7
        child: wsg::body
        X_PC:
            translation: [0, 0, 0.09]
            rotation: !Rpy { deg: [90, 0, 90]}
    
    - add_model:
        name: obstacle_back
        file: file:///workspaces/6.4210-final-project/sdfs/obstacle.sdf

    - add_weld:
        parent: world
        child: obstacle_back::link  
        X_PC:
            translation: [0, 0.65, 0]
            rotation: !Rpy { deg: [90, 0, 0] }

    - add_model:
        name: obstacle_front
        file: file:///workspaces/6.4210-final-project/sdfs/obstacle.sdf

    - add_weld:
        parent: world
        child: obstacle_front::link  
        X_PC:
            translation: [0, -0.65, 0]
            rotation: !Rpy { deg: [90, 0, 0] }

    - add_model:
        name: obstacle_left
        file: file:///workspaces/6.4210-final-project/sdfs/obstacle.sdf

    - add_weld:
        parent: world
        child: obstacle_left::link  
        X_PC:
            translation: [0.65, 0, 0]
            rotation: !Rpy { deg: [90, 0, 0] }

    # - add_model:
    #     name: obstacle_right
    #     file: file:///workspaces/6.4210-final-project/sdfs/obstacle.sdf

    # - add_weld:
    #     parent: world
    #     child: obstacle_right::link  
    #     X_PC:
    #         translation: [-0.65, 0, 0]
    #         rotation: !Rpy { deg: [90, 0, 0] }

# model_drivers:
#     iiwa: !IiwaDriver
#         hand_model_name: wsg
#     wsg: !SchunkWsgDriver {}
"""

# ----------------------------
# Build Diagram + Station
# ----------------------------
scenario = LoadScenario(data=scenario_data)
builder = DiagramBuilder()
station = MakeHardwareStation(scenario, meshcat=meshcat)
builder.AddSystem(station)

plant = station.GetSubsystemByName("plant")
base_model = plant.GetModelInstanceByName("mobile_base")


# Build diagram and simulator
diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
context = simulator.get_mutable_context()

# ----------------------------
# Teleport function
# ----------------------------
def teleport_robot(plant, station, context, base_model, position, yaw=0.0):
    """
    Teleports the mobile_base to a new pose.
    """
    # Get the plant subsystem context
    plant_context = plant.GetMyContextFromRoot(context)
    
    # Transform for chassis
    X_WB = RigidTransform(RollPitchYaw(0, 0, yaw), position)
    
    # Set the chassis pose
    plant.SetFreeBodyPose(
        plant_context,
        plant.GetBodyByName("chassis", base_model),
        X_WB
    )

forward = False
backward = False
right = True
left = False

if forward:
    start_pos = np.array([0.0, 0.0, 0.1])
    step_size = 0.005   # meters per step
    yaw = -1.5708          # no rotation

    # forward
    # Number of steps
    end_y = -1.5
    n_steps = int(-end_y / step_size) + 1

    # Build the pose list
    poses = []
    for i in range(n_steps):
        y = start_pos[1] - i * step_size
        poses.append(([start_pos[0], y, start_pos[2]], yaw))
    for i in range(n_steps // 2):
        poses.append(([start_pos[0], end_y, start_pos[2]], yaw))
elif backward:
    start_pos = np.array([0.0, 0.0, 0.1])
    step_size = 0.005   # meters per step
    yaw = -1.5708          # no rotation

    # backward
    # Number of steps
    end_y = 1.5
    n_steps = int(end_y / step_size) + 1

    # Build the pose list
    poses = []
    for i in range(n_steps):
        y = start_pos[1] + i * step_size
        poses.append(([start_pos[0], y, start_pos[2]], yaw))
    for i in range(n_steps // 2):
        poses.append(([start_pos[0], end_y, start_pos[2]], yaw))
elif right:
    start_pos = np.array([0.0, 0.0, 0.1])
    step_size = 0.005   # meters per step
    start_yaw = -1.5708          # no rotation
    

    # backward
    # Number of steps
    end_yaw = -3.1416
    n_steps = int(-(end_yaw - start_yaw) / step_size) + 1

    end_x = -1.5
    n_steps_x = int(-end_x / step_size) + 1

    end_yaw2 = -1.5708 + 0.291456794
    n_steps2 = int((end_yaw2 - end_yaw) / step_size) + 1

    # Build the pose list
    poses = []
    for i in range(n_steps):
        yaw = start_yaw - i * step_size
        poses.append(([start_pos[0], start_pos[1], start_pos[2]], yaw))
    for i in range(n_steps // 2):
        poses.append(([start_pos[0], start_pos[0], start_pos[2]], end_yaw))
    for i in range(n_steps_x):
        x = start_pos[0] - i * step_size
        poses.append(([x, start_pos[1], start_pos[2]], end_yaw))
    for i in range(n_steps2):
        yaw = end_yaw + i * step_size
        poses.append(([end_x, start_pos[1], start_pos[2]], yaw))
    for i in range(n_steps2 // 2):
        poses.append(([end_x, start_pos[1], start_pos[2]], end_yaw2))


elif left:
    start_pos = np.array([0.0, 0.0, 0.1])
    step_size = 0.005   # meters per step
    start_yaw = -1.5708          # no rotation

    # backward
    # Number of steps
    end_yaw = 0
    n_steps = int((end_yaw - start_yaw) / step_size) + 1

    end_x = 1.5
    n_steps_x = int(end_x / step_size) + 1

    end_yaw2 = -1.5708 - 0.291456794
    n_steps2 = int(-(end_yaw2 - end_yaw) / step_size) + 1

    # Build the pose list
    poses = []
    for i in range(n_steps):
        yaw = start_yaw + i * step_size
        poses.append(([start_pos[0], start_pos[1], start_pos[2]], yaw))
    for i in range(n_steps // 2):
        poses.append(([start_pos[0], start_pos[0], start_pos[2]], end_yaw))
    for i in range(n_steps_x):
        x = start_pos[0] + i * step_size
        poses.append(([x, start_pos[1], start_pos[2]], end_yaw))
    for i in range(n_steps2):
        yaw = end_yaw - i * step_size
        poses.append(([end_x, start_pos[1], start_pos[2]], yaw))
    for i in range(n_steps2 // 2):
        poses.append(([end_x, start_pos[1], start_pos[2]], end_yaw2))




dt = 0.005
sim_time = 0.0

meshcat.StartRecording()

for pos, yaw in poses:
    t_end = sim_time + .01  # advance 0.1 s per pose for visible motion
    while sim_time < t_end:
        teleport_robot(plant, station, context, base_model, pos, yaw)
        simulator.AdvanceTo(sim_time + dt)
        sim_time += dt

meshcat.StopRecording()
meshcat.PublishRecording()
