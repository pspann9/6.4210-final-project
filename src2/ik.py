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

class DifferentialIKSystem(LeafSystem):
    """Wrapper system for Differential IK.
    @param plant MultibodyPlant of the simulated plant.
    @param diffik_fun function object that handles diffik. Must have the signature
           diffik_fun(J_G, V_G_desired, q_now, v_now, X_now)
    """

    def __init__(self, plant: MultibodyPlant, diffik_fun: Callable) -> None:
        self.val = True
        LeafSystem.__init__(self)
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        self._iiwa = plant.GetModelInstanceByName("iiwa")
        self._G = plant.GetBodyByName("body").body_frame()
        self._W = plant.world_frame()
        self._diffik_fun = diffik_fun

        self.DeclareVectorInputPort("spatial_velocity", 6)
        self.DeclareVectorInputPort("iiwa.position_measured", 7)
        self.DeclareVectorInputPort("iiwa.velocity_measured", 7)
        self.DeclareVectorOutputPort("iiwa_velocity_command", 7, self.CalcOutput)

    def CalcOutput(self, context: Context, output: BasicVector) -> None:
        V_G_desired = self.get_input_port(0).Eval(context)
        q_now = self.get_input_port(1).Eval(context)
        v_now = self.get_input_port(2).Eval(context)

        self._plant.SetPositions(self._plant_context, self._iiwa, q_now)
        J_G = self._plant.CalcJacobianSpatialVelocity(
            self._plant_context,
            JacobianWrtVariable.kQDot,
            self._G,
            [0, 0, 0],
            self._W,
            self._W,
        )
        J_G = J_G[:, 0:7]  # Ignore gripper terms

        X_now = self._plant.CalcRelativeTransform(self._plant_context, self._W, self._G)
        p_now = X_now.translation()

        # You will be defining different versions of the diffik_fn in this problem.
        v = self._diffik_fun(J_G, V_G_desired, q_now, v_now, p_now)
        output.SetFromVector(v)

def BuildAndSimulate(
    diffik_fun: Callable,
    V_d: np.ndarray,
    duration: float,
    plot_system_diagram: bool = False,
) -> None:
    builder = DiagramBuilder()

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
                translation: [5, 0, 3.048]
                rotation: !Rpy { deg: [0, 0, 180] }

        - add_model:
            name: ball
            file: package://drake_models/manipulation_station/sphere.sdf

        - add_weld:
            parent: world
            child: ball::base_link
            X_PC:
                translation: [0.5, 0, 0]
                rotation: !Rpy { deg: [0, 0, 0] }

        model_drivers:
            iiwa: !IiwaDriver
                hand_model_name: wsg
            wsg: !SchunkWsgDriver {}
    """

    scenario = LoadScenario(data=scenario_data)
    station = builder.AddSystem(MakeHardwareStation(scenario, meshcat=meshcat))
    plant = station.GetSubsystemByName("plant")

    controller = builder.AddSystem(DifferentialIKSystem(plant, diffik_fun))
    integrator = builder.AddSystem(Integrator(7))
    desired_vel = builder.AddSystem(ConstantVectorSource(V_d))

    builder.Connect(controller.get_output_port(), integrator.get_input_port())
    builder.Connect(integrator.get_output_port(), station.GetInputPort("iiwa.position"))
    builder.Connect(
        station.GetOutputPort("iiwa.position_measured"),
        controller.get_input_port(1),
    )
    builder.Connect(
        station.GetOutputPort("iiwa.velocity_estimated"),
        controller.get_input_port(2),
    )
    builder.Connect(desired_vel.get_output_port(), controller.get_input_port(0))

    diagram = builder.Build()
    diagram.set_name("diagram")
    if running_as_notebook and plot_system_diagram:
        RenderDiagram(diagram)

    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    station_context = station.GetMyContextFromRoot(context)
    station.GetInputPort("iiwa.torque").FixValue(station_context, np.zeros((7, 1)))
    station.GetInputPort("wsg.position").FixValue(station_context, [0.1])

    integrator.set_integral_value(
        integrator.GetMyMutableContextFromRoot(context),
        plant.GetPositions(
            plant.GetMyContextFromRoot(context),
            plant.GetModelInstanceByName("iiwa"),
        ),
    )

    meshcat.StartRecording()
    simulator.AdvanceTo(duration)
    meshcat.PublishRecording()
    
def DiffIKQP(
    J_G: np.ndarray,
    V_G_desired: np.ndarray,
    q_now: np.ndarray,
    v_now: np.ndarray,
    p_now: np.ndarray,
) -> np.ndarray:
    prog = MathematicalProgram()
    v = prog.NewContinuousVariables(7, "v")
    v_max = 3.0  # do not modify

    # TODO: Add cost and constraints to prog here.
    prog.AddCost((J_G @ v - V_G_desired).dot(J_G @ v - V_G_desired))
    prog.AddConstraint(le(v, v_max))
    prog.AddConstraint(ge(v, -v_max))

    solver = SnoptSolver()
    result = solver.Solve(prog)
    if not (result.is_success()):
        raise ValueError("Could not find the optimal solution.")

    v_solution = result.GetSolution(v)

    return v_solution

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
        
        # Allows us to only use certain joints at different phases
        self.mask_align = np.array([0, 0, 0, 1, 1, 1, 1]) 
        self.mask_throw = np.array([1, 1, 1, 0, 0, 0, 0])

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
        
        t = context.get_time()
        if t < 6.0:
            mask = np.array([1, 1, 1, 1, 1, 1, 1])
        else:
            mask = np.array([0, 0, 1, 1, 1, 1, 1])

        v = v * mask

        output.SetFromVector(v)