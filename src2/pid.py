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
    InverseKinematics,
    Trajectory,
    PiecewisePose,
    Meshcat,
    ge,
    le,
    Solve,
    Quaternion
)

class PDController(LeafSystem):
    """PD controller for the IIWA robot with time-varying q_desired."""

    def __init__(self, kp: float, kd: float) -> None:
        super().__init__()

        # Measured [q; qdot]
        self.state_port = self.DeclareVectorInputPort("iiwa_state", 14)
        # Desired [q_des; qdot_des] OR just q_des (I'll use just q_des here)
        self.q_des_port = self.DeclareVectorInputPort("q_desired", 7)

        self.output_port = self.DeclareVectorOutputPort(
            "iiwa_torque", 7, self.ComputeTorque
        )

        self.kp = kp
        self.kd = kd

    def ComputeTorque(self, context: Context, output: BasicVector) -> None:
        x = self.state_port.Eval(context)
        q = x[:7]
        qdot = x[7:]

        q_des = self.q_des_port.Eval(context)
        qdot_des = np.zeros(7)  # still zero if you want simple PD

        position_error = q_des - q
        velocity_error = qdot_des - qdot

        torque = self.kp * position_error + self.kd * velocity_error
        output.set_value(torque)
