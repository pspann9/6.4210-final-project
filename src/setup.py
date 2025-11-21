

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