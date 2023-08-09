import numpy as np
from pydrake.math import eq
from pydrake.multibody.plant import MultiBodyPlant
from pydrake.multibody.tree import SpatialInertia, PlanarJoint
from pydrake.solvers import MathematicalProgram, Solve
import matplotlib.pyplot as plt
import time

def main():
    plant = MultibodyPlant(time_step = 0)
    cart = plant.AddRigidBody(
        "Cart",
        SpatialInertia.SolidBoxWithMass(
            mass = 1, 
            lx = 1,
            ly = 0.5,
            lz = 1
        )
    )
    pole = plant.AddRigidBody(
        "Pole",
        SpatialInertia.SolidCylinderWithMass(
            mass = 0.1, 
            radius = 0.1,
            length = 1,
            unit_vector = [0, -1, 0]
        )
    )
    # Let cart slide around in xy axis
    plant.AddJoint(
        PrismaticJoint(
            "xy", 
            plant.world_frame(), 
            cart.body_frame(),
            [1, 0, 0]
        )
    )
    plant.AddJoint(
        RevoluteJoint(
            "pole_axis",
            cart.body_frame(),
            pole.body_frame(),
            [0, 0, 1]
        )
    )
    plant.Finalize()

    prog = MathematicalProgram()
    q = prog.NewContinuousVariables(N, plant.num_positions(), "q") # x (m)
    qd = prog.NewContinuousVariables(N, plant.num_velocities(), "qd") # v (m/s)
    qdd = prog.NewContinuousVariables(N, 1, "qdd") # a (m/s/s) (force is only applied to base of cart)

    # Position Constraints
    prog.AddBoundingBoxConstraint(-100, 100, q[:, 0])
    prog.AddBoundingBoxConstraint(-100, 100, q[:, 1])
    # Force Constraints
    prog.AddBoundingBoxConstraint(-10, 10, qdd)

if __name__ == '__main__':
    main()
