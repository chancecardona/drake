#!/bin/env python3

import numpy as np
from pydrake.math import eq
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import SpatialInertia, PlanarJoint, RevoluteJoint, PrismaticJoint
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
            "cart_slide", 
            plant.world_frame(), 
            cart.body_frame(),
            [1, 0, 0]
        )
    )
    plant.AddJoint(
        RevoluteJoint(
            "pole_to_cart",
            cart.body_frame(),
            pole.body_frame(),
            [0, 0, 1]
        )
    )
    plant.Finalize()

    prog = MathematicalProgram()
    N = 100
    q = prog.NewContinuousVariables(N, plant.num_positions(), "q") # x (m)
    qd = prog.NewContinuousVariables(N, plant.num_velocities(), "qd") # v (m/s)
    qdd = prog.NewContinuousVariables(N, 1, "qdd") # a (m/s/s) (force is only applied to base of cart)

    # Position Constraints
    #prog.AddBoundingBoxConstraint(-100, 100, q[:, 0])
    #prog.AddBoundingBoxConstraint(-100, 100, q[:, 1])
    # Force Constraints
    prog.AddBoundingBoxConstraint(-10, 10, qdd)

    # Dynamic Constraints
    #q[:, 0] is x
    #q[:, 1] is theta
    for i in range(1, N):
        # x dot (velocity)
        prog.AddConstraint(
            eq(q[i], 
               q[i-1] + qd[i-1] * dt
            )
        )
        # x dot dot
        prog.AddConstraint(
            eq(qd[i][0], 
               qd[i-1][0] + 1 / (cart.get_mass() + pole.get_mass() * math.pow(math.sin(q[i-1][1], 2))) * 
                 (qdd[i-1][0] + pole.get_mass() * math.sin(q[i-1][1]) * (pole.get_length() * math.pow(qd[i-1][1], 2) + g * math.cos(q[i-1][1])))
            )
        )
        # theta dot dot
        prog.AddConstraint(
            eq(qd[i][1], 
               qd[i-1][1] + 1 / (pole.get_length() * (cart.get_mass() + pole.get_mass() * math.pow(math.sin(q[i-1][1]), 2)) ) *
                 (-qdd[i-1][0] * math.cos(q[i-1][1]) - pole.get_mass() * pole.get_length() * math.pow(qd[i-1][1], 2) * math.cos(q[i-1][1]) * math.sin(q[i-1][1]) - (cart.get_mass() + pole.get_mass()) * g * math.sin(q[i-1][1]))
            )
        )
    print(q)

if __name__ == '__main__':
    main()
