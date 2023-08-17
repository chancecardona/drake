#!/bin/env python3

import numpy as np
from pydrake.math import eq
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import SpatialInertia, PlanarJoint, RevoluteJoint, PrismaticJoint
from pydrake.solvers import MathematicalProgram, Solve
from pydrake.symbolic import sin, cos, pow
import matplotlib.pyplot as plt
import time

def main():
    dt = 0.01
    N = 100
    pole_length = 1
    cart_mass = 1
    pole_mass = 0.1
    g = 9.81

    plant = MultibodyPlant(time_step = 0)
    cart = plant.AddRigidBody(
        "Cart",
        SpatialInertia.SolidBoxWithMass(
            mass = cart_mass, 
            lx = 1,
            ly = 0.5,
            lz = 1
        )
    )
    pole = plant.AddRigidBody(
        "Pole",
        SpatialInertia.SolidCylinderWithMass(
            mass = pole_mass, 
            radius = 0.1,
            length = pole_length,
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
    q = prog.NewContinuousVariables(N, plant.num_positions(), "q") # x (m)
    qd = prog.NewContinuousVariables(N, plant.num_velocities(), "qd") # v (m/s)
    qdd = prog.NewContinuousVariables(N, 1, "qdd") # a (m/s/s) (force is only applied to base of cart)

    # Position Constraints
    #prog.AddBoundingBoxConstraint(-100, 100, q[:, 0])
    #prog.AddBoundingBoxConstraint(-100, 100, q[:, 1])

    # Force Constraints
    prog.AddBoundingBoxConstraint(-100, 100, qdd)

    # Initial Condition Constraints
    ICs = np.zeros((2,2))
    ICs[0][1] = np.pi
    prog.AddConstraint(eq(q[0], ICs[0, :]))
    prog.AddConstraint(eq(qd[0], ICs[1, :]))

    # Dynamic Constraints
    # x is q[:, 0]
    # theta is q[:, 1]
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
               (qd[i-1][0] + (qdd[i-1] + pole_mass * sin(q[i-1][1]) * (pole_length * pow(qd[i-1][1], 2) + g * cos(q[i-1][1]))) / (cart_mass + pole_mass * pow(sin(q[i-1][1]), 2)) * dt)
            )
        )
        # theta dot dot
        prog.AddConstraint(
            eq(qd[i][1], 
               (qd[i-1][1] + (-qdd[i-1] * cos(q[i-1][1]) - pole_mass * pole_length * pow(qd[i-1][1], 2) * cos(q[i-1][1]) * sin(q[i-1][1]) - (cart_mass + pole_mass) * g * sin(q[i-1][1])) / (pole_length * (cart_mass + pole_mass * pow(sin(q[i-1][1]), 2))) * dt)
            )
        )

    # Goal cost
    goal = np.zeros((2,2))
    for i in range(N):
        w_angle = 100.0
        w_cartpos = 10.0
        w_velocity = 0.01
        prog.AddQuadraticErrorCost(np.diag([w_cartpos, w_angle]), goal[0, :], q[i, :])
        prog.AddQuadraticErrorCost(w_velocity * np.eye(2), goal[0, :], qd[i, :])

    result = Solve(prog)
    q_opt = result.GetSolution(q)
    qd_opt = result.GetSolution(qd)
    qdd_opt = result.GetSolution(qdd)

    # Visualize
    pos_figure = plt.figure()
    plt.plot(q_opt[:, 0])
    plt.title("Cart Position")
    angle_figure = plt.figure()
    plt.plot(q_opt[:, 1])
    plt.title("Angle")
    force_figure = plt.figure()
    plt.plot(qdd_opt)
    plt.title("Force")
    xy_figure = plt.figure()
    plt.plot(np.sin(q_opt[:,1]) + q_opt[:, 0], np.cos(q_opt[:,1]))
    plt.title("Pole y vs x position")
    plt.show()

if __name__ == '__main__':
    main()
