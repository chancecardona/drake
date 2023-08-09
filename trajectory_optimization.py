#!/bin/env python3

from pydrake.math import eq
from pydrake.solvers import MathematicalProgram, Solve
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import SpatialInertia, PlanarJoint
import numpy as np

plant = MultibodyPlant(time_step=0)
# Add a "brick" on some "ice" with thrusters.
brick = plant.AddRigidBody("brick", SpatialInertia.SolidSphereWithMass(mass=1.0, radius=0.25))
# Just the rigid body would have 7 position DOF (3 translation, 4 orientation). Need a joint to reduce it to 3.
plant.AddJoint(PlanarJoint("joint", plant.world_frame(), brick.body_frame()))

plant.Finalize()

ctx = plant.CreateDefaultContext()

# Num positions is the DOF.
print("Position DOF: ", plant.num_positions())
print("Velocity DOF: ", plant.num_velocities())

# Time to do solving.
def solve_mpc(q0, qd0):
    N = 500
    prog = MathematicalProgram()
    # First set up variables we want to solve for (q like in lagrangian mechanics)
    # Create Nx3 matrix q. Each point is a (x,y,theta) coord, but we only need (x,y)
    q = prog.NewContinuousVariables(N, plant.num_positions(), "q")
    qd = prog.NewContinuousVariables(N, plant.num_positions(), "qd")
    qdd = prog.NewContinuousVariables(N, plant.num_positions(), "qdd")
    
    # Now need constraints and boundaries
    prog.AddBoundingBoxConstraint(-0.5, 0.5, qdd) # Acceleration constraints
    prog.AddBoundingBoxConstraint(-1, 1, qd) # Velocity constraints
    # Initial conditions (should just start at origin which is robot should start)
    prog.AddConstraint(eq(q[0], [0.0, 0.0, 0.0])) 
    prog.AddConstraint(eq(qd[0], [0.0, 0.0, 0.0])) 
    # Dynamic constraints (need to relate our variables via the laws of physics)
    # min (1/2)* xQx' + bx
    #    st.: Aeq * x = beq, Ain * x >= bin
    dt = 0.01
    for i in range(1, N):
        # Basic euler method
        prog.AddConstraint(eq(q[i], q[i-1] + qd[i - 1] * dt))
        prog.AddConstraint(eq(qd[i], qd[i-1] + qdd[i - 1] * dt))
    
    # Now Add costs (that we want to minimize)
    # Q * (x - x_target) ?
    # Configure weights for which we favor
    w_pos = 100.0
    w_vel = 1.0
    prog.AddQuadraticErrorCost(w_pos * np.eye(3), [2.0, 2.0, 0.0], q[-1, :])
    prog.AddQuadraticErrorCost(w_vel * np.eye(3), [0.0, 0.0, 0.0], qd[-1, :])
    
    result = Solve(prog)

while True: 
    result = solve_mpc(0, 0)
    print(result.is_success())
    print(result.get_solver_id().name())
    print(result.get_optimal_cost())
    print(result.get_solver_details().run_time)

    # Now print
    import matplotlib.pyplot as plt
    q_opt = result.GetSolution(q)
    plt.plot(q_opt[:, 0], q_opt[:, 1])
    plt.show()
