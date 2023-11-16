#!/bin/env python3 

# Thank you Kyle C. for the drake guidance and mujoco guruship.
import mujoco as mj
import mujoco.viewer as mjv
import numpy as np
import os
import time

from pydrake.math import eq, RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import FixedOffsetFrame
from pydrake.planning import DirectTranscription
from pydrake.solvers import MathematicalProgram, Solve
#from pydrake.symbolic import sin, cos, pow

# Init
dt = 0.05
N = 15 # Horizon samples to calculate (t_f = N * time_step)
# [x, theta, xd, thetad]
x_goal = np.zeros((4))
x_goal[1] = np.pi # Want the pole to be straight up, still

Q = 0.01 * np.diag([0.0, 0.0, 1.0, 1.0])
R = 0.01 * np.eye(1)
Qf = 1000.0 * np.eye(4)

# Solution
cart_pole = MultibodyPlant(time_step = dt)
parser = Parser(cart_pole).AddModelFromFile("cart_pole.urdf")
cart_pole.AddFrame(
    FixedOffsetFrame(
        "ee_frame", cart_pole.GetBodyByName("pole"), RigidTransform(p=[0.0, 0.0, -0.5])
    )
)
cart_pole.Finalize()
context = cart_pole.CreateDefaultContext()

def solve_mpc(model, x_init):
    trajopt = DirectTranscription(
        model,
        context,
        N,
        input_port_index=model.get_actuation_input_port().get_index(),
    )

    trajopt.prog().AddConstraint(eq(x_init, trajopt.initial_state()))

    u = trajopt.input()
    x = trajopt.state()
    x_err = x - x_goal
    trajopt.AddRunningCost(x_err.T @ Q @ x_err + u.T @ R @ u)
    trajopt.AddFinalCost(x_err.T @ Qf @ x_err)

    soln = Solve(trajopt.prog())
    t_star = trajopt.GetSampleTimes(soln)
    x_star = trajopt.GetStateSamples(soln)
    u_star = trajopt.GetInputSamples(soln)

    return t_star, x_star, u_star

    #self.prog = MathematicalProgram()
    #self.q = self.prog.NewContinuousVariables(self.N, self.plant.num_positions(), "q") # x (m)
    #self.qd = self.prog.NewContinuousVariables(self.N, self.plant.num_velocities(), "qd") # v (m/s)
    #self.qdd = self.prog.NewContinuousVariables(self.N, 1, "qdd") # a (m/s/s) (force is only applied to base of cart)
    ## Force Constraints
    #self.prog.AddBoundingBoxConstraint(-self.max_acc, self.max_acc, self.qdd)
    ## Initial Condition Constraints
    #self.prog.AddConstraint(eq(self.q[0], data.qpos))
    #self.prog.AddConstraint(eq(self.qd[0], data.qvel))

    # Dynamic Constraints
    # x is q[:, 0]
    # theta is q[:, 1]
    #for i in range(1, self.N):
    #    # x dot (velocity)
    #    self.prog.AddConstraint(
    #        eq(self.q[i], 
    #           self.q[i-1] + self.qd[i-1] * self.dt
    #        )
    #    )
    #    # x dot dot
    #    self.prog.AddConstraint(
    #        eq(self.qd[i][0],
    #           (self.qd[i-1][0] + (self.qdd[i-1] + self.pole_mass * sin(self.q[i-1][1]) * (self.pole_length * pow(self.qd[i-1][1], 2) + self.g * cos(self.q[i-1][1]))) / (self.cart_mass + self.pole_mass * pow(sin(self.q[i-1][1]), 2)) * self.dt)
    #        )
    #    )
    #    # theta dot dot
    #    self.prog.AddConstraint(
    #        eq(self.qd[i][1], 
    #           (self.qd[i-1][1] + (-self.qdd[i-1] * cos(self.q[i-1][1]) - self.pole_mass * self.pole_length * pow(self.qd[i-1][1], 2) * cos(self.q[i-1][1]) * sin(self.q[i-1][1]) - (self.cart_mass + self.pole_mass) * self.g * sin(self.q[i-1][1])) / (self.pole_length * (self.cart_mass + self.pole_mass * pow(sin(self.q[i-1][1]), 2))) * self.dt)
    #        )
    #    )

    ## Goal cost
    #for i in range(self.N):
    #    w_angle = 100.0
    #    w_cartpos = 40.0
    #    w_velocity = 0.01
    #    self.prog.AddQuadraticErrorCost(np.diag([w_cartpos, w_angle]), self.goal[0:2], self.q[i, :])
    #    self.prog.AddQuadraticErrorCost(w_velocity * np.eye(2), self.goal[2:], self.qd[i, :])

    #result = Solve(self.prog)
    #self.q_opt = result.GetSolution(self.q)
    #self.qd_opt = result.GetSolution(self.qd)
    #self.qdd_opt = result.GetSolution(self.qdd)
    #force = self.qdd_opt[0] * (self.pole_mass + self.cart_mass)
    #print("State:", data.qpos, data.qvel)
    #print("Applied:", force)
    #data.qfrc_applied[0] = force

def control_callback(model, data):
    x_init = [data.qpos[0], data.qpos[1], data.qvel[0], data.qvel[1]]
    t, x, u = solve_mpc(cart_pole, x_init)
    data.qfrc_applied[0] = u[0, 0]

model = mj.MjModel.from_xml_path("cart_pole.urdf")
data = mj.MjData(model)
# Reset data just cause
#mj.mj_resetData(model, data)
# Set to what we want
#data.qpos[1] = 0
#print("data ctrl:", data.ctrl)
mj.set_mjcb_control(control_callback)

with mjv.launch_passive(model, data) as viewer:
    viewer.cam.distance = 5.0

    while viewer.is_running():
        step_start = time.time()
        # mj_step can be replaced with code that evals a policy and applies and 
        # control signal before stepping the physics.
        mj.mj_step(model, data)
        # Pick up changes to physics states, update options, apply perturbations from GUI.
        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

