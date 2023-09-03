#!/bin/env python3 
import mujoco
#import mujoco.viewer
import mujoco_viewer
import os
import time
import numpy as np

from pydrake.math import eq
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.planning import DirectTranscription
from pydrake.solvers import MathematicalProgram, Solve
from pydrake.symbolic import sin, cos, pow

class mpc:
    def __init__(self, model):
        self.dt = 0.1
        self.N = 2
        self.max_acc = 1.0
        # [x, theta, xd, thetad]
        self.goal = np.zeros((4))
        self.goal[1] = np.pi # Want the pole to be straight up, still
        self.plant = MultibodyPlant(time_step = self.dt)
        # Todo: get the plant dynamics from the model
        self.parser = Parser(self.plant).AddModelsFromUrl(url="package://drake/examples/multibody/cart_pole/cart_pole.sdf")
        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()
        self.prog = MathematicalProgram()
        self.trajopt = DirectTranscription(
                      self.plant, 
                      self.context, 
                      self.N, 
                      input_port_index=self.plant.get_actuation_input_port().get_index(),
                 )

    def run_mpc(self, data):
        q_init = np.append(data.qpos, data.qvel)
        print(self.trajopt.initial_state())
        n_rows = self.trajopt.initial_state().shape
        print(n_rows)
        print(q_init)
        n_rows = q_init.shape
        print(n_rows)

        self.trajopt.prog().AddBoundingBoxConstraint(q_init, q_init, self.trajopt.initial_state())
        self.trajopt.prog().AddQuadraticErrorCost(100 * np.eye(4), self.goal, self.trajopt.final_state())
        sol = Solve(self.trajopt.prog())
        acc_opt = sol.GetSolution()
        print("Soln:", acc_opt)
        data.qfrc_applied = [0, acc_opt[-1]]

#def controller(model, data):
    #data.ctrl[0] = 0.5
    #data.qfrc_applied # specify teh generalized force applied to each joint
    #q = prog.NewContinuousVariables(N, plant.num_positions(), "q")
    #qd = prog.NewContinuousVariables(N, plant.num_velocities(), "qd")
    #qdd = prog.NewContinuousVariables(N, 1, "qdd")
    # Bounds on Acceleration according to our max
    #progAddBoundingBoxConstraint(-max_acc, max_acc, qdd)
    # Initial Conditions
    #prog.AddConstraint(eq(q[0, :], data.qpos))
    #prog.AddConstraint(eq(qd[0, :], data.qvel))
    #for i in range(1, N):
        # Dynamics Constraints
    #    prog.AddConstraint(eq(q[i], q[i-1] + qd[i-1] * dt))
    #    prog.AddConstraint(eq(qd[i], qd[i-1] + qdd[i-1] * dt))
        # Cost Function (mminimize goal position difference and final velocity)
    #    prog.addQuadraticErrorCost(100 * np.sum(pow(q[i, :], 2)) + np.sum(pow(qd[i, :], 2)))    # Dynamics Constraints
    #result = Solve(prog)
    #data.qfrc_applied = result.GetSolution(qdd)
#def mpc(model, data):
    # Traj Opt Solution
    #trajopt = DirectTranscription(plant, context, N, input_port_index=plant.get_actuation_input_port().get_index(),)
    #trajopt.prog().AddBoundingBoxConstraint(data.qpos, data.qpos, trajopt.initial_state())
    #trajopt.prog().AddBoundingQuadraticErrorCost(100 * np.eye(4), goal, trajopt.final_state())
    #sol = Solve(trajopt.prog())
    #acc_opt = sol.GetSolution(u)
    #data.qfrc_applied = acc_opt

    

model_path = os.path.expanduser("./cart_pole.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)
#print("Model:", model)
#print("Data:", data)

mujoco.mj_resetData(model, data)
#mujoco.mjcb_control = controller;
#mjcb_control = controller;
data.qpos[1] = 1.67
print("data ctrl:", data.ctrl)
myMPC = mpc(model)
mjcb_control = myMPC.run_mpc(data)

#viewer = mujoco.viewer.launch_passive(model, data)
viewer = mujoco_viewer.MujocoViewer(model, data)
time.sleep(0.5)
for _ in range(10000):
    #if viewer.is_running():
    if viewer.is_alive:
        mujoco.mj_step(model, data) # x_(t+h) = f(x_t)
        #viewer.sync()
        viewer.render()
    else:
        break

viewer.close()
