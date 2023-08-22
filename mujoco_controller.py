#!/bin/env python3

import mujoco
import mujoco.viewer as viewer
import os
import time
import rclpy
import rclpy.timer

# Start our ROS node
rclpy.init()
node = rclpy.create_node("mujoco_controller_node")

# Load a model (model is MJCF format)
#model_path = os.path.expanduser("~/mujoco_menagerie/universal_robots_ur5e/ur5e.xml")
model_path = os.path.expanduser("./cart_pole.xml")
model = mujoco.MjModel.from_xml_path(model_path)
# Data stores the state (time, generalized positions/vel's)
data = mujoco.MjData(model)

passive_viewer = viewer.launch_passive(model, data)

rate = node.create_rate(1) # 1 Hz loop rate

# Start initial positions
print(data.qpos)
data.qpos

while passive_viewer.is_running():
    # Evaluate the control policy and apply a signal, then step the physics
    mujoco.mj_step(model, data)

    #with passive_viewer.lock():
    #    passive_viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONSTACTPOINT] = int(d.time % 2)
    
    # Sync physics changes to viewer
    passive_viewer.sync()

    data.qpos[0] += 1
    print(data.qpos)

    rate.sleep()
