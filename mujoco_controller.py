#!/bin/env python3

import mujoco
import mujoco_viewer
import os
import time
import rclpy
import rclpy.timer

#def cartpole_mpc(const mjModel* m, mjData* d):
    # d is cart_pos, pole_angle

    

def main():
    # Start our ROS node
    #rclpy.init()
    #node = rclpy.create_node("mujoco_controller_node")
    
    # Load a model (model is MJCF format)
    #model_path = os.path.expanduser("~/mujoco_menagerie/universal_robots_ur5e/ur5e.xml")
    model_path = os.path.expanduser("./cart_pole.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    # Data stores the state (time, generalized positions/vel's)
    data = mujoco.MjData(model)
    print(model)
    
    #rate = node.create_rate(1) # 1 Hz loop rate
    duration = 3.8 #s
    #framerate = 60 #Hz
    #frames = []
    mujoco.mj_resetData(model, data) # reset state and time
    #data.qpos[0] = 1.67
    data.qpos[1] = 1.67
    print(data.qpos)
    viewer = mujoco_viewer.MujocoViewer(model, data)
    time.sleep(0.5)
    #while data.time < duration:
    for _ in range(10000):
        if viewer.is_alive:
            mujoco.mj_step(model, data) # x_(t+h) = f(x_t)
            viewer.render()
        else:
            break
        #rate.sleep()
    
    viewer.close()

if __name__ == '__main__':
    main()
