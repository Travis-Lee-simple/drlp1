import time
import numpy as np
from libs import robot_sim
import scipy.misc

sleep_time = 2

# Set the workspace limits in simulator.
workspace_limits = np.asarray([[-0.724, -0.276], [-0.224, 0.224], [-0.0001, 0.4]])
time.sleep(3)

# Create the robot object, and start the simulation.
robot = robot_sim.SimRobot(workspace_limits)
time.sleep(sleep_time)

# Move to the target position.
pos1 = [-0.608, 0.034, 0.199]
robot.move_to(pos1)

# Get the camera data.
s_rgb, s_depth = robot.get_camera_data(robot.static_cam_handle)
g_rgb, g_depth = robot.get_camera_data(robot.gripper_cam_handle)
scipy.misc.imsave('./sRGB1.jpg', s_rgb)
scipy.misc.imsave('./gRGB1.jpg', g_rgb)

# Transfer the depth data into point cloud data.
s_pts = robot.get_pointcloud(s_depth, robot.cam_intrinsics)
g_pts = robot.get_pointcloud(g_depth, robot.cam_intrinsics)

# Save the depth data as point cloud file.
robot.pclwriter_ply('./s_pts.ply', s_pts)
robot.pclwriter_txt('./s_pts.txt', s_pts)
robot.pclwriter_ply('./g_pts.ply', g_pts)
robot.pclwriter_txt('./g_pts.txt', g_pts)

# Visualization of the depth image data
sdmax, sdmin = np.max(s_depth), np.min(s_depth)
gdmax, gdmin = np.max(g_depth), np.min(g_depth)
sdvis = (s_depth - sdmin) / (sdmax - sdmin)
gdvis = (g_depth - gdmin) / (gdmax - gdmin)
sdvis = sdvis * 255
gdvis = gdvis * 255
sdvis.astype(int)
gdvis.astype(int)
scipy.misc.imsave('./sD1.jpg', sdvis)
scipy.misc.imsave('./gD1.jpg', gdvis)

time.sleep(sleep_time)

# Rotate the gripper to the target angle(deg).
angle_alpha = 15.585
angle_beta = 11.111
angle_gamma = 170
robot.rotate_gripper(rotate_angle_alpha=angle_alpha, rotate_angle_beta=angle_beta, rotate_angle_gamma=angle_gamma)

# Open the gripper.
robot.open_gripper()
time.sleep(sleep_time)

# Move to the target position.
pos2 = [-0.608, 0.034, 0.0509]
robot.move_to(pos2)

# Close the gripper.
robot.close_gripper()
time.sleep(sleep_time)

# Rotate the gripper to the target angle(deg).
angle_alpha = 0
angle_beta = 0
angle_gamma = 0
robot.rotate_gripper(rotate_angle_alpha=angle_alpha, rotate_angle_beta=angle_beta, rotate_angle_gamma=angle_gamma)
time.sleep(sleep_time)

# Move to the target position.
pos3 = [-0.608, 0.034, 0.3]
robot.move_to(pos3)

# Stop the simulation.
robot.close_simulation()
print('Done!')
