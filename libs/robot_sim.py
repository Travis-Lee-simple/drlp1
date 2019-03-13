import time
import struct
import numpy as np
from . import vrep
import math
import scipy.misc

class SimRobot():
    def __init__(self, workspace_limits):
        self.workspace_limits = workspace_limits
        vrep.simxFinish(-1)
        self.sim_client = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        if self.sim_client == -1:
            print('Failed to connect to simulation (V-REP remote API server). Exiting.')
            exit()
        else:
            print('Connected to simulation.')
            self.restart_sim()
            self.setup_sim_cameras()
            self.check_sim()

    def restart_sim(self):
        sim_ret, self.UR5_target_handle = vrep.simxGetObjectHandle(self.sim_client,'UR5_target',vrep.simx_opmode_blocking)
        vrep.simxSetObjectPosition(self.sim_client, self.UR5_target_handle, -1, (-0.5,0,0.3), vrep.simx_opmode_blocking)
        vrep.simxStopSimulation(self.sim_client, vrep.simx_opmode_blocking)
        vrep.simxStartSimulation(self.sim_client, vrep.simx_opmode_blocking)
        time.sleep(1)
        sim_ret, self.RG2_tip_handle = vrep.simxGetObjectHandle(self.sim_client, 'UR5_tip', vrep.simx_opmode_blocking)
        sim_ret, gripper_position = vrep.simxGetObjectPosition(self.sim_client, self.RG2_tip_handle, -1, vrep.simx_opmode_blocking)
        while gripper_position[2] > 0.4: # V-REP bug requiring multiple starts and stops to restart
            vrep.simxStopSimulation(self.sim_client, vrep.simx_opmode_blocking)
            vrep.simxStartSimulation(self.sim_client, vrep.simx_opmode_blocking)
            time.sleep(1)
            sim_ret, gripper_position = vrep.simxGetObjectPosition(self.sim_client, self.RG2_tip_handle, -1, vrep.simx_opmode_blocking)

    def check_sim(self):

        # Check if simulation is stable by checking if gripper is within workspace
        sim_ret, gripper_position = vrep.simxGetObjectPosition(self.sim_client, self.RG2_tip_handle, -1, vrep.simx_opmode_blocking)
        sim_ok = gripper_position[0] > self.workspace_limits[0][0] - 0.1 and gripper_position[0] < self.workspace_limits[0][1] + 0.1 and gripper_position[1] > self.workspace_limits[1][0] - 0.1 and gripper_position[1] < self.workspace_limits[1][1] + 0.1 and gripper_position[2] > self.workspace_limits[2][0] and gripper_position[2] < self.workspace_limits[2][1]
        if not sim_ok:
            print('Simulation unstable. Restarting environment.')
            self.restart_sim()
        else:
            print('sim_ok = ', sim_ok)

    def close_gripper(self):

        gripper_motor_velocity = -0.5
        gripper_motor_force = 100
        sim_ret, RG2_gripper_handle = vrep.simxGetObjectHandle(self.sim_client, 'RG2_openCloseJoint', vrep.simx_opmode_blocking)
        sim_ret, gripper_joint_position = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle, vrep.simx_opmode_blocking)
        vrep.simxSetJointForce(self.sim_client, RG2_gripper_handle, gripper_motor_force, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(self.sim_client, RG2_gripper_handle, gripper_motor_velocity, vrep.simx_opmode_blocking)
        gripper_fully_closed = False
        while gripper_joint_position > -0.047: # Block until gripper is fully closed
            sim_ret, new_gripper_joint_position = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle, vrep.simx_opmode_blocking)
            # print(gripper_joint_position)
            if new_gripper_joint_position >= gripper_joint_position:
                return gripper_fully_closed
            gripper_joint_position = new_gripper_joint_position
        gripper_fully_closed = True

        return gripper_fully_closed
    
    def open_gripper(self, async=False):

        gripper_motor_velocity = 0.5
        gripper_motor_force = 20
        sim_ret, RG2_gripper_handle = vrep.simxGetObjectHandle(self.sim_client, 'RG2_openCloseJoint', vrep.simx_opmode_blocking)
        sim_ret, gripper_joint_position = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle, vrep.simx_opmode_blocking)
        vrep.simxSetJointForce(self.sim_client, RG2_gripper_handle, gripper_motor_force, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(self.sim_client, RG2_gripper_handle, gripper_motor_velocity, vrep.simx_opmode_blocking)
        while gripper_joint_position < 0.0536: # Block until gripper is fully open
            sim_ret, gripper_joint_position = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle, vrep.simx_opmode_blocking)
    
    def move_to(self, tool_position):

        sim_ret, UR5_target_position = vrep.simxGetObjectPosition(self.sim_client, self.UR5_target_handle,-1,vrep.simx_opmode_blocking)

        move_direction = np.asarray([tool_position[0] - UR5_target_position[0], tool_position[1] - UR5_target_position[1], tool_position[2] - UR5_target_position[2]])
        move_magnitude = np.linalg.norm(move_direction)
        move_step = 0.02*move_direction/move_magnitude
        num_move_steps = int(np.floor(move_magnitude/0.02))

        for step_iter in range(num_move_steps):
            vrep.simxSetObjectPosition(self.sim_client,self.UR5_target_handle,-1,(UR5_target_position[0] + move_step[0], UR5_target_position[1] + move_step[1], UR5_target_position[2] + move_step[2]),vrep.simx_opmode_blocking)
            sim_ret, UR5_target_position = vrep.simxGetObjectPosition(self.sim_client,self.UR5_target_handle,-1,vrep.simx_opmode_blocking)
        vrep.simxSetObjectPosition(self.sim_client,self.UR5_target_handle,-1,(tool_position[0],tool_position[1],tool_position[2]),vrep.simx_opmode_blocking)

    def rotate_alpha(self, rotate_angle):
        rotate_angle = rotate_angle / 180.0
        rotate_angle = rotate_angle * np.pi
        tool_rotation_angle = (rotate_angle % np.pi)
        sim_ret, gripper_orientation = vrep.simxGetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, vrep.simx_opmode_blocking)
        rotation_step = 0.3 if (tool_rotation_angle - gripper_orientation[0] > 0) else -0.3
        num_rotation_steps = int(np.floor((tool_rotation_angle - gripper_orientation[0])/rotation_step))

        # Simultaneously move and rotate gripper
        for step_iter in range(num_rotation_steps):
            vrep.simxSetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, (gripper_orientation[0] + rotation_step*min(step_iter,num_rotation_steps), gripper_orientation[1], gripper_orientation[2]), vrep.simx_opmode_blocking)
        vrep.simxSetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, (tool_rotation_angle, gripper_orientation[1], gripper_orientation[2]), vrep.simx_opmode_blocking)
    
    def rotate_beta(self, rotate_angle):
        rotate_angle = rotate_angle / 180.0
        rotate_angle = rotate_angle * np.pi
        tool_rotation_angle = (rotate_angle % np.pi)
        sim_ret, gripper_orientation = vrep.simxGetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, vrep.simx_opmode_blocking)
        rotation_step = 0.3 if (tool_rotation_angle - gripper_orientation[1] > 0) else -0.3
        num_rotation_steps = int(np.floor((tool_rotation_angle - gripper_orientation[1])/rotation_step))

        # Simultaneously move and rotate gripper
        for step_iter in range(num_rotation_steps):
            vrep.simxSetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, (gripper_orientation[0], gripper_orientation[1] + rotation_step*min(step_iter,num_rotation_steps), gripper_orientation[2]), vrep.simx_opmode_blocking)
        vrep.simxSetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, (gripper_orientation[0], tool_rotation_angle, gripper_orientation[2]), vrep.simx_opmode_blocking)

    def rotate_gamma(self, rotate_angle):
        rotate_angle = rotate_angle / 180.0
        rotate_angle = rotate_angle * np.pi
        tool_rotation_angle = (rotate_angle % np.pi)
        sim_ret, gripper_orientation = vrep.simxGetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, vrep.simx_opmode_blocking)
        rotation_step = 0.3 if (tool_rotation_angle - gripper_orientation[2] > 0) else -0.3
        num_rotation_steps = int(np.floor((tool_rotation_angle - gripper_orientation[2])/rotation_step))

        # Simultaneously move and rotate gripper
        for step_iter in range(num_rotation_steps):
            vrep.simxSetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, (gripper_orientation[0], gripper_orientation[1], gripper_orientation[2] + rotation_step*min(step_iter,num_rotation_steps)), vrep.simx_opmode_blocking)
        vrep.simxSetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, (gripper_orientation[0], gripper_orientation[1], tool_rotation_angle), vrep.simx_opmode_blocking)

    def rotate_gripper(self, rotate_angle_alpha=None, rotate_angle_beta=None, rotate_angle_gamma=None):
        if rotate_angle_alpha != None:
            self.rotate_alpha(rotate_angle_alpha)
        if rotate_angle_beta != None:
            self.rotate_beta(rotate_angle_beta)
        if rotate_angle_gamma != None:
            self.rotate_gamma(rotate_angle_gamma)
        if rotate_angle_alpha == rotate_angle_beta == rotate_angle_gamma == None:
            print('None angle has been given, the gripper will not rotate.')
    
    def close_simulation(self):
        vrep.simxStopSimulation(self.sim_client, vrep.simx_opmode_blocking)

    def get_camera_data(self, camera_handle):

        # Get color image from simulation
        sim_ret, resolution, raw_image = vrep.simxGetVisionSensorImage(self.sim_client, camera_handle, 0, vrep.simx_opmode_blocking)
        color_img = np.asarray(raw_image)
        color_img.shape = (resolution[1], resolution[0], 3)
        color_img = color_img.astype(np.float) / 255
        color_img[color_img < 0] += 1
        color_img *= 255
        color_img = np.fliplr(color_img)
        color_img = color_img.astype(np.uint8)

        # Get depth image from simulation
        sim_ret, resolution, depth_buffer = vrep.simxGetVisionSensorDepthBuffer(self.sim_client, camera_handle, vrep.simx_opmode_blocking)
        depth_img = np.asarray(depth_buffer)
        depth_img.shape = (resolution[1], resolution[0])
        depth_img = np.fliplr(depth_img)
        zNear = 0.01
        zFar = 10
        depth_img = depth_img * (zFar - zNear) + zNear

        return color_img, depth_img

    def get_camera_pose(self, camera_handle):
        # Get camera pose in simulation
        sim_ret, cam_position = vrep.simxGetObjectPosition(self.sim_client, camera_handle, -1, vrep.simx_opmode_blocking)
        sim_ret, cam_orientation = vrep.simxGetObjectOrientation(self.sim_client, camera_handle, -1, vrep.simx_opmode_blocking)
        cam_trans = np.eye(4, 4)
        cam_trans[0:3, 3] = np.asarray(cam_position)
        cam_orientation = [-cam_orientation[0], -cam_orientation[1], -cam_orientation[2]]
        cam_rotm = np.eye(4, 4)
        cam_rotm[0:3, 0:3] = np.linalg.inv(self.euler2rotm(cam_orientation))
        cam_pose = np.dot(cam_trans, cam_rotm)  # Compute rigid transformation representating camera pose

        return cam_pose

    def setup_sim_cameras(self):

        # Get handle to camera
        sim_ret, self.static_cam_handle = vrep.simxGetObjectHandle(self.sim_client, 'Vision_sensor_static', vrep.simx_opmode_blocking)
        sim_ret, self.gripper_cam_handle = vrep.simxGetObjectHandle(self.sim_client, 'Vision_sensor_gripper', vrep.simx_opmode_blocking)

        self.cam_intrinsics = np.asarray([[618.62, 0, 320], [0, 618.62, 240], [0, 0, 1]])
        self.cam_depth_scale = 1

        self.s_pose = self.get_camera_pose(self.static_cam_handle)
        self.g_pose = self.get_camera_pose(self.gripper_cam_handle)

        print('s_pose is :')
        print(self.s_pose)
        print('g_pose is :')
        print(self.g_pose)

        # Get background image
        self.sbg_color_img, self.sbg_depth_img = self.get_camera_data(self.static_cam_handle)
        self.sbg_depth_img = self.sbg_depth_img * self.cam_depth_scale

        self.gbg_color_img, self.gbg_depth_img = self.get_camera_data(self.gripper_cam_handle)
        self.gbg_depth_img = self.gbg_depth_img * self.cam_depth_scale

        scipy.misc.imsave('./sRGB.jpg', self.sbg_color_img)
        scipy.misc.imsave('./gRGB.jpg', self.gbg_color_img)

    # Get rotation matrix from euler angles
    def euler2rotm(self, theta):
        R_x = np.array([[1, 0, 0], [0, math.cos(theta[0]), -math.sin(theta[0])], [0, math.sin(theta[0]), math.cos(theta[0])]])
        R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])], [0, 1, 0], [-math.sin(theta[1]), 0, math.cos(theta[1])]])
        R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0], [math.sin(theta[2]), math.cos(theta[2]), 0], [0, 0, 1]])
        R = np.dot(R_z, np.dot(R_y, R_x))
        return R

    def get_pointcloud(self, depth_img, camera_intrinsics):

        im_h = depth_img.shape[0]
        im_w = depth_img.shape[1]

        pix_x,pix_y = np.meshgrid(np.linspace(0,im_w-1,im_w), np.linspace(0,im_h-1,im_h))
        cam_pts_x = np.multiply(pix_x - camera_intrinsics[0][2], depth_img / camera_intrinsics[0][0])
        cam_pts_y = np.multiply(pix_y - camera_intrinsics[1][2], depth_img / camera_intrinsics[1][1])
        cam_pts_z = depth_img.copy()
        cam_pts_x.shape = (im_h * im_w, 1)
        cam_pts_y.shape = (im_h * im_w, 1)
        cam_pts_z.shape = (im_h * im_w, 1)

        cam_pts = np.concatenate((cam_pts_x, cam_pts_y, cam_pts_z), axis=1)

        return cam_pts

    def pclwriter_ply(self, filename, xyz_pts):
        rgb_pts = np.ones(xyz_pts.shape).astype(np.uint8)*255

        pc_file = open(filename, 'wb')
        pc_file.write('ply\n'.encode())
        pc_file.write('format binary_little_endian 1.0\n'.encode())
        #pc_file.write('format ascii 1.0\n'.encode())
        pc_file.write('element vertex %d\n'.encode() % xyz_pts.shape[0])
        pc_file.write('property float x\n'.encode())
        pc_file.write('property float y\n'.encode())
        pc_file.write('property float z\n'.encode())
        pc_file.write('property uchar red\n'.encode())
        pc_file.write('property uchar green\n'.encode())
        pc_file.write('property uchar blue\n'.encode())
        pc_file.write('end_header\n'.encode())

        for i in range(xyz_pts.shape[0]):
            pc_file.write(bytearray(struct.pack("fffccc",xyz_pts[i][0],xyz_pts[i][1],xyz_pts[i][2],rgb_pts[i][0].tostring(),rgb_pts[i][1].tostring(),rgb_pts[i][2].tostring())))
        pc_file.close()
    
    def pclwriter_txt(self, filename, xyz_pts):

        pc_file = open(filename, 'w')

        for i in range(xyz_pts.shape[0]):
            data = str(xyz_pts[i][0]) + ', ' + str(xyz_pts[i][1]) + ', ' +  str(xyz_pts[i][2]) + '\n'
            pc_file.write(data)
        pc_file.close()
