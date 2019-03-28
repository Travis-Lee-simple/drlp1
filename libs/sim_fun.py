'''
designed for lab1 wirobot track
'''
import time
import struct
import numpy as np
from . import vrep
import math
import scipy.misc
import cv2

class SimRobot():
    def __init__(self):
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
        sim_ret, self.UR5_target_handle = vrep.simxGetObjectHandle(self.sim_client,'Vision_sensor_static',vrep.simx_opmode_blocking)
        vrep.simxStopSimulation(self.sim_client, vrep.simx_opmode_blocking)
        vrep.simxStartSimulation(self.sim_client, vrep.simx_opmode_blocking)
        time.sleep(1)

    def check_sim(self):
        pass

    def get_3dcamera_data(self, camera_handle):

        # Get color image from simulation
        sim_ret, resolution, raw_image = vrep.simxGetVisionSensorImage(self.sim_client, camera_handle, 0, vrep.simx_opmode_blocking)
        
        color_img = np.asarray(raw_image)
        #print(color_img.shape)
        
        color_img.shape = (resolution[1], resolution[0], 3)
        color_img = color_img.astype(np.float) / 255
        color_img[color_img < 0] += 1
        color_img *= 255
        color_img = np.fliplr(color_img)
        color_img = color_img.astype(np.uint8)

        #cv2.imshow('raw_image',color_img)
        #cv2.waitKey(0)
        #color_img=cv2.mat(color_img)
        
        

        '''
        # Get depth image from simulation
        sim_ret, resolution, depth_buffer = vrep.simxGetVisionSensorDepthBuffer(self.sim_client, camera_handle, vrep.simx_opmode_blocking)
        depth_img = np.asarray(depth_buffer)
        depth_img.shape = (resolution[1], resolution[0])
        depth_img = np.fliplr(depth_img)
        zNear = 0.01
        zFar = 10
        depth_img = depth_img * (zFar - zNear) + zNear
        '''
        return color_img #, depth_img

    def get_camera_data(self, camera_handle):

        # Get color image from simulation
        sim_ret, resolution, raw_image = vrep.simxGetVisionSensorImage(self.sim_client, camera_handle, 0, vrep.simx_opmode_blocking)
        
        color_img = np.asarray(raw_image)
        #print(color_img.shape)
        
        color_img.shape = (resolution[1], resolution[0], 3)
        color_img = color_img.astype(np.float) / 255
        color_img[color_img < 0] += 1
        color_img *= 255
        color_img = np.fliplr(color_img)
        color_img = color_img.astype(np.uint8)

        #cv2.imshow('raw_image',color_img)
        #cv2.waitKey(0)
        #color_img=cv2.mat(color_img)
        
        

        '''
        # Get depth image from simulation
        sim_ret, resolution, depth_buffer = vrep.simxGetVisionSensorDepthBuffer(self.sim_client, camera_handle, vrep.simx_opmode_blocking)
        depth_img = np.asarray(depth_buffer)
        depth_img.shape = (resolution[1], resolution[0])
        depth_img = np.fliplr(depth_img)
        zNear = 0.01
        zFar = 10
        depth_img = depth_img * (zFar - zNear) + zNear
        '''
        return color_img #, depth_img

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
        #sim_ret, self.gripper_cam_handle = vrep.simxGetObjectHandle(self.sim_client, 'Vision_sensor_gripper', vrep.simx_opmode_blocking)

        self.cam_intrinsics = np.asarray([[618.62, 0, 320], [0, 618.62, 240], [0, 0, 1]])
        self.cam_depth_scale = 1

        self.s_pose = self.get_camera_pose(self.static_cam_handle)
        #self.g_pose = self.get_camera_pose(self.gripper_cam_handle)

        #print('s_pose is :')
        #print(self.s_pose)
        #print('g_pose is :')
        #print(self.g_pose)

        # Get background image
        #self.sbg_color_img, self.sbg_depth_img = self.get_camera_data(self.static_cam_handle)
        self.sbg_color_img = self.get_camera_data(self.static_cam_handle)
        #self.sbg_depth_img = self.sbg_depth_img * self.cam_depth_scale

        #self.gbg_color_img, self.gbg_depth_img = self.get_camera_data(self.gripper_cam_handle)
        #self.gbg_depth_img = self.gbg_depth_img * self.cam_depth_scale

        #cv2.imsave('./sRGB.jpg', self.sbg_color_img)
        #scipy.misc.imsave('./gRGB.jpg', self.gbg_color_img)

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

    def turn_left(self):
        self.set_Tank_Param(3)

    def turn_right(self):
        self.set_Tank_Param(4)

    def go_straight(self):
        self.set_Tank_Param(1)

    def go_back(self):
        self.set_Tank_Param(2)

    def set_Tank_Param(self,index):
        x=[0,0,0,0,0]
        x[index]=1
        #x=[0,1,0,0,0]
        vrep.simxCallScriptFunction(self.sim_client,'Edgeless#0',vrep.sim_scripttype_childscript,'set_Tank_Param',
            x,[],[],bytearray(),vrep.simx_opmode_blocking)
