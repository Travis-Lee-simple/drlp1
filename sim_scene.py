import time
import numpy as np
from libs import sim_fun
from libs import vrep
import scipy.misc

# Modified from Tetromino by Al Sweigart al@inventwithpython.com
# http://inventwithpython.com/pygame
# Released under a "Simplified BSD" license
import random, time,  sys#,pygame
#from pygame.locals import *

CAMARA_NAME='Vision_sensor_static'
MAX_STEP=1000
FPS = 25



class GameState:
    def __init__(self):
        self.scene=sim_fun.SimRobot()
        # DEBUG
        self.total_lines = 0
        # setup variables for the start of the game
        self.time_step=0

    def reinit(self):
        self.scene.restart_sim()

        
    def frame_step(self,input):
        terminal=False
        self.time_step=self.time_step+1

        # moving the piece sideways
        if (input[1] == 1) and self.isValidPosition(adjX=-1):
            print('frame_step input:',input)

        elif (input[2] == 1) and self.isValidPosition(adjX=1):
            print('frame_step input:',input)

        # rotating the piece (if there is room to rotate)
        elif (input[3] == 1):
            print('frame_step input:',input)

        elif (input[4] == 1): # rotate the other direction
            print('frame_step input:',input)

        # drawing everything on the screen
        reward=self.getReward()
        image_data=self.getCameraData()
        if not self.isValidPosition():
        	terminal=True

        return image_data, reward, terminal


    def getReward(self):
        return 0
        stack_height = None
        num_blocks = 0
        for i in range(0, BOARDHEIGHT):
            blank_row = True
            for j in range(0, BOARDWIDTH):
                if self.board[j][i] != '.':
                    num_blocks += 1
                    blank_row = False
            if not blank_row and stack_height is None:
                stack_height = BOARDHEIGHT - i
                    
        if stack_height is None:
            return BOARDHEIGHT
        else:
            return BOARDHEIGHT - stack_height
            return float(num_blocks) / float(stack_height * BOARDWIDTH)

    def getCameraData(self):
    	sim_ret, camera_handle = vrep.simxGetObjectHandle(self.scene.sim_client,CAMARA_NAME,vrep.simx_opmode_blocking)
    	return self.scene.get_camera_data(camera_handle)

    def isValidPosition(self,adjX=0, adjY=0):
        # Return True if the piece is within the self.board and not colliding
        if self.time_step>MAX_STEP:
        	return True
        return False