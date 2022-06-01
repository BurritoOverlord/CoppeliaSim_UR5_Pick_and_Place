# globalvariables.py is the module that contains the global variables for other files to access.
# Additionally, all initializations (clientID, handles, coordinates, etc, etc.) are configured here.

import sys
import sim
import numpy as np
import time


kFinal = 0



## Initialization ##

clientID = sim.simxStart('127.0.0.1', 19990, True, True, 5000, 5)  #Server IP, Port num, boolean wait-until-connected
                                                                   #boolean doNotReconnectOnceDisconnected, timeOutinMs,
                                                                   #commThreadCycleinMs (usually set as 5)

def connectionMessage(clientid):
    '''
    #################################### Setup #####################################
    '''
    #sim.simxFinish(-1)  # just in case, close all opened connections ----- IF I NEED to reset sandboxscript on Vrep
    if clientid != -1:
        print('Connected to remote API server')
    else:
        print('connection not successful')
        sys.exit("could not connect")

PI = np.pi #For move_L calculations

#Obtaining appropriate handles
errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_blocking) #target dummy
errorCode, j1 = sim.simxGetObjectHandle(clientID, 'ROBOTIQ_85_active1', sim.simx_opmode_blocking) #gripper joint 1
errorCode, j2 = sim.simxGetObjectHandle(clientID, 'ROBOTIQ_85_active2', sim.simx_opmode_blocking) #gripper joint 2
errorCode, connector = sim.simxGetObjectHandle(clientID, 'ROBOTIQ_85_attachPoint',
                                               sim.simx_opmode_blocking) #gripper connect point

#Obtaining joint positions for the gripper to close & open
errorCode, p1 = sim.simxGetJointPosition(clientID, j1, sim.simx_opmode_streaming)
errorCode, p2 = sim.simxGetJointPosition(clientID, j2, sim.simx_opmode_streaming)

returnCode, pos = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_streaming)
returnCode, orient = sim.simxGetObjectOrientation(clientID, target, -1, sim.simx_opmode_streaming)

                ## Coordinates ##

#Initial + spawn point coordinates
initial_pos = [0.275, 0.505, 0.87, 0, 0, 0] #[x, y, z, alpha, beta, gamma]
basket_spawn = [0.25, 0.875, 0.765] #[x, y, z]

#Basket-returning coordinates
b1_return_pos = [0.15, 0.775, 0.765, 0, 0, 0]
b2_return_pos = [0.275, 0.505, 0.87, 0, 0, 0]
b3_return_pos = [0.25-0.1, 0.875-0.1, 0.765]
b4_return_pos = [0.275, 0.505, 0.87, 0, 0, 0]

#Basket-moving coordinates for basket 1
b1_int_pos = [0.25-0.1, 0.875-0.1, 0.765]
b1_int_pos_2 = [0.275, 0.505, 0.87, 0, 0, 0]
b1_int_pos_3 = [0.25-0.1, 0.875-0.1, 0.765]
b1_int_pos_4 = [0.275, 0.505, 0.87, 0, 0, 0]
b1_final_pos = [0.25-0.1, 0.875-0.1, 0.765]
