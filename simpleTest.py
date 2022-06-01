# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import gripper as grip

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19990,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    time.sleep(2)

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    startTime=time.time()

    # Obtaining appropriate handles
    errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_blocking)  # target dummy
    errorCode, j1 = sim.simxGetObjectHandle(clientID, 'ROBOTIQ_85_active1', sim.simx_opmode_blocking)  # gripper joint 1
    errorCode, j2 = sim.simxGetObjectHandle(clientID, 'ROBOTIQ_85_active2', sim.simx_opmode_blocking)  # gripper joint 2
    errorCode, connector = sim.simxGetObjectHandle(clientID, 'ROBOTIQ_85_attachPoint',
                                                   sim.simx_opmode_blocking)  # gripper connect point

    # Obtaining joint positions for the gripper to close & open
    errorCode, p1 = sim.simxGetJointPosition(clientID, j1, sim.simx_opmode_streaming)
    errorCode, p2 = sim.simxGetJointPosition(clientID, j2, sim.simx_opmode_streaming)

    returnCode, pos = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_streaming)
    returnCode, orient = sim.simxGetObjectOrientation(clientID, target, -1, sim.simx_opmode_streaming)


    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    grip.openGripperAtStart(clientID, j1, j2, p1, p2) #Opens gripper at the very beginning of moving the basket


    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
