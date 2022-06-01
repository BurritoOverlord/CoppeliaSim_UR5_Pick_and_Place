
import threading
import sim
import globalvariables as g
import time
import gripper as grip
import moveL as mL
import datetime


# Initialization

g.kFinal = 700/1000  # speed
g.connectionMessage(g.clientID)  # Printing out a successful/unsuccessful remote API connection message



# Functions


#Performs the operation of moving the bone-chicken basket from the conveyor to the deep-fryer
def function(arrIndex, counter):
    initBasket(arrIndex) #Spawning a non-dynamic basket to the 'batter and breading machine' platform
    g.tableArr[counter].type = "BONE"
    #checktimers.delayIfNecessary() #Accounting for multiple baskets in the system

    grip.openGripperAtStart(g.clientID, g.j1, g.j2, g.p1, g.p2) #Opens gripper at the very beginning of moving the basket
    time.sleep(2)
    moveBasketFunc(g.clientID, 1, arrIndex) #Move basket from conveyor -> table x
    time.sleep(5)
    returnBasketFunc(g.clientID, 1, arrIndex)

def initBasket(arrIndex):
    errorCode, basketH = sim.simxGetObjectHandle(g.clientID, 'Basket' + arrIndex, sim.simx_opmode_blocking)
    sim.simxSetObjectPosition(g.clientID, basketH, -1, g.basket_spawn, sim.simx_opmode_oneshot)

def moveBasketFunc(clientid, targetPosition, arrIndex):
    errorCode, basketH = sim.simxGetObjectHandle(g.clientID, 'Basket' + arrIndex, sim.simx_opmode_blocking)

    #Moving to initial position
    mL.move_L(clientid, g.target, g.initial_pos, g.kFinal)
    time.sleep(2)
    sim.simxSetObjectParent(clientid, basketH, g.connector, True, sim.simx_opmode_blocking)

    #Closing gripper
    grip.closeGripper(clientid)
    time.sleep(1)

    if (targetPosition == 1):
        #Moving object to deep fryer
        mL.move_L(clientid, g.target, g.b1_int_pos, g.kFinal)
        time.sleep(0.5)
        mL.move_L(clientid, g.target, g.b1_int_pos_2, g.kFinal)
        time.sleep(0.5)
        mL.move_L(clientid, g.target, g.b1_int_pos_3, g.kFinal)
        time.sleep(0.5)
        mL.move_L(clientid, g.target, g.b1_int_pos_4, g.kFinal)
        time.sleep(0.5)
        mL.move_L(clientid, g.target, g.b1_final_pos, g.kFinal)
        time.sleep(2)
        sim.simxSetObjectParent(clientid, basketH, -1, True, sim.simx_opmode_blocking)
        grip.gripperFunction(clientid, 0, g.j1, g.j2, g.p1, g.p2)

        g.tableArr[0].occupy() #Table 1 is now occupied
        g.tableArr[0].startTimer() #Starting timer for table 1
        time.sleep(2)

        #Moving robotic arm to the 'mutual position'--where it can now reach the other baskets
        mL.move_L(clientid, g.target, g.b1_int_pos_4, g.kFinal)
        time.sleep(1)
        mL.move_L(clientid, g.target, g.b1_int_pos_3, g.kFinal)
        time.sleep(1)

def returnBasketFunc(clientid, targetPosition, arrIndex):
    errorCode, basketH = sim.simxGetObjectHandle(g.clientID, 'Basket' + arrIndex, sim.simx_opmode_blocking)

    if (targetPosition == 1):
        mL.move_L(clientid, g.target, g.b1_final_pos, g.kFinal)
        time.sleep(2)
        sim.simxSetObjectParent(clientid, basketH, g.connector, True, sim.simx_opmode_blocking)
        grip.closeGripper(clientid)
        time.sleep(1)

        mL.move_L(clientid, g.target, g.b1_return_pos, g.kFinal)
        time.sleep(1.5)
        sim.simxSetObjectParent(clientid, basketH, -1, True, sim.simx_opmode_blocking)
        grip.gripperFunction(clientid, 0, g.j1, g.j2, g.p1, g.p2)
        time.sleep(1)


############################# Python Script ###############################
function("",0)

# Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
sim.simxGetPingTime(g.clientID)

# Now close the connection to CoppeliaSim:
sim.simxFinish(g.clientID)