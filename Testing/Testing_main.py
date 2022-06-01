import sim
from Testing import Testing_globalvariables as g
import time
from Testing import Testing_gripper as grip
from Testing import Testing_moveL as mL

# Initialization

g.kFinal = 700/1000  # speed
g.connectionMessage(g.clientID)  # Printing out a successful/unsuccessful remote API connection message



# Functions


#Performs the operation of moving the bone-chicken basket from the conveyor to the deep-fryer
def function():
    initCups() #Spawning a non-dynamic basket to the 'batter and breading machine' platform

    #grabCupFunc(g.clientID, 1, "")  # Grab Cup "" from its position

    """
    moveBasketFunc(g.clientID, 1, "") #Move basket from conveyor -> table x
    time.sleep(5)
    returnBasketFunc(g.clientID, 1, "")
    """

def initCups():
    errorCode, cupH = sim.simxGetObjectHandle(g.clientID, 'Cup', sim.simx_opmode_blocking)
    sim.simxSetObjectPosition(g.clientID, cupH, -1, g.basket_spawn, sim.simx_opmode_oneshot)

    time.sleep(2)
    errorCode, connector = sim.simxGetObjectHandle(g.clientID, 'ROBOTIQ_85_attachPoint', sim.simx_opmode_blocking)  # gripper connect point

    grip.openGripperAtStart(g.clientID, g.j1, g.j2, g.p1, g.p2) #Opens gripper at the very beginning of moving the basket
    time.sleep(2)

    time.sleep(2)
    mL.move_L(g.clientID, g.target, g.initial_pos, g.kFinal) #Go to initial Position
    time.sleep(0.5)
    mL.move_L(g.clientID, g.target, g.b1_return_pos, g.kFinal)  # Go to initial Position

def grabCupFunc(clientid, targetPosition, arrIndex):
    # Get proper Cup handle based on the ID or the ArUco markers
    errorCode, cupH = sim.simxGetObjectHandle(g.clientID, 'Cup' + arrIndex, sim.simx_opmode_blocking)

    #Moving to desired Position Then Fake grasping
    mL.move_L(clientid, g.target, g.b1_return_pos, g.kFinal)
    time.sleep(2)
    #sim.simxSetObjectParent(clientid, cupH, g.connector, True, sim.simx_opmode_blocking)

    #Closing gripper
    grip.closeGripper(clientid)
    time.sleep(1)

############################# Python Script ###############################

def main():
    try:
        sim.simxStartSimulation(g.clientID, sim.simx_opmode_oneshot_wait)
        time.sleep(1)  # necessary for simulation to start properly

        ######### Main Functions ##########################################
        function()


    except Exception as e:
        print(e)
        sim.simxFinish(g.clientID)
    finally:
        sim.simxPauseSimulation(g.clientID, sim.simx_opmode_oneshot_wait)


if __name__ == "__main__":
    main()
