import pybullet as p
import time
import pybullet_data

if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0, 0, -9.81)
    planeId = p.loadURDF("plane.urdf")
    ObjId = p.loadURDF("/home/userfs/r/rh1937/Kinematic_Assignment/MDaK/URDF/MDaK_assignment_1.urdf", [0,0,1])
    # You can also specify the position and orientation by
    #------------------------------------------------------------------------
    # startPos = [0,0,1]
    # startOrientation = p.getQuaternionFromEuler([0,0,0])
    # boxId = p.loadURDF("example.urdf",startPos, startOrientation)
    #------------------------------------------------------------------------
    while True:
        p.stepSimulation()
        time.sleep(1./240.)