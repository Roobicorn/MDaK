import pybullet as p
import time
import pybullet_data
import subprocess
from pathlib import Path

#Change project_folder file path to local path to run on local machine
#you may need to add the character 'r' before the filepath e.g. Path(r"filepath")
project_folder = Path("/home/userfs/r/rh1937/Kinematic_Assignment/MDaK/URDF/")
pool_bot_file = project_folder / "pool_bot.urdf"
ball_white_file = project_folder / "ball_white.urdf"
ball_red_file = project_folder / "ball_red.urdf"

if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0, 0, -9.81)
    planeId = p.loadURDF("plane.urdf")

    #Load Table & Balls URDFs
    pool_bot = p.loadURDF(str(pool_bot_file.absolute()), [0,0,0])
    ball_white = p.loadURDF(str(ball_white_file.absolute()), [0, 0.4, 0.8])
    ball_red = p.loadURDF(str(ball_red_file.absolute()), [0, -0.4, 0.8])



    #Joint controllers (from xarm.py example)
    jointIds = []
    paramIds = []

    for j in range(p.getNumJoints(pool_bot)):
        #p.changeDynamics(fr3_robot, j, linearDamping=0, angularDamping=0)
        info = p.getJointInfo(pool_bot, j)
        # print(info)
        jointName = info[1]
        jointType = info[2]
        jointLLim = info[8]
        jointULim = info[9]

        if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
            jointIds.append(j)
            paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), jointLLim, jointULim, 0))




    while True:
        p.stepSimulation()

        for i in range(len(paramIds)):
            c = paramIds[i]
            targetPos = p.readUserDebugParameter(c)
            p.setJointMotorControl2(pool_bot, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)

        time.sleep(1./240.)