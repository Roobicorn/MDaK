import pybullet as p
import time
import pybullet_data
import subprocess
from pathlib import Path

#Change project folder file path to run on local machine
project_folder = Path("/home/userfs/r/rh1937/Kinematic_Assignment/MDaK/URDF/")
pool_bot_file = project_folder / "pool_bot.urdf"

if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0, 0, -9.81)
    planeId = p.loadURDF("plane.urdf")
    #Load Table & Balls URDFs

    # Code to automate converting xacro file to URDF.
    # Not currently working, manually convert files in terminal.
    # xacro_file = "/home/userfs/r/rh1937/Kinematic_Assignment/MDaK/URDF/pool_bot.xacro"
    # urdf_file = "/home/userfs/r/rh1937/Kinematic_Assignment/MDaK/URDF/pool_bot.urdf"
    # subprocess.run(['xacro', xacro_file, '>', urdf_file])

    pool_bot = p.loadURDF(str(pool_bot_file.absolute()), [0,0,0])
    # You can also specify the position and orientation by
    #------------------------------------------------------------------------
    # startPos = [0,0,1]
    # startOrientation = p.getQuaternionFromEuler([0,0,0])
    # boxId = p.loadURDF("example.urdf",startPos, startOrientation)
    #------------------------------------------------------------------------

    #Joint controllers (from xarm.py example)
    jointIds = []
    paramIds = []

    for j in range(p.getNumJoints(pool_bot)):
        #p.changeDynamics(fr3_robot, j, linearDamping=0, angularDamping=0)
        info = p.getJointInfo(pool_bot, j)
        # print(info)
        jointName = info[1]
        jointType = info[2]

        if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
            jointIds.append(j)
            paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))




    while True:
        p.stepSimulation()

        for i in range(len(paramIds)):
            c = paramIds[i]
            targetPos = p.readUserDebugParameter(c)
            p.setJointMotorControl2(pool_bot, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)

        time.sleep(1./240.)