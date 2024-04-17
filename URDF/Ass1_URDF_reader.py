import pybullet as p
import time
import pybullet_data
from pathlib import Path

#Change project folder file path to run on local machine
project_folder = Path("/home/userfs/r/rh1937/Kinematic_Assignment/MDaK/URDF/")
fr3_urdf_file = project_folder / "MDaK_assignment_1.urdf"

if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0, 0, -9.81)
    planeId = p.loadURDF("plane.urdf")
    fr3_robot = p.loadURDF(str(fr3_urdf_file.absolute()), [0,0,0])
    # You can also specify the position and orientation by
    #------------------------------------------------------------------------
    # startPos = [0,0,1]
    # startOrientation = p.getQuaternionFromEuler([0,0,0])
    # boxId = p.loadURDF("example.urdf",startPos, startOrientation)
    #------------------------------------------------------------------------



    #Getting joint info
    num_joints = p.getNumJoints(fr3_robot)
    print("number of joints: ", num_joints)

    # revolute_joints = []
    # for num in range(num_joints):
    #     info = p.getJointInfo(fr3_robot, num)
    #     if(info[2] == 0):
    #         revolute_joints.append(info)
    #         print(info)

    #Joint controllers (from xarm.py example)
    jointIds = []
    paramIds = []

    for j in range(p.getNumJoints(fr3_robot)):
        p.changeDynamics(fr3_robot, j, linearDamping=0, angularDamping=0)
        info = p.getJointInfo(fr3_robot, j)
        # print(info)
        jointName = info[1]
        jointType = info[2]
        if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
            jointIds.append(j)
            paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))

    print("prismatic or revolute joint IDs =", jointIds)

    #Getting link info

    linkStates = p.getLinkStates(fr3_robot, list(range(num_joints)), computeLinkVelocity=0, computeForwardKinematics=1)
    for i in range(len(linkStates)):
        print("link id =", i, "worldPos =", linkStates[i][0], "worldOrn =", linkStates[i][1], "locInertialPos =", linkStates[i][2],
              "locInertialOrn =", linkStates[i][3], "worldLinkFramePos=", linkStates[i][4], "worldLinkFrameOrn=", linkStates[i][5])


    # base_pos_orn = p.getBasePositionAndOrientation(fr3_robot)
    # base_euler_orn = p.getEulerFromQuaternion(base_pos_orn[1])
    # print("base position and orientation =", base_pos_orn)
    # print("base euler orientation", base_euler_orn)



    #manipulating joints
    #rotates arm_link_1a_to_base_cyl ccw around up z axis
    #p.setJointMotorControl2(fr3_robot, 2, controlMode=p.VELOCITY_CONTROL, targetVelocity=5, force=100)

    #rotates arm_link_2b_to_1b
    #p.setJointMotorControl2(fr3_robot, 5, controlMode=p.POSITION_CONTROL, targetPosition=-0.45, force=50)

    #p.setJointMotorControl2(fr3_robot, 8, controlMode=p.POSITION_CONTROL, targetPosition=2, force=50)

    #jointMotorControlArray
    #p.setJointMotorControlArray(fr3_robot, r_joint_indexes, controlMode = p.POSITION_CONTROL) #targetPositions = [list], forces= [list]

    # velo = 2.0
    # vel_rising = False

    while True:
        p.stepSimulation()

        for i in range(len(paramIds)):
            c = paramIds[i]
            targetPos = p.readUserDebugParameter(c)
            p.setJointMotorControl2(fr3_robot, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)


        #Oscillates joint 1a_to_base:
        # p.setJointMotorControl2(fr3_robot, 2, controlMode=p.VELOCITY_CONTROL, targetVelocity=velo, force=10)
        # if vel_rising == False:
        #     velo -= 0.01
        #     if velo < -2: vel_rising = True
        # else:
        #     velo += 0.01
        #     if velo > 2: vel_rising = False


        time.sleep(1./240.)
