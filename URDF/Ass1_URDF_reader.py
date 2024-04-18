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

    rJointIds = []
    paramIds = []
    link_name_to_index = {p.getBodyInfo(fr3_robot)[0].decode('UTF-8'): -1, }
    joint_name_to_index = {}

    for j in range(p.getNumJoints(fr3_robot)):
        #p.changeDynamics(fr3_robot, j, linearDamping=0, angularDamping=0)
        info = p.getJointInfo(fr3_robot, j)
        # print(info)
        jointName = info[1].decode('UTF-8')
        jointType = info[2]
        linkName = info[12].decode('UTF-8')
        if (jointType == p.JOINT_REVOLUTE):
            paramIds.append(p.addUserDebugParameter(jointName, info[8], info[9], 0)) #sets up joint control slider

            rJointIds.append(j)
            link_name_to_index[linkName] = j
            joint_name_to_index[jointName] = j

    print("prismatic or revolute joint IDs:", rJointIds)
    print("associated joint names:", joint_name_to_index)
    print("associated link names:", link_name_to_index)

    #Getting link state info

    linkStates = p.getLinkStates(fr3_robot, list(range(num_joints)), computeLinkVelocity=0, computeForwardKinematics=1)
    for i in rJointIds:
        print("link id =", i, "worldPos =", linkStates[i][0], "worldOrn =", p.getEulerFromQuaternion(linkStates[i][1]),
              #"locInertialPos =", linkStates[i][2], "locInertialOrn =", p.getEulerFromQuaternion(linkStates[i][3]),
              "worldLinkFramePos=", linkStates[i][4], "worldLinkFrameOrn=", p.getEulerFromQuaternion(linkStates[i][5]))

    #TO DO: Try implementing the following from forum post:
    _link_name_to_index = {p.getBodyInfo(fr3_robot)[0].decode('UTF-8'): -1, }

    for _id in range(p.getNumJoints(fr3_robot)):
        _name = p.getJointInfo(fr3_robot, _id)[12].decode('UTF-8')
        _link_name_to_index[_name] = _id

    print(_link_name_to_index)


    #manipulating joints
    #rotates arm_link_7a_to_6a
    #p.setJointMotorControl2(fr3_robot, 23, controlMode=p.POSITION_CONTROL, targetPosition=-0.45, force=50)


    while True:
        p.stepSimulation()

        for i in range(len(paramIds)):
            c = paramIds[i]
            targetPos = p.readUserDebugParameter(c)
            p.setJointMotorControl2(fr3_robot, rJointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)

        # linkStates = p.getLinkStates(fr3_robot, list(range(num_joints)), computeLinkVelocity=0, computeForwardKinematics=1)
        # #for linkId in range(len(linkStates)):
        # linkId = 22
        # print("link id =", linkId, "worldPos =", linkStates[linkId][0], "worldOrn =", p.getEulerFromQuaternion(linkStates[linkId][1]),
        #           #"locInertialPos =", linkStates[linkId][2], "locInertialOrn =", p.getEulerFromQuaternion(linkStates[linkId][3]),
        #           "worldLinkFramePos=", linkStates[linkId][4], "worldLinkFrameOrn=", p.getEulerFromQuaternion(linkStates[linkId][5]))

        time.sleep(1./240.)
