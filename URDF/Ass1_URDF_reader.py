import pybullet as p
import time
import pybullet_data
from pathlib import Path
import numpy as np

#Change project folder file path to run on local machine
project_folder = Path("/home/userfs/r/rh1937/Kinematic_Assignment/MDaK/URDF/")
fr3_urdf_file = project_folder / "MDaK_assignment_1.urdf"

if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0, 0, -9.81)
    planeId = p.loadURDF("plane.urdf")
    fr3_robot = p.loadURDF(str(fr3_urdf_file.absolute()), [0,0,0])

    #Getting joint info
    num_joints = p.getNumJoints(fr3_robot)
    print("number of joints: ", num_joints)

    rJointIds = []
    paramIds = []
    #link_name_to_index = {p.getBodyInfo(fr3_robot)[0].decode('UTF-8'): -1}
    index_to_link_name = {-1: p.getBodyInfo(fr3_robot)[0].decode('UTF-8')}
    #joint_name_to_index = {}
    index_to_joint_name = {}

    for j in range(p.getNumJoints(fr3_robot)):
        #p.changeDynamics(fr3_robot, j, linearDamping=0, angularDamping=0)
        info = p.getJointInfo(fr3_robot, j)
        # print(info)
        jointName = info[1].decode('UTF-8')
        jointType = info[2]
        linkName = info[12].decode('UTF-8')
        if (jointType == p.JOINT_REVOLUTE):
            rJointIds.append(j)
            #link_name_to_index[linkName] = j
            index_to_link_name[j] = [linkName]
            #joint_name_to_index[jointName] = j
            index_to_joint_name[j] = jointName

            #set up joint control sliders
            paramIds.append(p.addUserDebugParameter(jointName, info[8], info[9], 0))

    print("revolute joint IDs:", rJointIds)
    #print("associated joint names:", joint_name_to_index)
    print("associated joint names: ", index_to_joint_name)
    #print("associated link names:", link_name_to_index)
    print("associated link names:", index_to_link_name)


    linkStates = p.getLinkStates(fr3_robot, list(range(num_joints)), computeLinkVelocity=0, computeForwardKinematics=1)
    for i in rJointIds:
        print("link id =", i, "worldPos =", linkStates[i][0], "worldOrn =", p.getEulerFromQuaternion(linkStates[i][1]),
              #"locInertialPos =", linkStates[i][2], "locInertialOrn =", p.getEulerFromQuaternion(linkStates[i][3]),
              "worldLinkFramePos=", linkStates[i][4], "worldLinkFrameOrn=", p.getEulerFromQuaternion(linkStates[i][5]))

    #for printing link info in simulation
    stepcount = 0
    testLink = 23

    ####################################################################################################################
    # Assignment 2.1: Homogeneous Transform Matrices to the base frame for each joint

    d_rotation_matrices = {}
    d_position_matrices = {}
    np.set_printoptions(precision=3, suppress=True)

    for joint in rJointIds:
        print("Transformation matrices for joint", index_to_joint_name.get(joint), "(joint ID:", joint, "):")
        d_position_matrices[joint] = np.array([[linkStates[joint][0][0]],[linkStates[joint][0][1]],[linkStates[joint][0][2]]])
        print("Position Matrix: \n", d_position_matrices.get(joint))





    while True:
        p.stepSimulation()





# For forward kinematics
        # stepcount += 1
        # if stepcount == 500:
        #     linkStates = p.getLinkStates(fr3_robot, list(range(num_joints)), computeLinkVelocity=0,
        #                                  computeForwardKinematics=1)
        #     print("link id =", testLink, "worldPos =", linkStates[testLink][0], "worldOrn =",
        #           p.getMatrixFromQuaternion(linkStates[testLink][1]),
        #           # "locInertialPos =", linkStates[testLink][2], "locInertialOrn =", p.getEulerFromQuaternion(linkStates[testLink][3]),
        #           "worldLinkFramePos=", linkStates[testLink][4], "worldLinkFrameOrn=",
        #           p.getMatrixFromQuaternion(linkStates[testLink][5]))
        #     stepcount = 0


        for i in range(len(paramIds)):
            c = paramIds[i]
            targetPos = p.readUserDebugParameter(c)
            p.setJointMotorControl2(fr3_robot, rJointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)


        time.sleep(1./240.)
