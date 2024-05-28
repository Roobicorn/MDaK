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


    ####################################################################################################################
    # Assignment 2.1: Homogeneous Transform Matrices to the base frame for each joint

    d_rotation_matrices = {}
    d_position_matrices = {}
    d_homogeneous_t_matrices = {}
    t_matrix_bottom_row = np.array([0, 0, 0, 1])

    np.set_printoptions(precision=4) #, suppress=True)

    for joint in rJointIds:
        print("Transformation matrices for joint", index_to_joint_name.get(joint), "(joint ID:", joint, "):")
        d_position_matrices[joint] = np.array([[linkStates[joint][0][i]] for i in range(3)])
        print("Position Matrix: \n", d_position_matrices.get(joint))

        rotation_list = p.getMatrixFromQuaternion(linkStates[joint][1])
        d_rotation_matrices[joint] = np.array([[rotation_list[i] for i in range(j*3, (j*3)+3)] for j in range(3)])
        print("Rotation Matrix: \n", d_rotation_matrices.get(joint))

        d_homogeneous_t_matrices[joint] = \
            np.vstack((np.concatenate((d_rotation_matrices[joint], d_position_matrices[joint]), axis=1), t_matrix_bottom_row))
        print("Homogeneous Transformation Matrix: \n", d_homogeneous_t_matrices[joint], "\n")

    ####################################################################################################################
    # Assignment 2.2: Compute the Forward Kinematics for one configuration of your robot. (default configuration)

    #Apply transformation matrix for joint 'arm_link_7a_to_6a' (Joint ID: 23)
    print("Forward Kinematics for joint", index_to_joint_name.get(rJointIds[-1]), "(joint ID:", rJointIds[-1], "):")
    print("v' = Tv")

    formatted_t_matrix = np.array([[f"{item:.3f}" for item in row] for row in d_homogeneous_t_matrices[rJointIds[-1]]])

    print("[[x']     ", formatted_t_matrix[0,:], " [[x]\n",
          " [y']  =  ", formatted_t_matrix[1,:],"  [y]\n",
          " [z']]    ", formatted_t_matrix[2,:], "  [z]]\n", sep="")

    local_coords = np.array([[0],[0], [0], [1]])
    print("local coordinates = \n", local_coords)
    base_frame_coords = np.dot(d_homogeneous_t_matrices, local_coords) #or matmul?
    print("local coordinates = \n", base_frame_coords)





    # #for printing link info in simulation
    # stepcount = 0
    # testLink = 23

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
