# MDaK
Code for Mechanical Design and Kinematics assessment.

Assignments 1 and 2:

The URDF robot model for assignment 1 is MDaK_assignment_1.urdf

To run Ass1_URDF_reader.py 
edit the file path of the project folder in line 8 to match local directory:
project_folder = Path("/home/userfs/r/rh1937/Kinematic_Assignment/MDaK/URDF/")

The program should load the simulation, cycle through several poses (demonstrating inverse kinematics) then return to default position from where the robot can be moved manually via the parameter sliders.
The program will also print the Homogeneous Transformations for each joint to the base frame, and the results of FK, IK and VK calculations to the terminal.

To run Ass4_URDF_reader.py 
edit the file path of the project folder in line 8 to match local directory:
project_folder = Path("/home/userfs/r/rh1937/Kinematic_Assignment/MDaK/URDF/")

The program should load the simulation of a pool table with 2 balls, and a moveable robot.

To run MDaK_Assignment_3_Control.ipynb
Import it into google Colab:
go to https://colab.research.google.com/, and go to  file > upload notebook.
Once notebook is open, run the code cells in sequence.