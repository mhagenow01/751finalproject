# Robotiq Gripper Simulations Using Chrono

### Author: Mike Hagenow
### ME 751 Final Project

## Project Description

This project investigates the use of a linkage-based robotic gripper, particularly as a method to perform tasks involving slip in orientation. Typically, robots
perform rigid grasps of objects, but we draw inspiration from human demonstrations where slip is often used to avoid awkward postures. Our hypothesis is that
by including slip degrees of freedom, a robot can perform tasks (e.g. opening a refrigerator) in ways that are more feasible for the robot's kinematic chain.

This project aims to model a linkage-based gripper in chrono and perform basic analysis using a complementarity and penalty-based friction/contact model to see if
we can gain insight for how the gripper should be controlled to facilitate slip. Our basic hypothesis is that gripping normal force can serve as a proxy for slip allowance,
but we are also interested if the grip force signal contains other artifacts that can be used in the gripper control paradigm.

![Example Gripper Simulation Image](/media/gripper_image.PNG?raw=true "Gripper in Chrono")

## Building the Project

The project builds against the Chrono C++ API. To build the gripper simulation files, you will need to follow the steps outlined here: http://api.projectchrono.org/development/tutorial_install_project.html

Additionally, since the project uses some relative paths, please follow the below steps to build successfully:
* Chrono should be placed in the root directory of the github repository (i.e., at the same level as the gripper_slip_simulations directory)
* Chrono is configured to build all files into the 'build' directory in the root
* gripper_slip_simulations contains a CMakeLists that can also be used through CMake to build the gripper simulation files. I did all testing with Visual Studio 2017. It was tested one two separate machines.
 When I tested, the executables were built to the build/Release/ directory (provided you compile with Release).
 
 ![Intended File Structure](/media/example_file_structure.PNG?raw=true "Project File Structure")

## Running the simulations
Once the files have been build, this repository contains 4 exes that can be run alone that include irrchlet visualization (gripper_kinematics, gripper_friction_test_comp, gripper_complementarity_friction, gripper_penalty_friction).

placeholder text