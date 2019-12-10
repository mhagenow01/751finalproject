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
Once the files have been build, this repository contains 4 exes that can be run alone that include irrlicht visualization (gripper_kinematics, gripper_friction_test_comp, gripper_complementarity_friction, gripper_penalty_friction).

Note: If you don't want to actually build the files, I have also included a video in the media folder (gripper_simulation_videos.mp4) that shows all four of the simulations.

In order to create the plots in the report, there are bash scripts in the /scripts/ directory that allow you to run the penalty and complementarity methods
and plot the contact forces from one of the gripper fingers.
* run_complementarity.bash
* run_penalty.bash
These scripts will run the chrono simulation, save the data to a file, and then plot the data in python. It is important these are run from the scripts directory
as they rely on relative paths.

They also require the following python packages: numpy, matplotlib. I tested using Python 3.7.0 on windows and using the Git Bash terminal (Note: I had to run alias python='winpty python.exe' for the system to recognize the python command).