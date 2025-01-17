# Auro Assessment Submission

## Overview
Thorugh use of  ROS2, Gazebo, RViz, and the Nav2 navigation stack this project aims to provide a modular simulation of 1-3 robots in a virtual environment.
These robots are tasked to pick up and deposit items in the appropriate zones by a task manager node.

##Prerequisites
This is to be ran within the environment provided for the AURO module

## Installation
1. Navigate to the submission directory

2. Build the project : 
colcon build 

3. Source the install file : 
source install/local_setup.bash

4. Launch the solution : 
ros2 launch solution solution_nav2_launch.py

5. Optionally modify the variables in the solution
See what arguments can be modified:
--show-arguments
And then pass arguments as:
'<name>:=<value>'

## Usage
Tasks can be dynamically generated and assigned to robots
Can be used to simulate a robot_controller in a real world scenario

## Features
Centralised task generation and allocation
Robot collaboration
Collision avoidance
Modularity
