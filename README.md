# Neobotix
MSc Dissertation Project

ROS configuration and launch files for Neobotix MPO-700
![GitHub Logo](/Pictures/mpo_700.jpeg)
Format: ![Alt Text](url)

This package contains configuration and launch files for Neobotix MPO-700.

Neobotix MPO-700Neobotix MMO-700

The omnidirectional MPO-700 is the ideal base for high-end service robots. Its four Omni-Drive-Modules enable it to move extremely smoothly into any direction. This robot is even capable of rotating freely while driving to its destination. The Omni-Drive-Modules of the MPO-700 feature important benefits compared to other omnidirectional drive kinematics, like for example the MPO-500's Mecanum wheels.

Fully omnidirectional manoeuvrability
Very steady movements
High stability and payload
Compact, easily integrated drive units
This makes the MPO-700 a premium alternative for applications that require omnidirectional movements without the limitations of traditional kinematics.

Contact information
For more information please visit our website at www.neobotix-robots.com. If you have any questions, just get in touch with us:

General information: http://www.neobotix-robots.com/company-contact.html
ROS related questions: ros@neobotix.de

------------Installation----------------------------
Create a catkin workspace (tutorial)

Download all packages 

Install all packages listed under "Additionally required third party ROS-Packages"

Build your workspace

Additionally required Neobotix-ROS-Packages:
Hardware connection: neo_relayboard_v2

Teleoperation: neo_teleop

Additionally required third party ROS-Packages:
sudo apt install ros-melodic-teleop-twist-keyboard 
sudo apt install ros-melodic-navigation
sudo apt install ros-melodic-amcl



How to Launch Basic Monte Carlo Position

Run "Roscore"

1. Open the map with launch neo_track2 
roslaunch neo_simulation neo_track2.launch 

2. Spawn the Robot 
roslaunch neo_simulation mpo_700_spawn.launch

3. Load the Map 
rosrun map_server map_server Neobotix/src/neo_simulation/maps/neo_track2.yaml
4. Run cob_scan_unifier <<< it can be done on the spawnfile 
roslaunch neo_simulation mpo_700_scan_unifier.launch 

5. Launch MonteCarlo simulation 
roslaunch neo_simulation mpo_700_amcl.launch

6. Launch RVIZ 
roslaunch neo_simulation mpo_700_rviz.launch
