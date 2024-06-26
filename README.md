# Tiago navigation and obstacle detection

The goal of the project is to get the robot Tiago from the initial point Starting Pose to point Pose B by navigating through the environment that includes two rooms, corridor and obstacles. Once Tiago reaches the Pose B, it must recognize the obstacles, cylindrical in shape, from his surroundings and indicate how many there are and their location in the room.
Tiago can reach Pose B in three different ways:
 - The first involves using the motion control low that we developed to reach Pose B directly, thus never using the Navigation stack (Mode 1)
- The second involves using a motion control low that we developed to cross the corridor and then using the Navigation stack to reach Pose B (Mode 2)
- The third involves using the control low that is already there by taking advantage of the Navigation stack (Mode 3)

<div align="center">
  <img src="media/client.png" alt="Client terminal" title="Client terminal" />
  <p><em>Client terminal</em></p>
</div>

<div align="center">
  <img src="media/result.png" alt="Results terminal + Simulation (Gazeboo) + LaserData (Rviz)" title="Results terminal + Simulation (Gazeboo) + LaserData (Rviz)" />
  <p><em>Results terminal + Simulation (Gazeboo) + LaserData (Rviz)</em></p>
</div>
## Compiling istructions
Clone the environment and launch file from this [repository](https://github.com/PieroSimonet/tiago_iaslab_simulation.git)

You will need 4 terminal sourced in ~/catkin_ws

For terminal 1-2-3-4 run:

> start_tiago
> source /opt/ros/noetic/setup.bash 
> source /tiago_public_ws/devel/setup.bash

> catkin build  (one terminal is fine)
> source ~/catkin_ws/devel/setup.bash

## Running instructions

For terminal 1:
> roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library
(wait for the simulation started)

For terminal 2:
> roslaunch tiago_iaslab_simulation navigation.launch
(wait for prompted 'odom recieved')

For terminal 3:
> rosrun Assignment_1_group_12 server

For terminal 4:
> rosrun Assignment_1_group_12 client <x> <y> <yaw_angle>

## Members
The project was developed with the collaboration of Stefano Trenti and Matteo Villani
