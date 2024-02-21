
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
The project was developed with the collaboration of Stefano Trenti and Matteo Viallani
