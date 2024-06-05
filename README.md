SY31
Commands

Launch directly with the config :

rviz -d config/rviz/config.rviz


Launch keyboard teleop to control the robot via the keyboard (once gazebo launched) :

rosrun turtlebot3_teleop turtlebot3_teleop_key

Nodes

Each Node can be launched separately for debug with

python3 XXX_node.py

To launch all nodes at once :

roslaunch ros_labyrinth_sy15 robot.launch
