# SY31


## Commands

Launch directly with the config :
``` bash
rviz -d config/rviz/config.rviz
```

---


Launch keyboard teleop to control the robot via the keyboard :
``` bash
rosrun turtlebot3_teleop turtlebot3_teleop_key
```

## Nodes

Each Node can be launched separately for debug with
``` bash
python3 XXX_node.py
```

To launch all nodes at once :
``` bash
roslaunch ros_labyrinth_sy15 robot.launch
```

```
catkin_make # Construit le workspace
source devel/setup.bash # Charge les nodes et messages dans le terminal
```

