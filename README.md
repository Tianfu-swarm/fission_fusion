# Information_dissemination
A demo to verify how information is disseminated in a swarm system. 
The simulation environment used is ARGoS3, but the code development is primarily done with ROS. This is because we use a package similar to ros_bridge to connect ARGoS with ROS. 
By purposefully triggering information to drive directed dissemination, this study analyzes how effective information spread influences a swarm’s transition from random movement to aggregation.

# Start up

## Launch simulation environment

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3:/YOUR_WORKSPACE/PATH/src/information_dissemination/lib
export ARGOS_PLUGIN_PATH=/YOUR_WORKSPACE/PATH/src/information_dissemination/lib
argos3 -c /experiments/information_dissemination.argos
```

## Launch controller node
```bash
source install/setup.bash
ros2 launch information_dissemination run.launch.py
```

## Launch manual test robot
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=bot0/cmd_vel
```