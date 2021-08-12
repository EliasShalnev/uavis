#!/bin/bash

echo "Setting \"scout1\" local pose to \"z=100\"" &
rostopic pub /scout1/mavros/local_position/pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 100.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" -r 10 &


echo "Setting \"sim_p3at_1\" local pose to \"x=0.0 y=0.0\"" &
rostopic pub /sim_p3at_1/local_position geometry_msgs/Point "x: 0.0
y: 0.0
z: 0.0" -r 10 &


echo "Setting \"sim_p3at_2\" local pose to \"x=0.0 y=0.0\"" 
rostopic pub /sim_p3at_2/local_position geometry_msgs/Point "x: 30.0
y: 20.0
z: 0.0" -r 10 &


echo "Gazebo model_states publishing..."
rostopic pub /gazebo/model_states gazebo_msgs/ModelStates "name:
- 'sim_p3at_1'
- 'sim_p3at_2'
pose:
- position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
- position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
twist:
- linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
- linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0" -r 1 