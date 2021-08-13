#!/bin/bash

echo "Setting \"scout0\" local pose to \"x=0 y=0 z=500\"" &
rostopic pub /scout0/mavros/local_position/pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 100.0
    y: 200.0
    z: 2750.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" -r 10 &


echo "Setting \"p3at0\" local pose to \"x=30.0 y=20.0\"" &
rostopic pub /sim_p3at0/local_position geometry_msgs/Point "x: -300.0
y: -100.0
z: 0.0" -r 10 &


echo "Gazebo model_states publishing..."
rostopic pub /gazebo/model_states gazebo_msgs/ModelStates "name:
- 'p3at0'
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
twist:
- linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0" -r 1 