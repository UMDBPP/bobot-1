#!/bin/bash

# |***************************************************************|
# |* (c) Copyright 2023                                           |
# |* Balloon Payload Program, University of Maryland              |
# |* College Park, MD 20742                                       |
# |* https://bpp.umd.edu                                          |
# |***************************************************************|

# Temporary bash script to help the Flight Timer Node get started

# First, configure the node (set it into the configuration state)
ros2 service call /FlightTimer/change_state lifecycle_msgs/ChangeState "{transition: {id: 1}}"

# sleep for 5 seconds so that we don't try to change states too fast
sleep 5

# Set the node's state to be active
ros2 service call /FlightTimer/change_state lifecycle_msgs/ChangeState "{transition: {id: 3}}"

# sleep for 15 seconds
sleep 15

# After 15 seconds, shut down the Flight Timer node
ros2 service call /FlightTimer/change_state lifecycle_msgs/ChangeState "{transition: {id: 7}}"
