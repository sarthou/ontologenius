# Using a docker container to build ontologenius

This folder contains scripts and configuration files to use a docker container to build and run
ontologenius. It is based on ROS melodic with perception packages, but can be easily adapted to
other versions of ROS.

## Instructions

* ensure ontologenius is located in a path like *catkin_ws/src/ontologenius/*
* go into *ontologenius/.docker* folder
* run ```./docker-run.sh catkin_make```

