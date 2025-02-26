#!/bin/bash
source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun my_package auto_nav_func.py

# wait for app to end
dt-launchfile-join