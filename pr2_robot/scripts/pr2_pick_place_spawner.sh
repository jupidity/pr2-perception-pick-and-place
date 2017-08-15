#! /bin/bash
# This script safely launches ros nodes for pick place project with buffer time to allow param server population
x-terminal-emulator -e roslaunch pr2_robot pick_place_project.launch & sleep 20 &&
x-terminal-emulator -e roslaunch pr2_robot  pick_place_perception.launch & sleep 1 &&
x-terminal-emulator -e rosrun pr2_robot marker_generation.py
