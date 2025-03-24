#!/bin/bash

### Description:
#
# Runs all Genom3 modules needed for running demo1

h2 init
sleep 1

roscore &
genomixd &
sleep 1
demo-ros &
sleep 1
rotorcraft-ros &
sleep 1
optitrack-ros &
sleep 1
pom-ros &
sleep 1
nhfc-ros &
sleep 1
mrsim-ros &
sleep 1
maneuver-ros &
