#!/bin/bash

### Description:
#
# Runs all Genom3 modules needed for running demo1

h2 init
sleep 1

# Get the current script directory
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Navigate to the CAMP folder by going up three levels
CAMP_DIR=$(realpath "$SCRIPT_DIR/../../../../")

# Construct the relative path to the binary
BINARY_PATH_NHFC="$CAMP_DIR/src/simulation_genom/genom_custom/bin/nhfc-pocolibs"
BINARY_PATH_MAN="$CAMP_DIR/src/simulation_genom/genom_custom/bin/maneuver-pocolibs"

genomixd &
rotorcraft-pocolibs -f &
optitrack-pocolibs -f &
pom-pocolibs -f &
"$BINARY_PATH_MAN" -f &
mrsim-pocolibs -f &
"$BINARY_PATH_NHFC" -f &