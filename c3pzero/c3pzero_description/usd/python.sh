#!/bin/bash

# Script to call the isaac launch system (see python.sh in the ISAAC_SCRIPT_DIR)

# ISAAC_SCRIPT_DIR="$HOME/.local/share/ov/pkg/isaac_sim-2022.2.1"
ISAAC_SCRIPT_DIR="/isaac-sim"

# Prepend the path to all arguments passed in and also pass the current directory
# to allow the launch script to find the USD files located here.
CUR_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
NEW_ARGS=""
for arg in "$@"
do
    NEW_ARGS="${NEW_ARGS} ${CUR_SCRIPT_DIR}/${arg}"
done

pushd ${ISAAC_SCRIPT_DIR}
./python.sh $NEW_ARGS $CUR_SCRIPT_DIR
popd
