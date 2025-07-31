#!/bin/bash

# Source ROS 2 and conda
source /opt/ros/humble/setup.sh
eval "$(~/miniconda3/bin/conda shell.bash hook)"
conda activate env_isaaclab

echo "Running frontend with shared process group..."

# Run commands in background process group
setsid bash -c '
    python3 frontend/steering_emulator_with_modes.py &
    python3 frontend/main.py
    wait
' &
PGID=$!

# Make sure this script's shell is in its own process group
# (so kill -- -$PGID kills that group)
trap "echo 'Stopping frontend...'; kill -- -$PGID; wait $PGID" SIGINT SIGTERM

# Wait for processes to finish
wait $PGID
