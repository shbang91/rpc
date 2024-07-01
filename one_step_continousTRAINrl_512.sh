#!/bin/bash

while true; do
    # Determine the current timestep value (you need to implement this logic)
    # For demonstration, let's assume it's stored in a variable named $timestep
	CURR_TIMESTEP=$(cat timesteps_512.txt)
    echo "hello"
	echo "Current timestep: $CURR_TIMESTEP"
	source /home/carlosaj/miniconda3/bin/activate rpc
    # Run the Python script with the TIMESTEPS argument
    python -u simulator/pybullet/rl/rl_one_step/train_512.py --timesteps "$CURR_TIMESTEP"

    echo "Restarting script..."
    sleep 60  # Optional: adjust sleep time as needed
done
