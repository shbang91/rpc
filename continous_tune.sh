#!/bin/bash

while true; do
	source /home/carlos/anaconda3/bin/activate rpc
    # Run the Python script with the TIMESTEPS argument
    python -u simulator/pybullet/rl/rl_one_step/parameter_opti/tune_PPO.py 

    echo "Restarting script..."
    sleep 60  # Optional: adjust sleep time as needed
done