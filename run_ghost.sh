
source /opt/ros/humble/setup.sh
eval "$(~/miniconda3/bin/conda shell.bash hook)" 
conda activate env_isaaclab
tmux new-session -d -s mysession bash -c 'python3 simulation/loader.py --ghost-opacity 0.3; exec bash'
tmux split-window -h -t mysession bash -c 'python3 frontend/steering_emulator_with_modes.py; exec bash'
tmux attach-session -t mysession