source /opt/ros/humble/setup.sh
echo "Running Isaac Lab"
eval "$(~/miniconda3/bin/conda shell.bash hook)" 
conda activate env_isaaclab
./isaaclab/isaaclab.sh -p simulation/loader.py "$@"
