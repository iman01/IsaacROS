chmod +x ./run_isaac.sh ./run_frontend.sh
tmux new-session -d -s mysession bash -c './run_isaac.sh; exec bash'
tmux split-window -h -t mysession bash -c './run_frontend.sh; exec bash'
tmux attach-session -t mysession