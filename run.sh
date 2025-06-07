tmux new-session -d -s mysession bash -c './run_isaac.sh "$@"' _ "$@"
tmux split-window -h './run_frontend.sh'
tmux attach-session -d -t mysession