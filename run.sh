tmux new-session -d -s mysession './run_isaac.sh' "$@"
tmux split-window -h './run_frontend.sh'
tmux attach-session -d -t mysession