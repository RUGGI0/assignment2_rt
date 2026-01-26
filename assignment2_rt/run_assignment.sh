#!/bin/bash
set -e

SESSION="assignment2_rt"
WORKSPACE="/home/ubuntu/ros2_workshop"

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -c "$WORKSPACE"

common_setup="cd $WORKSPACE; source /opt/ros/jazzy/setup.bash; source install/setup.bash;"

# Base: split verticale (sinistra | destra)
tmux split-window -h -t "$SESSION":0

# Sinistra: split orizzontale (top | bottom)
tmux select-pane -t "$SESSION":0.0
tmux split-window -v

# Destra: split orizzontale (top | bottom)
tmux select-pane -t "$SESSION":0.1
tmux split-window -v

# Pane mapping:
# 0.0 = sinistra alto   -> Gazebo
# 0.2 = sinistra basso  -> shell/debug
# 0.1 = destra alto     -> safety
# 0.3 = destra basso    -> TELEOP

# 0.0 Gazebo + spawn + bridge + RViz
tmux select-pane -t "$SESSION":0.0
tmux send-keys "$common_setup ros2 launch bme_gazebo_sensors spawn_robot.launch.py rviz:=true x:=6.0 y:=6.0 yaw:=0.0" C-m

# 0.1 safety_node
tmux select-pane -t "$SESSION":0.1
tmux send-keys "$common_setup ros2 run assignment2_rt safety_node" C-m

# 0.2 shell/debug
tmux select-pane -t "$SESSION":0.2
tmux send-keys "$common_setup bash" C-m

# 0.3 TELEOP (in basso a destra)
tmux select-pane -t "$SESSION":0.3
tmux send-keys "$common_setup ros2 run assignment2_rt teleop_node" C-m

tmux attach -t "$SESSION"
