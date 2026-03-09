#!/bin/bash
# Layout:
#   Pane 0 : ros1_bridge  (source noetic + galactic)
#   Pane 1 : lidar_guard.py
#   Pane 2 : udp_cmd_relay.py

ROBOT_WS="$HOME/660610822_ws"
GUARD="$ROBOT_WS/src/agv_robot/agv_robot/lidar_guard.py"
RELAY="$ROBOT_WS/src/agv_robot/agv_robot/udp_cmd_relay.py"

SESSION=agv_robot_660610822
tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION -x 220 -y 50

# ── Pane 0: ros1_bridge ──────────────────────────────────────────
tmux send-keys -t $SESSION:0.0 "echo '=== Pane 0: ros1_bridge (Noetic <-> Galactic) ==='" C-m
tmux send-keys -t $SESSION:0.0 "source /opt/ros/noetic/setup.bash" C-m
tmux send-keys -t $SESSION:0.0 "source /opt/ros/galactic/setup.bash" C-m
tmux send-keys -t $SESSION:0.0 "export ROS_DOMAIN_ID=0" C-m
tmux send-keys -t $SESSION:0.0 "ros2 run ros1_bridge dynamic_bridge --bridge-all-topics" C-m

# ── Pane 1: lidar_guard ──────────────────────────────────────────
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "echo '=== Pane 1: lidar_guard ==='" C-m
tmux send-keys -t $SESSION:0.1 "source /opt/ros/galactic/setup.bash" C-m
tmux send-keys -t $SESSION:0.1 "source $ROBOT_WS/install/setup.bash" C-m
tmux send-keys -t $SESSION:0.1 "export ROS_DOMAIN_ID=0" C-m
tmux send-keys -t $SESSION:0.1 "python3 $GUARD" C-m

# ── Pane 2: udp_cmd_relay ────────────────────────────────────────
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.2 "echo '=== Pane 2: udp_cmd_relay ==='" C-m
tmux send-keys -t $SESSION:0.2 "source /opt/ros/galactic/setup.bash" C-m
tmux send-keys -t $SESSION:0.2 "source $ROBOT_WS/install/setup.bash" C-m
tmux send-keys -t $SESSION:0.2 "export ROS_DOMAIN_ID=0" C-m
tmux send-keys -t $SESSION:0.2 "python3 $RELAY" C-m

tmux select-layout -t $SESSION:0 even-horizontal
tmux select-pane  -t $SESSION:0.0
tmux attach -t $SESSION