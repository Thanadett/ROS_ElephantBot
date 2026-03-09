#!/bin/bash     
SESSION=agv_robot_660610822
tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION -x 220 -y 50

# ─── Pane 0: ROS1 Noetic ↔ ROS2 Galactic bridge ─────────────────
# ต้อง source noetic ก่อน galactic เสมอ
tmux send-keys -t $SESSION:0 "echo '=== Pane 0: ros1_bridge (Noetic↔Galactic) ==='" C-m
tmux send-keys -t $SESSION:0 "source /opt/ros/noetic/setup.bash" C-m
tmux send-keys -t $SESSION:0 "source /opt/ros/galactic/setup.bash" C-m
tmux send-keys -t $SESSION:0 "export ROS_DOMAIN_ID=0" C-m
tmux send-keys -t $SESSION:0 "ros2 run ros1_bridge dynamic_bridge --bridge-all-topics" C-m

# ─── Pane 1: Robot ROS2 Galactic nodes ───────────────────────────
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "echo '=== Pane 1: Robot ROS2 nodes (Galactic) ==='" C-m
tmux send-keys -t $SESSION:0.1 "source /opt/ros/galactic/setup.bash" C-m
tmux send-keys -t $SESSION:0.1 "source ~/660610822_ws/install/setup.bash" C-m
tmux send-keys -t $SESSION:0.1 "export ROS_DOMAIN_ID=0" C-m
tmux send-keys -t $SESSION:0.1 "ros2 launch agv_bringup robot.launch.py" C-m

tmux select-layout -t $SESSION:0 even-horizontal
tmux select-pane  -t $SESSION:0.0
tmux attach -t $SESSION
