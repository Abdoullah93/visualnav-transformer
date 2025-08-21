#!/bin/bash
# tmux helper to run ViNT with ROS 2 + Gazebo Harmonic
# Usage:
#   ./navigate_ros2.sh --model <vint|nomad|...> --dir <topomap_dir> [--goal-node -1] [--image-topic /camera/image_raw] [--waypoint-topic /waypoint] [--world /path/to.world]

session_name="vint_ros2_$(date +%s)"
WORLD=""

# Parse --world arg to start gz sim
ARGS=()
while [[ $# -gt 0 ]]; do
  case $1 in
    --world)
      WORLD="$2"; shift 2;;
    *)
      ARGS+=("$1"); shift;;
  esac
done

# New session
if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux not found. Please install tmux or run the processes manually." >&2
  exit 1
fi

tmux new-session -d -s "$session_name"

# Layout: 4 panes
 tmux selectp -t 0
 tmux splitw -h -p 50
 tmux selectp -t 0
 tmux splitw -v -p 50
 tmux selectp -t 2
 tmux splitw -v -p 50
 tmux selectp -t 0

# Pane 1: Gazebo Harmonic (gz sim)
 tmux select-pane -t 0
 if [[ -n "$WORLD" ]]; then
   tmux send-keys "gz sim -v 4 \"$WORLD\"" Enter
 else
   tmux send-keys "echo 'Tip: pass --world /path/to.world to start Gazebo here'" Enter
 fi

# Pane 2: Joy node (ROS 2)
 tmux select-pane -t 1
 tmux send-keys "ros2 run joy joy_node" Enter

# Pane 3: ViNT navigation node (ROS 2)
 tmux select-pane -t 2
 tmux send-keys "python3 navigate_ros2.py ${ARGS[*]}" Enter

# Pane 4: PD controller (ROS 2)
 tmux select-pane -t 3
 tmux send-keys "python3 pd_controller_ros2.py" Enter

# Attach
 tmux -2 attach-session -t "$session_name"
