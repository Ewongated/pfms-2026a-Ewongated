#!/bin/bash
# ── A3 Racing — launch everything ────────────────────────────────────────────
# Usage: ./run_racing.sh
# Run this from any directory. Opens 5 tabs in the same terminal window.
# Requires gnome-terminal (standard on Ubuntu lab machines).

WS=~/ros2_ws
SOURCE="source $WS/install/setup.bash"
GOALS_FILE="$WS/install/a3_support/share/a3_support/data/A3_RACING.TXT"

#!/bin/bash
# ── A3 Racing — launch everything ────────────────────────────────────────────
# Usage: ./run_racing.sh

WS=~/ros2_ws
SOURCE="source $WS/install/setup.bash"
GOALS_FILE="$WS/install/a3_support/share/a3_support/data/A3_RACING.TXT"

echo "Killing any stale ROS/Gazebo processes..."
pkill -f "ros2" 2>/dev/null
pkill -f "ign gazebo" 2>/dev/null
pkill -f "ruby" 2>/dev/null
pkill -f "gzserver" 2>/dev/null
pkill -f "gzclient" 2>/dev/null
pkill -f "racing_node" 2>/dev/null
pkill -f "goals_publisher" 2>/dev/null
sleep 3
echo "Stale processes cleared."

echo "Starting Gazebo..."
gnome-terminal --title="Gazebo" -- bash -c "$SOURCE && ros2 launch pfms a3_racing.launch.py; exec bash" &

sleep 2
echo "Starting RacingNode..."
gnome-terminal --title="RacingNode" -- bash -c "sleep 6 && $SOURCE && ros2 run a3_racing racing_node; exec bash" &

sleep 1
echo "Starting Goals publisher..."
gnome-terminal --title="Goals" -- bash -c "sleep 10 && $SOURCE && ros2 run a3_support goals_publisher --ros-args -r goals:=/orange/goals -p filename:=$GOALS_FILE; exec bash" &

sleep 1
echo "Starting Mission..."
gnome-terminal --title="Mission" -- bash -c "sleep 14 && $SOURCE && ros2 service call /orange/mission std_srvs/srv/SetBool '{data: true}'; exec bash" &

sleep 1
echo "Opening spare shell..."
gnome-terminal --title="Shell" -- bash -c "$SOURCE; exec bash" &

echo "All terminals launched."