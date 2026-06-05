#!/bin/bash
# ── A3 Racing — launch everything ────────────────────────────────────────────
# Usage: ./run_racing.sh

WS=~/ros2_ws
SOURCE="source $WS/install/setup.bash"
GOALS_FILE="$WS/install/a3_support/share/a3_support/data/A3_RACING.TXT"

echo "Killing any stale ROS/Gazebo processes..."
pkill -f "ros2" 2>/dev/null
pkill -f "ign gazebo" 2>/dev/null
pkill -f "racing_node" 2>/dev/null
pkill -f "goals_publisher" 2>/dev/null
sleep 3
echo "Stale processes cleared."

echo "Building..."
cd $WS
colcon build --packages-select a3_racing
source $WS/install/setup.bash
echo "Build complete."

# Terminal 1: Gazebo
gnome-terminal --title="Gazebo" -- bash -c "$SOURCE && ros2 launch pfms a3_racing.launch.py; exec bash" &

# Terminal 2: RacingNode + goals + mission
gnome-terminal --title="RacingNode" -- bash -c "
  $SOURCE
  echo 'Waiting for Gazebo...'
  sleep 10
  echo 'Starting racing_node...'
  ros2 run a3_racing racing_node &
  NODE_PID=\$!
  echo 'Waiting for racing_node to be ready...'
  until ros2 node list 2>/dev/null | grep -q racing_node; do
    sleep 1
  done
  echo 'racing_node is up.'
  echo 'Waiting for /orange/odom to be published...'
  until ros2 topic info /orange/odom 2>/dev/null | grep -q 'Publisher count: [1-9]'; do
    sleep 1
  done
  echo 'Odometry is live.'
  echo 'Waiting for /orange/goals subscriber to be ready...'
  until ros2 topic info /orange/goals 2>/dev/null | grep -q 'Subscription count: [1-9]'; do
    sleep 1
  done
  echo 'Subscriber ready. Publishing goals...'
  ros2 run a3_support goals_publisher --ros-args \
    -r goals:=/orange/goals \
    -p filename:=$GOALS_FILE &
  echo 'Waiting for goals to be received...'
  sleep 5
  echo 'Starting mission...'
  ros2 service call /orange/mission std_srvs/srv/SetBool '{data: true}'
  wait \$NODE_PID
  exec bash" &

# Terminal 3: spare shell
gnome-terminal --title="Shell" -- bash -c "$SOURCE; exec bash" &

echo "All terminals launched."