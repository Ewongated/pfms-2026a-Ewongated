#!/bin/bash
# ── A3 Racing — launch everything ────────────────────────────────────────────
# Usage: ./run_racing.sh
#        ./run_racing.sh reset   ← kills everything and cleans ROS state
WS=~/ros2_ws
SOURCE="source $WS/install/setup.bash"
GOALS_FILE="$WS/install/a3_support/share/a3_support/data/A3_RACING.TXT"

ros_reset() {
    echo "── Stopping all ROS/Gazebo processes ──"
    pkill -f ros2; pkill -f gazebo; pkill -f gzserver; pkill -f gzclient; pkill -f rviz2
    sleep 1
    echo "── Clearing ROS daemon ──"
    ros2 daemon stop 2>/dev/null
    pkill -f "ros2 daemon" 2>/dev/null
    rm -rf ~/.ros/daemon*
    echo "── Clearing FastDDS shared memory ──"
    rm -rf /dev/shm/fast* /dev/shm/fastrtps* /tmp/fast*
    echo "── Restarting daemon ──"
    source "$WS/install/setup.bash"
    ros2 daemon start
    echo "── Verifying clean state ──"
    sleep 1
    echo -n "Nodes:  "; ros2 node list  2>/dev/null || echo "none"
    echo -n "Topics: "; ros2 topic list 2>/dev/null || echo "none"
    echo "── Reset complete ──"
}

if [ "${1}" = "reset" ]; then
    ros_reset
    echo "── Run ./run_racing.sh to relaunch. ──"
    exit 0
fi

# ── Normal launch ─────────────────────────────────────────────────────────────
ros_reset

# Terminal 1: Gazebo (separate window)
gnome-terminal --title="Gazebo" -- bash -c "$SOURCE && ros2 launch pfms a3_racing.launch.py; exec bash" &

# ── RacingNode runs in the calling shell (VS Code terminal) ──────────────────
eval "$SOURCE"

echo 'Waiting for Gazebo...'
until ros2 topic list 2>/dev/null | grep -q '/orange/odom'; do sleep 1; done
echo 'Gazebo up.'

ros2 run a3_racing racing_node &
NODE_PID=$!

until ros2 node list 2>/dev/null | grep -q racing_node; do sleep 1; done
echo 'racing_node up.'

until ros2 topic info /orange/odom 2>/dev/null | grep -qE 'Publisher count: [1-9][0-9]*'; do sleep 1; done
echo 'Odometry live.'

until ros2 topic info /orange/goals 2>/dev/null | grep -qE 'Subscription count: [1-9][0-9]*'; do sleep 1; done
echo 'racing_node subscribed to goals.'

MISSION_STARTED=false
while [ "$MISSION_STARTED" = false ]; do
    echo 'Launching goals_publisher...'
    ros2 run a3_support goals_publisher --ros-args \
      -r goals:=/orange/goals \
      -p filename:=$GOALS_FILE &
    GP_PID=$!
    sleep 2
    RESULT=$(ros2 service call /orange/mission std_srvs/srv/SetBool '{data: true}' 2>&1)
    echo "$RESULT"
    if echo "$RESULT" | grep -qi 'success=True'; then
        MISSION_STARTED=true
        echo 'Mission started.'
    else
        echo 'Goals not accepted yet -- retrying...'
        kill $GP_PID 2>/dev/null
        sleep 1
    fi
done

wait $NODE_PID
echo "racing_node exited with code $?"