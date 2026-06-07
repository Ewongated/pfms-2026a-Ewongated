#!/bin/bash
# ── A3 Racing — launch everything ────────────────────────────────────────────
# Usage: ./run_racing.sh               ← normal run
#        ./run_racing.sh record        ← normal run + record rosbags
#        ./run_racing.sh reset         ← kills everything and cleans ROS state
#        ./run_racing.sh record reset  ← reset then record

WS=~/ros2_ws
SOURCE="source $WS/install/setup.bash"
GOALS_FILE="$WS/install/a3_support/share/a3_support/data/A3_RACING.TXT"
DATA_DIR="$WS/src/pfms-2026a-Ewongated/scratch/a3_racing/data"

RECORD=false
if echo "$@" | grep -q "record"; then RECORD=true; fi

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
    sleep 1
    echo -n "Nodes:  "; ros2 node list  2>/dev/null || echo "none"
    echo -n "Topics: "; ros2 topic list 2>/dev/null || echo "none"
    echo "── Reset complete ──"
}

if echo "$@" | grep -q "reset"; then
    ros_reset
fi

if [ "${1}" = "reset" ] && [ -z "${2}" ]; then
    echo "── Run ./run_racing.sh to relaunch. ──"
    exit 0
fi

# ── Normal launch ─────────────────────────────────────────────────────────────
ros_reset  # Always reset before launching, matching original behaviour
eval "$SOURCE"
mkdir -p "$DATA_DIR"

# Terminal 1: Gazebo
gnome-terminal --title="Gazebo" -- bash -c "$SOURCE && ros2 launch pfms a3_racing.launch.py; exec bash" &

echo 'Waiting for Gazebo...'
until ros2 topic list 2>/dev/null | grep -q '/orange/odom'; do sleep 1; done
echo 'Gazebo up.'

# ── Record obstacle_clear BEFORE mission starts ───────────────────────────────
if [ "$RECORD" = true ]; then
    echo "── Recording obstacle_clear (5s) — car should be stationary, nothing ahead ──"
    rm -rf "$DATA_DIR/obstacle_clear"
    timeout 5 ros2 bag record /orange/laserscan /orange/odom \
        -o "$DATA_DIR/obstacle_clear" &
    wait $!
    echo "obstacle_clear done."

    echo "── Recording goal_in_corridor (5s) ──"
    rm -rf "$DATA_DIR/goal_in_corridor"
    timeout 5 ros2 bag record /orange/laserscan /orange/odom \
        -o "$DATA_DIR/goal_in_corridor" &
    wait $!
    echo "goal_in_corridor done."

    echo "── Recording goal_out_corridor (5s) ──"
    rm -rf "$DATA_DIR/goal_out_corridor"
    timeout 5 ros2 bag record /orange/laserscan /orange/odom \
        -o "$DATA_DIR/goal_out_corridor" &
    wait $!
    echo "goal_out_corridor done."

    echo "── Recording waypoint_visible (5s) ──"
    rm -rf "$DATA_DIR/waypoint_visible"
    timeout 5 ros2 bag record /orange/laserscan /orange/odom \
        -o "$DATA_DIR/waypoint_visible" &
    wait $!
    echo "waypoint_visible done."

    # Print odom so you can note the car position for utest.cpp
    echo "── Car position at recording time ──"
    ros2 topic echo /orange/odom --once 2>/dev/null | grep -A3 "position:"

    echo ""
    echo "══════════════════════════════════════════════════════"
    echo "Now place a box in Gazebo ~2m in front of the car:"
    echo "  ros2 run gazebo_ros spawn_entity.py -database unit_box -entity obstacle -x X -y Y -z 0.5"
    echo "Then press ENTER to record obstacle_blocked..."
    echo "══════════════════════════════════════════════════════"
    read -r

    echo "── Recording obstacle_blocked (5s) ──"
    rm -rf "$DATA_DIR/obstacle_blocked"
    timeout 5 ros2 bag record /orange/laserscan /orange/odom \
        -o "$DATA_DIR/obstacle_blocked" &
    wait $!
    echo "obstacle_blocked done."

    echo "── Removing obstacle ──"
    ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'obstacle'}" 2>/dev/null

    echo ""
    echo "══════════════════════════════════════════════════════"
    echo "Now move/rotate the car so the laser can't see both walls"
    echo "(e.g. teleop to face a wall end-on or open area)"
    echo "Then press ENTER to record waypoint_no_walls..."
    echo "══════════════════════════════════════════════════════"
    read -r

    echo "── Recording waypoint_no_walls (5s) ──"
    rm -rf "$DATA_DIR/waypoint_no_walls"
    timeout 5 ros2 bag record /orange/laserscan /orange/odom \
        -o "$DATA_DIR/waypoint_no_walls" &
    wait $!
    echo "waypoint_no_walls done."

    echo ""
    echo "── All bags recorded. Verifying... ──"
    for bag in obstacle_clear obstacle_blocked goal_in_corridor \
               goal_out_corridor waypoint_visible waypoint_no_walls; do
        COUNT=$(ros2 bag info "$DATA_DIR/$bag" 2>/dev/null | grep "Message count" | awk '{print $NF}')
        echo "  $bag: $COUNT messages"
    done
fi

# ── Launch racing node ─────────────────────────────────────────────────────────
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

# ── Record sample bag during mission ──────────────────────────────────────────
if [ "$RECORD" = true ]; then
    echo "── Recording sample bag during mission (10s) ──"
    rm -rf "$DATA_DIR/sample"
    timeout 10 ros2 bag record /orange/laserscan /orange/odom \
        -o "$DATA_DIR/sample" &
    BAG_PID=$!
    wait $BAG_PID
    echo "sample done."
fi

wait $NODE_PID
echo "racing_node exited with code $?"