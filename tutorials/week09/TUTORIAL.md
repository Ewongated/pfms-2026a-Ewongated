Week 9 Tutorial Questions
=========================

ROS2 is a middleware, each executable is also called a node, and communicates with other nodes via topics and services (topics - one way communication like a produce/consumer; services - two way communication with a guaranteed return value).

One of the larger changes between ROS1 and ROS2 is the way the nodes establish communication. In ROS1 this was done via a central server called `roscore`. ROS2 does not have a central server, rather the nodes use `avahi` to discover topics and services.

You need to look at the canvas pre-work for week 9:

1. Shows [using turtlesim, ros2, and rqt](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)

2. Examine [looking at nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

3. [Understanding topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

4. [Understanding services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)

5. ROS [logging mechanism](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html)

Now we can use this knowledge to explore the ROS2 ecosystem used for Assessment 2 simulator. 

Exploring the Command Line Interface (CLI)
----------------------------

Unlike the tutorial on canvas thus far, instead of runnign one node at a time we use `launch` to stat several `nodes` at once.

```bash
ros2 launch gazebo_tf a2.launch.py
```

The simulator has two robots (Ackerman and Skidsteer) equipped with sensors (Laser and a Sonar Array -number of Sonars).  All ROS2 CLI command contain `ros2` as the first argument, as noted above, refer [CANVAS Week 09 Prework](https://canvas.uts.edu.au/courses/30581/pages/week-09-prework) if your uncertain about the commands

### Questions And Tasks ###

* What executable(s) are part of the `gazebo_tf` package?

* What nodes are running?

* Access more information `info` about the `/orange/reach` node?

* Access more information `info` about the `/husky/pose_to_tf` node?

* What are the topics available?

* Name two differences between the `/orange/odom` and  `/husky/laserscan` topics?

* Print the contents on the `/orange/odom` topic to screen using `ros2 topic echo <topic_name>`

* What are the fields in this message? HINT use `ros2 interface show <msg_name>`

* What message type is being sent on  `/husky/cmd_vel`? (HINT: use `ros2 topic info` and `ros2 interface show`

* Send a linear velocity of 0.1 m/s to go up via the `/drone/cmd_vel`  using `rostopic pub` (HINT `ros2 topic pub -h` can tell you how to send data at a specific rate, the below would send all zeros 10 times.)

  ```bash
  ros2 topic pub -r 10 /drone/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
  ```

* What message is being sent to topic `/orange/register_goals`

* Send two goals to this topic

```bash
ros2 topic pub --once /orange/register_goals geometry_msgs/msg/PoseArray "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'}, poses: [{position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}]}"
```

* List all ROS services available

* Let's use `/orange/check_goals` , what does the below show?

```bash
ros2 service type /orange/check_goals
```

* Let's ask if the orange audi has reached the goals

```bash
ros2 service call /orange/check_goals std_srvs/srv/Trigger
```

* Use rqt_console to view the log from `audi_reach`, can you change the default log level?

* Can you use rqt to show the linear velocity in x direction for  `/orange/odom` 
