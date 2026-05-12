## Using Bag Recording

We can use ros2 bag which comes installed with ROS2 to record a set of predefined topics, one can specify them on the command line explicitly.

```
ros2 bag record <topic1> <topic2> … <topicN> -o <bag_name>
```

The topics we would need to record contain messages that are relevant to our unit test as well as transform `tf` and `tf_static` so for our example the syntax is and the switch `-o` allows us to specify a folder where data is recorded.  However, this records multiple messages until we terminate. And for a unit test we often just need one message from each topic. This is why we have also supplied a custom logger that only logs one message which you can run with

We can record data from any folder to a bag, and this will record continuously until we CTRL+C the recording, there is no current mechanism to stop recording simply after one message is received. Therefore start recording and be prepared to stop very quickly. We use below command which has all the topics of interest and we specify data to be stored in `<bag_name>` which is an entire folder. If we do not provide a folder name it is a random name generated `ros2_XXXXXXX` and the folder name is embedded inside the data which means we can not simply rename the folder subsequently.  The below will record four topics into a bag called `positon2`

```bash
ros2 bag record /drone/laserscan /drone/gt_odom /tf /tf_static -o position2
```

Stop the recording as soon as your see 

```bash
[rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...
```

This indicates the bag has all the data, we only need one message, hit `CTRL+C `immediately and it will stop recording. 

You can review contents via `ros2 bag info position2`, (the folder name depends on the folder name supplied initially). On my attempt I'm shown below. So it's almost 1MB for 3s of data.

```bash
Files:             position2_0.db3
Bag size:          723.8 KiB
Storage id:        sqlite3
Duration:          2.976s
Start:             May 10 2024 23:33:01.244 (1715347981.244)
End:               May 10 2024 23:33:04.220 (1715347984.220)
Messages:          842
Topic information: Topic: /drone/laserscan | Type: sensor_msgs/msg/LaserScan | Count: 65 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 671 | Serialization Format: cdr
                   Topic: /drone/gt_odom | Type: nav_msgs/msg/Odometry | Count: 104 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 2 | Serialization Format: cdr
```
