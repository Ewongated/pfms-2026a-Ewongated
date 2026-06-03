Week 12 Tutorial Questions
=========================

Work through these questions and make sure you understand what is going on in each example. 
If you have any questions about the material, please raise them in the next lab session.

This week's lab questions will make use of `a3_support` package showcasing `ros parameters` and GOALS you can use for your A3 testing. It will also leverage the `project_example` package to assist you with assignment 3.

Of course, you need to these packages to your ros2 workspace. You should know the syntax of the command thus far (the `ln -s` syntax) we have done this in week 10,11, quiz4 and quiz5 thus far. As a reminder it's below. Replace `<your_github>` with the location of your github.

```bash
cd ~/ros2_ws/src
ln -s ~/<your_github>/tutorials/week12/a3_support
```

This week we examine:

* ROS parameters and Visualisation
* Working with ROS bags and Unit Testing
* Loading existing maps in rviz

## ROS parameters and remapping, visualisation

You have been supplied a `a3_support` package that has two nodes. 

### ROS parameters and remapping

The `goal_publisher` node loads goals from a file supplied and publishes them on two topics. Here we introduce [ros parameters](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html) and also look at visualisation messages. This package contain GOALS that can be used for testing your algorithms towards Assignment 3. 

If we decompose the `goals_publisher` we can see that it looks at obtaining parameters that are passed on command line (or saved on the system). Specifically:

```c++
auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
param_desc.description = "Filename to load goals";
node->declare_parameter("filename", default_filename, param_desc);
std::string filename = node->get_parameter("filename").as_string();
RCLCPP_INFO_STREAM(node->get_logger(),"file name with goals to be saved:" << filename);
```

The line `node->declare_parameter("filename", default_filename, param_desc);` tries to find `filename` being passed to this node as a **parameter** and saves it to `param_desc` where `default_filename` is used if a parameter is not provided (default value).  In our code we thereafter open the filename and read information about goals from  the file. **Note** that the topic name the data to be published is supplied when declaring the publisher.

```c++
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goals_pub = 
	node->create_publisher<geometry_msgs::msg::PoseArray>("goals", 100);
```

If we run `ros2 run a3_support goals_publisher` it will publish `GOALS.TXT` which are the default goals in the data directory to `/goals` topic. However if we wish to publish different set of goals,  we can supply them on command line a remapping such as:

```bash
ros2 run a3_support goals_publisher --ros-args -p filename:=$HOME/ros2_ws/install/a3_support/share/a3_support/data/A3_RACING.TXT
```

The path here is related to my system (where my ros2_ws is in my home directory)  

If we wish to publish to a different topic then we can start the node with a **remapping** argument `--ros-args --remap` followed by the remapping we wish to achieve `existing_topic:=new_topic` so if we wanted to publish to `/mission/goals` instead of `goals`we need to start on command line such as

```bash
ros2 run a3_support goals_publisher --ros-args --remap goals:=/mission/goals -p filename:=$HOME/ros2_ws/install/a3_support/share/a3_support/data/A3_RACING.TXT
```

Run any of the launch files for a3 and: 

- [ ] run goals_publisher by supplying the A3_SEARCH.TXT goals instead of GOALS.txt using the ros parameters
- [ ] view what rviz is showing and indicate how many goals are diplayed
- [ ] display in terminal the goals, that are published as a `PoseArray`  (use the ROS CLI)

### Visualisation

Let's have a look at the visualisation aspects, where we use visualisation_msgs. In the `goal_publisher` we sent the goals to rviz via the visualisation_msgs by assembling an Array of Markers. The syntax is

```c++
visualization_msgs::msg::Marker marker;

// We need to set the frame, usually using world
marker.header.frame_id = "world";
// Here we assign the current time
marker.header.stamp = stamp;

//We set lifetime (it will dissapear in this many seconds)
marker.lifetime = rclcpp::Duration(1000,0); //zero is forever

// Set the namespace and id for this marker.  This serves to create a unique ID
// Any marker sent with the same namespace and id will overwrite the old one
marker.ns = "goals"; //This is namespace, markers can be in diofferent namespace
// We need to keep incrementing markers to send others ... 
marker.id = ct++; 

// The marker type
marker.type = visualization_msgs::msg::Marker::CUBE;

// Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
marker.action = visualization_msgs::msg::Marker::ADD;

// x,y ,z location
marker.pose.position.x = pt.x;
marker.pose.position.y = pt.y;
marker.pose.position.z = pt.z;

//Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;

// Set the scale of the marker -- Here we use equal sides, think about cylinders
marker.scale.x = 1.0;
marker.scale.y = 1.0;
marker.scale.z = 1.0;

//Let's send a marker with color (green for reachable, red for now)
std_msgs::msg::ColorRGBA color;
color.a=0.5;//a is alpha - transparency 0.5 is 50%;
color.r=0;
color.g=1.0; // We picked green (1.0 is full color - sometimes colour is 0-255 here 0-1)
color.b=0;

marker.color = color;
markerArray.markers.push_back(marker);
```

Can you modify the code to

- [ ] Send a cylinder 1.5m height and 0.5m radius
- [ ] Send a Red object (the current one is green)
- [ ] Change the namespace to be "test"
- [ ] Can you enable your A3 submission to receive the PoseArray and print it on screen

## ROS Bag

We have selected for this demo the Husky laser data and look at detecting either the number of segments or the closest point for instance.

 In order to record topic we need to have the simulator working, let's use the terrain project (project 1) from A3:

```bash
ros2 launch pfms a3_terrain.launch.py
```

We need some "ground truth", being that we need to record data for our unit test, thie data can be recorded into a bag while rviz can be used to inpscte the simulation and get "ground truth" using UI tools.

Once the launch file is running we can see laser data from the husky in `rviz2` as below.

<img src="./pics/rviz_laser_data.png" style="zoom:67%;" />

You can in another terminal run `ros2 topic echo /goal_pose` which will show you the outcome of interacting with the **2D Goal Pose** tool in RVIZ.

We can count the **number of segments** here to be 13, and we can also use the **2D Goal Pose** tool  and click on any location and also swing around that point while holding the mouse to have an orientation (not necessarily required for any of the projects).  The output of this clicking and the release will show the point in terminal such as below (the reference frame is `world` coordinates).

```
header:
  stamp:
    sec: 1715347100
    nanosec: 302426412
  frame_id: world
pose:
  position:
    x: 3.8024001121520996
    y: 3.645364284515381
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: -0.6661770373657229
    w: 0.7457936409534667
```

### Using single_message_recorder (Recommended for Unit Tests)

The default recording mechanism is via ros2 bag record [explained here](./ROS2BAG.md). The approach creates very large data (in 3s woudl be ~1MB) is awkward and if recording is too short will not get all data. To solve this, the `a3_support` package provides `single_message_recorder` - a custom tool that automatically records exactly **one message per topic** (except `/tf` which records continuously for 2 seconds to ensure proper frame transforms in RViz).

This tool is specifically designed for creating deterministic, small bag files suitable for unit testing. It reads topic names from a configuration file and automatically stops after collecting the required data. **You will use this tool extensively in Assignment 3** to create test data for your unit tests.

**Why use single_message_recorder?**

- **Deterministic**: Always records exactly one message per sensor topic
- **Automatic timing**: No need to hit CTRL+C at the right moment
- **Includes transforms**: Automatically records `/tf` and `/tf_static` for proper coordinate frame handling
- **Small file sizes**: Perfect for unit tests (typically 40-50 KiB vs 700+ KiB with manual recording)
- **Configurable**: Easy to specify which topics to record via YAML file

**Configuration:**

The topics to record are specified in `config/bag_params.yaml`:

```yaml
single_message_recorder:
  ros__parameters:
    topics:
      - /sjtu_drone/laserscan
      - /sjtu_drone/odom
      - /tf
      - /tf_static
```

**Usage:**

From your package's `data` folder (for instance)

```bash
cd ~/ros2_ws/src/a3_submission/data
ros2 run a3_support single_message_recorder --ros-args --params-file $HOME/pfms_ws/src/a3_support/config/bag_params.yaml
```

The output will show exactly what's being recorded:

```
[INFO] [rosbag2_storage]: Opened database 'sample_2026-05-11_22-20-45/sample_2026-05-11_22-20-45_0.db3' for READ_WRITE.
[INFO] [single_message_recorder]: Recording to bag file: sample_2026-05-11_22-20-45
[INFO] [single_message_recorder]: Recorded message from /tf_static
[INFO] [single_message_recorder]: Recorded message from /sjtu_drone/odom
[INFO] [single_message_recorder]: First snapshot recorded, will record /tf for 2 more seconds...
[INFO] [single_message_recorder]: Recording /tf messages... (14 total)
[INFO] [single_message_recorder]: Recorded message from /sjtu_drone/laserscan
[INFO] [single_message_recorder]: Recording /tf messages... (45 total)
[INFO] [single_message_recorder]: Finished recording 48 messages
```

**Verify the bag:**

```bash
ros2 bag info sample_2026-05-11_22-20-45
```

Expected output showing small, clean data:

```
Files:             sample_2026-05-11_22-20-45_0.db3
Bag size:          41.2 KiB
Storage id:        sqlite3
Duration:          1.320464838s
Start:             May 11 2026 22:20:45.093315941 (1778502045.093315941)
End:               May 11 2026 22:20:46.413780779 (1778502046.413780779)
Messages:          48
Topic information: Topic: /sjtu_drone/laserscan | Type: sensor_msgs/msg/LaserScan | Count: 1 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 45 | Serialization Format: cdr
                   Topic: /sjtu_drone/odom | Type: nav_msgs/msg/Odometry | Count: 1 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr
```

Notice: Only **1 message** per sensor topic, but **45 /tf messages** for smooth transforms in RViz.

**Customizing topics:**

You can copy the config file to your package and modify it for your specific needs:

```bash
cp $HOME/pfms_ws/src/a3_support/config/bag_params.yaml ~/ros2_ws/src/a3_submission/config/
# Edit the file to change topics
# Then use: --params-file $HOME/ros2_ws/src/a3_submission/config/bag_params.yaml
```

**Note:** All bag recording commands (`ros2 bag record`, `single_message_recorder`, `ros2 bag info`, `ros2 bag play`) should be executed from the `data` folder inside your package. This ensures bags are stored in the correct location for distribution with your code.

**Comparing the two recording methods:**

| Aspect | Manual `ros2 bag record` | `single_message_recorder` |
|--------|-------------------------|---------------------------|
| File size | ~700+ KiB | ~40-50 KiB |
| Timing | Manual CTRL+C required | Automatic |
| Messages per sensor topic | 50-100+ | Exactly 1 |
| /tf messages | 600+ | ~45 (2 seconds) |
| Use case | General purpose | Unit testing |

Use `ros2 bag info <folder>` to verify which method was used - look at the message counts!

### Replaying and Visualizing Bag Files

You need to **terminate the launch file** before playing back bags. To replay the bag (works with both manual and single_message_recorder bags):

```bash
ros2 bag play -r 0.1 -l position2
```

This replays at `-r` 0.1× speed in a `-l` loop.

**For proper RViz visualization with transforms**, use the QoS override file:

```bash
ros2 bag play -l sample_2026-05-11_22-20-45/ --qos-profile-overrides-path $HOME/pfms_ws/src/a3_support/config/qos_override.yaml
```

The QoS override ensures `/tf` messages are received correctly by RViz, allowing proper coordinate frame transformations.

You can visualise the bag by running `rviz2` in another terminal and then in the menu under `Files/Recent Configs` finding the rviz file related to your project.

If you want to view the terrain directly in rviz2, this is possible with project 1 and 3 via (supply the number as the parameter)

```bash
ros2 run a3_support stl_publisher --ros-args -p project:=1
```



<img src="./pics/rviz_laser_data2.png" style="zoom: 67%;" />



## A3 Skeleton Package

The example skeleton package 

- Create nodes and communicate via topics and services

- Separate ROS "plumbing" from processing logic

- Write thread-safe libraries with unit tests

- Use timers, callbacks, and threading patterns

- Work with laser scan data and visualization markers

- The package includes sample bag data in data/ for testing.

Let's have a look at current unit test, you will need to modify/create a new test (refer `test/utest.cpp`) folder in your package.

In the existing unit test `CountSegments` you will find

```c++
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("a3_skeleton");//If you change package name you need to change this
    std::string bag_filename=package_share_directory + "/data/sample"; // The data for the bag are in the data folder and each folder (such as sample) has a bag file, that you can record
```

Also, in the unit test we are comparing against a ground truth we obtained. You will see a lines ` EXPECT_EQ(segments,10);` which compare the result of `unsigned int segments = laserProcessing.countSegments();` Well, if you had an algorithm that counted segments, and you know there were 13 this part would be

```c++
unsigned int segments = laserProcessing.countSegments();
EXPECT_EQ(segments,13);
```

And you could also use the x,y we captured if that was being determine by your algorithm `x: 3.8` and ` y: 3.6` for instance.

Try to move skidsteer using `rqt`, via the `gui` or `comand_skidsteer.` Stop the platform and then complete following.

- [ ] Record a ROS bag with data related to platfrom using `single_message_recorder` (recommended) 
- [ ] Obtaining ground truth of number of segments
- [ ] Use `ros2 bag info` to verify the bag contains the expected topics and message counts
- [ ] Replaying the bag back with QoS overrides, visualising it in RViz and taking another look at the data
- [ ] Creating another unit test, one that extracts both laser and odometry from the bag
- [ ] Answer: where is the ros bag going to be deposited on your system after you compile the code?
- [ ] How could you read odometry and laser for the unit test? 
- [ ] Can you create a test to find the closest object in laser, transfer into world coordinates (HINT: add another library) and then compare with the independent method of getting points from rviz

To compile and run the unit tests developed, you need to use the name of your package,. So, if your package is called `a3_skeleton` the syntax is:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select a3_skeleton
colcon test --event-handlers console_cohesion+ --packages-select a3_skeleton
```

## 	