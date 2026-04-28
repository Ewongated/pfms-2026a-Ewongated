Week 10 Tutorial Questions
=========================

Symbolically link the `wk10_starter` folder supplied in the `ros2_ws` `src` folder. This allows us to have the folders and there respective packages in git so we can commit and backup to git) as well as part of the ros workspace. 

```bash
cd ~/ros2_ws/src 
ln –s <your_git_repo>/tutorials/week10/wk10_starter .
cd ~/ros2_ws/
```
Build the package using the `colcon build --symlink-install` command  in the terminal from the `ros2_ws` folder. It will ONLY work from this folder.

The `--symlink-install` portion of the command expedites install and ensures folders that do not need compilation are symbolically linked. 

## Using VSCode

If you want to examine the code in Visual Studio code, you need to open vscode from the source `ros2_ws` folder.  Instructions for working with ROS2 packages and vscode have been taken from [Erik Kramer's guide](https://github.com/ErickKramer/ros2_with_vscode/tree/humble).   To do so copy the contents of `tutorials/week10/vscode_ros` to your `.vscode` folder inside `~/ros2_ws`, this will enable to use tasks to debug and compile code! We will use this in class.

## General Examples

We have provided full ROS2 tutorial material on publishers and subscribers which is available in this weeks `wk10_start/topics` folder as two packages.  

Of the nodes supplied the ones that you should examine are:

```bash
wk10_starter/topics/minimal_subscriber/
   lambda.cpp          (most basic example of subscriber)
   member_function.cpp (example of subscriber in a class)
wk10_starter/topics/minimal_publisher/
   lambda.cpp          (most basic example of publisher)
   member_function.cpp (example of publisher in a class)
```

To compile code we need to use colcon, and this can ONLY be executed from workspace folder `~/ros2_ws` 

``` bash
cd ~/ros2_ws
colcon build --symlink-install
```

When compiling your code you will receive a warning below. You can ignore it on this occasion. This is because ROS already ships with the compiled examples of these two projects, we are adding the source code for viewing.

```bash
'examples_rclcpp_minimal_subscriber' is in: /opt/ros/humble
'examples_rclcpp_minimal_publisher' is in: /opt/ros/humble
If a package in a merged underlay workspace is overridden and it installs headers, then all packages in the overlay must sort their include directories by workspace order. Failure to do so may result in build failures or undefined behavior at run time.
If the overridden package is used by another package in any underlay, then the overriding package in the overlay must be API and ABI compatible or undefined behavior at run time may occur.

If you understand the risks and want to override a package anyways, add the following to the command line:
	--allow-overriding examples_rclcpp_minimal_publisher examples_rclcpp_minimal_subscriber
```



# Working with Laser Data

The folder `topics_masterclass` uses Laser data and the position of the Platform to compute closes point to platform. In the second example `quadcopter_control` inserting a way to directly send a goal to a `Quadcopter`.

We will be modifying the `week10_laser` package in `topics_masterclass` folder and  can run our code we have been developing using `ros2 run week10_laser sample` . When running code, do so with simulation `ros2 launch pfms a2.launch.py` . 

See the notes in `PfmsSample` class to complete this exercise.

## TASK 1: Subscribing to Laser and Processing Data
We need to modify the code so we can subscribe to the laser data from this platform. To do so Modify `PfmsSample`:

* In `PfmsSample` constructor subscribe to laser data (`sensor_msgs::msg::LaserScan`) on topic `orange/laserscan` and use `laser_callback` as the callback function.
* Add the member variable for the subscriber to the `PfmsSample`
* Add the callback function to the `PfmsSample` class (both the header as declaration as well as the cpp file as implementation)
* In your callback save the laserData on each callback to the `laserData_` and use the corresponding `dataMtx_`
* In the threaded function `process` a call is made to `LaserProcessing::closestPoint ` . Implement your code that finds the closest point in `LaserProcessing::closestPoint()`

As noted. this function is being called from `process` function, which is attached to a thread of execution independent from the callbacks. This is typical arrangement when we need to combine data from two sources. Inside the function we print the location of the nearest obstacle in the laser scan by calling `closestPoint`.

We have one mutex and a conditional variable as we need to coordinate to have both pieces of data at same time.

From the `ros2_ws` folder, to compile and build on the terminal execute: `colcon build --symlink-install --packages-select week10_laser` as this will build just this package.

You can run your ROS2 node in a terminal via `ros2 run week10_laser sample`

If you receive an error `Package 'week10_laser' not found` it usually means the package structure in ROS2 has not been re-indexed, you can execute `source ~/ros2_ws/install/setup.bash` just once to get ROS2 to be aware new packages exist.

## TASK 2: Point in Global Coordinates
We will now look at transferring the data into "world" coordinates. We need to take the position of the robot into consideration and attempt to publish the data in "world" reference frame.

As we want to utilise both sources of data (robot position and the laser that is on the robot), we have `PfmsSample::process()` which we will use to combine both sources of data and then compute the position in global coordinates.

Find the closest point in global coordinates {x,y} and print on screen.

# Working with Controlling a Platform

We will use `week10_quad` package in `quadcopter_control` folder and use `rosrun week10_quad sample` to run our code. 

You will need to update a few packages for this example

```bash
sudo apt update
sudo apt install ros-humble-sjtu-drone-bringup ros-humble-sjtu-drone-description
```

To run the simulator `ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py`  , you can also add `gui:=true` to see the environment (which will use another physical core of your PC) `ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py gui:=true`. 



TASK 3: Controlling the Quadcopter
-----------------------------------------------
We need to subscribe and publish to topics to control the quadcopter, and can do so in base/derived class `Controller` and `Quadcopeter` 

Consider:

* which topics do we need to send controls to the quadcopter to get it to take off and then to control it's motion (check topics vis `ros2 topic list`, inspect those that are from `sjtu_drone` )
  * What are the topic names and data types?

* which topics do we need to subscribe to, in order to determine where we are (odometry)? 
  * What are the topic names and data types?

We can also use a topic to give the quadcopter a single goal to go to and have already inserted the details required.

The syntax is, in the below what is the data type, can you use the CLI to find what it contains? What is the topic name?

```c++
this->create_subscription<geometry_msgs::msg::Point>(
        "/drone/goal", 5, std::bind(&Controller::setGoal,this,_1));
```

In order to accommodate controlling quadcopter we need to modify the Controller and Quadcopter class. 

Modify Controller:

* In Controller constructor subscribe to the drone odometry
* Add the member variable for the subscriber to the Controller  
* Add the callback function to the class (both in header as well as cpp file)

Modify Quadcopter:

* In Quadcopter constructor create publishers to send commands (control and takeoff) 
* Create the message in the `sendCmd` method and publish it (also consider to check if we should send takeoff)

From the `ros2_ws` folder, to compile and build on the terminal execute: `colcon build --symlink-install --packages-select week10_quad` as this will build just this package.

You can run your ROS2 node via `ros2 run week10_quad sample`

You can send goals directly to your Quadcopter code via terminal:

```bash
ros2 topic pub -1 /sjtu_drone/goal geometry_msgs/msg/Point "{x: 1.0, y: 2.0, z: 3.0}"
```


TASK 4: Subscribe to another topic
---------------------------

Let's receive information directly from rviz into this node, receive a [ClickedPoint from RViz](https://answers.ros.org/question/69019/how-to-point-and-click-on-rviz-map-and-output-the-position/) into this node

* Find the topic name `/goal_pose` in the list of topics and determine its type
* Create a callback for this topic name and correct type
* Print out the sequence number and x , y coordinates using `RCLCPP_INFO_STREAM`
* In the callback for `/goal_pose` assign the point to a goal, use 2m height for z at the point

Now we can click on rviz using the `publish point` tool, and the quadcopter will fly to this location .. neat! 

# Solutions

Our solutions after class are provided in `wk10_examples` subfolder. You need to symbolically link that subfolder to `~/ros2_ws/src`. ROS will not allow to have packages with the same name. Therefore our solutions will have  `_solution` added to the original package name.


[ROS Installation Instructions]: http://wiki.ros.org/ROS/Installation
[ROS Tutorials]: http://wiki.ros.org/ROS/Tutorials
[ROS TF]: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
[Polar to Cartesian]: https://www.mathsisfun.com/polar-cartesian-coordinates.html
