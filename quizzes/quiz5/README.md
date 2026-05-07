Quiz 5
======

General Information
------
The `quiz5` folder contains all **five tasks** (there is no folder *b*).

- **Tasks 1–3**: Unit tests are provided.
- **Task 4**: The test is withheld and will be used for marking after submission.
- **Task 5**: Testing is executed separately from the command line.

To develop the quiz, create a **symbolic link** from the `quiz5` folder to your `~/ros2_ws/src` directory. The exact path will differ for each student, depending on where their repository is located.

For example, if your `quiz5` folder is located at: `~/git/pfms-2026a-buzz/quizzes/quiz5/`you would execute:

```bash
cd ~/ros2_ws/src
ln -s ~/git/pfms-2026a-buzz/quizzes/quiz5/
```
Then check the link is correct `ln -la` should show `quiz5` in teal. If it is in red your symbolic links is not correct. The path you used is incorrect.

### Building and Testing

The easiest way to compile the code is via the terminal.

1. Navigate to the workspace root:

   `cd ~/ros2_ws`

2. Build the package:

   `colcon build --symlink-install --packages-select quiz5`

3. Run unit tests for Tasks 1–3:

   `colcon test --event-handlers console_cohesion+ --packages-select quiz5`

Task 5 uses a separate testing procedure, described later.

For convenience, the full build-and-test sequence is:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select quiz5
colcon test --event-handlers console_cohesion+ --packages-select quiz5
```

## Unit Testing in ROS

In ROS, we typically unit test libraries independently from publishers and subscribers. This is known as **Level 1 testing**.

To achieve this, we provide a ROS bag (a recorded dataset) for which we know the *ground truth*. Unit tests load this bag and verify the correctness of your functions. Refer to `test/utest.cpp` to see how this is implemented.

### Visualising the ROS2 Bag

To inspect/visualize the ROS bag supplied for quiz5  

1. open a terminal and change the directory to the package folder 

   `cd ~/ros2_ws/src/quiz5/a1` 

2. Play the bag from this folder 

    `ros2 bag play -r 0.1 -l data/ros2` 

    `-r 0.1` replays the data at `0.1` of the full speed, `-l` loops indefinitely, until you `CTRL+C` to terminate this. 

3. In another terminal (same directory)

    `rviz2 -d rviz/quiz5.rviz` 

The vehicle will not appear, but you will have the coordinate system of the vehicle shown as an axis. The laser readings are in red. You can see these clumped dots, which are actually cones. In below figure we have 7 cones (6 are directly in front of the vehicle).

<img src="./pic/quiz5.png" style="zoom: 50%;" />

If you are looking at the data in rviz, in a terminal type `ros2 topic echo /clicked_point `. Then in rviz select the **Publish Point** tool and click on any of the laser points. You will see the output in terminal similar to below. which is the value of the point clicked on, in the reference frame of the laser.

```
header:
  stamp:
    sec: 1714654182
    nanosec: 641331799
  frame_id: orange/front_laser_link
point:
  x: 13.2736234664917
  y: -4.433747291564941
  z: -0.563690185546875
```

TASKS 1-4
------

For Tasks 1–4, you are required to process laser data to extract useful information about the surrounding environment (laser data is shown in red in RViz). All programming for Tasks 1–4 is done in the [LaserProcessing Class](./a1/src/laserprocessing.h). 

An object of this class is called from `Detection` class, but we can also use unit tests to achieve independent testing, by providing data for which we know the "ground truth" - the exact output we are expecting.  

Important: Always compile your code from the ~/ros2_ws directory. 
`colcon build --symlink-install --packages-select quiz5`  (this command ONLY works from within `ros2_ws` folder). 

You can compile via vscode if you have enabled the 'Termianl/Run Task' as per week 10 material. 

To execute the unit tests `colcon test --event-handlers console_cohesion+ --packages-select quiz5`

**TASK 1 - Count Returns**

Count number of laser readings that belong to obstacles. You can conisider readings belonging to objects those in a laser scan that are not infinity, nan or at max range.

As a reminder `ros2 interface show sensor_msgs/msg/LaserScan` will show you the `LaserScan` message, you will note that ranges and intensities are vectors and for each range you have the corresponding intensity at same location in the corresponding vector. Refer `week 10` for examples of working with LaserScan data.

**TASK 2 - Count Segments**

Segments are formed by consecutive laser readings that are close. To simplify things we can use a rule that successive points are less than 0.3m apart from each other (the Euclidian distance between successive points <0.3 ). The example image above has 7 segments, which belong to 7 cones.

**TASK 3 - Detect Closest Cone**

The position should be the location of the closest cone, we can leverage the segments that we computed in TASK 3. Each of those segments is a number of readings that belong to one cone. Use the mean of the points to compute the location of the cone.

**TASK 4 - Detect Road Centre**

Detect two cones, that are closest together, and on either side of the road, the road is ~8m wide. The road centre is the in the middle of the two cones, as the cones are either side of road.  


## TASK 5

Before starting Task 5, strongly consider pushing your current code to Git. This ensures you retain a working version in case changes made for Task 5 cause build issues.

Task 5 requires modifying the code to add a service node. Services are two way communication mechanisms. When [nodes](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) communicate using [services](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html), the node that sends a request for data is called the client node, and  the one that responds to the request is the service node. [Refer to ROS guide for writing services in c++](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html).

We have selected the `std_srvs` package and the `Trigger` service name.

Use `ros2 interface show /std_srvs/srv/Trigger`  to examine this service type. 

This means we now need to let our package `quiz5` know we need the `std_srvs` package as a dependency. Then we need to create a service object, tie it to a service name and have a callback function. 

For this exercise, we will not complete anything sophisticated in callback function. You need to augment the field so the Response to the service call changes values - as described below in full steps. 

**Steps**

[package.xml](./a1/package.xml) 

- [ ] Add the `std_srvs` package as both `build_depend` and `exec_depend`

[CMakeLists.txt](./a1/CMakeLists.txt) 

- [ ] Add a  `find_package` for `std_srvs`. 
- [ ] As our executable `sample` needs to use the `std_srvs` package/library we need to add to the existing `ament_target_dependencies` of `sample` executable, we need to add the `std_srvs` library.

[detection_node.h](./a1/src/detection_node.h) 

- [ ] Include the Trigger message

  `Syntax: #include "package_name/srv/service_name.hpp"`

- [ ] Create a service object

  `Syntax: rclcpp::Service<service_type>::SharedPtr variable_name_;`

- [ ] Change the `detect` function so it can be a callback for the service

​	` Syntax: void function_name(const std::shared_ptr<service_type::Request>  req,std::shared_ptr<service_type::Response> res);`

[detection_node.cpp](./a1/src/detection_node.cpp) 

- [ ] Create a service object of service type `std_srvs/srv/Trigger` on service name `detect_road_centre` with function name `detect` as callback.
- [ ] Change the `detect` function so it matches the header
- [ ] Change the Response fields `success` which is a boolean to be `true` and make `message` (which is a string) be exactly the following `Road centre detected`

**Testing**

In a terminal run your node `ros2 run quiz5 sample`  and in another terminal `ros2 service call /detect_road_centre std_srvs/srv/Trigger {}`

If you have completed all steps you will see  response in the terminal that has made the service call which indicated you have passed this test.

```bash
waiting for service to become available...
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='Road centre detected')
```

