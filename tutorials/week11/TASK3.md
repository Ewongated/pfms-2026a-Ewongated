TASK 3: Using GridMaps and Laser Data (the Sonar)
----------------------------

We will use the package in the `grid_example`, the package is called `grid_example`. This contains  an example of using a `grid_map` to store elevation data (think of it as a height map). 

The package has an example `simple_demo_node` which creates a `GridMap` containing some sample data. Examine the `simple_demo_node.cpp` file for how the grid is created, and what occurs in the loop. Find the section of the code that changes a specific cell in the `GridMap` and subsequently publishes this to ROS.  You can run the node via `ros2 run grid_example simple_demo`

You can start `rviz2` in the terminal, and then under `Displays` click `Add` and select the `GridMap`. If you under topics select `/grid_map` you will be able to visualise a nice grid with a peak, where is the peak located? How does that get created in the code. If you don't see the `GridMap` you need to install the plugin `sudo apt install ros-humble-grid-map-rviz-plugin`  and restart rviz2.

We have a node being created from `sample.cpp` source file. Check in `CMakeLists.txt`how this is occurring. In the `sample.cpp` there is a function called `process` which runs as a thread at 1Hz. 

Running this code is via `ros2 run grid_example_solution sample`

Your task: In the `process` function, use the `sjtu_drone` odometry and the sonar `range` and create a height map as we fly around. Your goal is to combine this data and deposit in the `GridMap` the current map height as seen by the `Sonar`. The Gazebo Fortress does not have a Sonar plugin so we have a Laser instead of it looking downwards.

The solution we will provide after class (examples) uses a fictions value for the sonar `2`  meters, and simply changes any existing value. By default the grid will be stored with `nan` or `inf` values for each cell. You need to consider that the grid might already have a value, and what you would consider doing if you get a new measurement, would you not simply erase the previous value (most likely you should average them or ignore outliers - big jumps). A `GridMap` can be a lot of data, so publishing it should be done with consideration, at a lower rate.

#### Execution

We executing the code we need to have a few terminals.

Start the simulator

```bash
ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py
```

We can run our week 10 solution for the quadcopter (or the one developed above)

```bash
ros2 run week10_quad_solution sample
```

We then run the grid_node

```bash
ros2 run grid_example sample
```

Finally, if your using week 10 to control the quadcopter send it a goal. 

If using week 11 you need to send a goal and call the service.

```bash
ros2 topic pub -1 /drone/goal geometry_msgs/msg/Point "{x: 1.0, y: -3.0, z: 4.0}"
```