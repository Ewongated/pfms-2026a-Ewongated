TASK 2: Using ROS Service
----------------------------

We will use the package in the `services_masterclass`, the package is called `week11_quad`. 

The Quadcopter receives a goal via topic name `/sjtu_drone/goal` of the type `geometry_msgs/msg/Point`.  We need to accommodate a incoming service call (we are a service node) that can be made on `/reach_goal`  of type `std_srvs/srv/SetBool` which runs function `Quadcopter::request`. To view the service message type `ros2 interface info std_srvs/srv/SetBool`. We aim to track the status of the quadcopter via `quadcopter::PlatformStatus status_` , status is defined in `quadcopter.h` and return information via return fields.

**Modify the code to incorporate the service call**

We have selected the `std_srvs` package and the `SetBool` service name.

Use `ros2 interface show /std_srvs/srv/SetBool`  to examine this service type. 

This means we now need to let our package `week11_quad` know we need the `std_srvs` package as a dependency. Then we need to create a service object, tie it to a service name and have a callback function. 

For this exercise, we will not complete anything sophisticated. In callback function you need to augment the field so the Response to the service call changes values - as described below in full steps. 

**Steps**

[package.xml](./wk11_starter/services_masterclass/package.xml) 

- [ ] Add the `std_srvs` package as both `depend` 

[CMakeLists.txt](./wk11_starter/services_masterclass/CMakeLists.txt) 

- [ ] Add a  `find_package` for `std_srvs`. 
- [ ] As our library `controller` needs to use the `std_srvs` package/library we need to add to the existing `ament_target_dependencies` of `controller` library, we need to add the `std_srvs` library.

[controller.h](./wk11_starter/services_masterclass/src/cotroller.h) 

- [ ] Include the SetBool message

  `#include "package_name/srv/service_name.hpp"`

- [ ] Create a service object

  `rclcpp::Service<service_type>::SharedPtr variable_name_;`

- [ ] Create a `control` function so it can be a callback for the service, and make it a pure virtual so it's implemented in  Quadcopter,

​	` void function_name(const std::shared_ptr<service_type::Request>  req,std::shared_ptr<service_type::Response> res);`

[controller.cpp](./wk11_starter/services_masterclass/src/controller.cpp) 

- [ ] Create a service object of service type `std_srvs/srv/SetBool` on service name `reach_goal` with function name `control` as callback.

[quadcopter.cpp](./wk11_starter/services_masterclass/src/quadcopter.cpp) 

- [ ] Examine the state machine in `reachGoal` draw it out, what are the states and transitions.
- [ ] Implement the `control` function as per below which adds to the state machine.
- [ ] Change the Request and Response fields as per below

#### **Modify the code so that**

1. We need to track the status of Quadcopter via `pfms::PlatformStatus` that tracks current status of Quadcopter.

2. Enable to query and change the status via the service call  (which activates `control` function) and the data field in Request portion 
   *  if data is true
      * will `TAKEOFF` if the platform is `IDLE` and there are no goals
      * will `TAKEOFF` and then perform execution of the goal (go into `RUNNING`) if there is a goal set that is not yet reached
      * will keep in `RUNNING` is still going to a goal
   *  if data is false
      * will perform `LANDING` and then go to `IDLE` irrespective if there is a goal

3. In Responce we can obtain the current status of the Quadcopter via the service and the success and message part of the response portion

* The `success` field
  * returns true if there is a goal set that is not yet reached
* The `message` field 
  * shall have a string corresponding to current status (for instance `IDLE` status responds back with `IDLE` string value)

**HINTS**

* There is a thread of execution `reachGoal` linked to a timer. Keep this the loop that runs through at 10Hz  (play around with the rate needed)
* Check your select/case or if/then statements to handle status 
* How if the odometry accesses and why is it private in controller.h? 
* We have had to place the PlatformStatus value under namespace quadcopter
* Do we have to assume quadcopter is `IDLE` at startup ... or can we check the odometry to put it in `RUNNING`?

#### Execution

We executing the code we need to have a few terminals.

We can start the simulator

```bash
ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py
```

Then we can run the node itself

```
ros2 run week11_quad sample
```

To send a goal via the ROS2 CLI.

```bash
 ros2 topic pub -1 /drone/goal geometry_msgs/msg/Point "{x: 0, y: 5, z: 2}"
```

And finally, an additional terminal to invoke the service call using the CLI

```bash
ros2 service call /reach_goal std_srvs/srv/SetBool "{data: true}"
```