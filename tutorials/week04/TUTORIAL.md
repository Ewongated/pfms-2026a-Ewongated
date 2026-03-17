Week 4 Tutorial Questions
=========================
Work through these questions and make sure you understand what is going on in each example.  If you have any questions about the material, please raise them in the tutorial session.

Today's work builds upon solutions in week 03 which are in starter code.

The material is to assist Assignment 1 and understanding how to layer [a1_skeleton](../../skeleton/a1_skeleton).

Distributing your own library is beyond the scope of this subject, though some material to achieve this is part of  [ADVANCED](./ADVANCED.md). 

Inheritance  
------------------

We use [shapes folder in starter](./starter/shapes) , look at `Shapes` , `Rectangle`  , `Triangle`, `Circle`, what is there relationship. Which functions should be defined and implemented in `Shape`, which should be left to be implemented in `Rectangle`  , `Triangle`, `Circle`,. How do we achieve this? Examine `ShapeProcessing` class.

#### TASK: 

Implement the constructor and `checkIntersections` function. Details of expected functionality are in the header file. The main  emulates a process similar to Battleship, have a look at it's inner working (there is no fancy gui, just the logic).

<img src="https://articulate-heroes.s3.amazonaws.com/uploads/rte/udxpzlbr_Storyline%20E-Learning%20Battleship%20Game%20Template.GIF" style="zoom: 50%;" />



Library Creation and Unit Testing 
-----------------

We use [shapes folder in starter](./starter/shapes) , look at the  [CMakeLists.txt](./starter/shapes/CMakeLists.txt) is generating two **shared** libraries, one called `shapes` and the other `shapeprocessing`

This enables us to unit test of the libraries, which is essential to ensure it performs correctly. We therefore introduce Unit Tests at this point, so you can start to design your own tests for your code.

In [utest.cpp](./starter/shapes/test/utest.cpp) we are checking our implementation of intercept for the Circle. The syntax of the test is `TEST (IntercectTest, Circle)` the `TEST` indicates it is a unit test `IntercectTest` is the suite name and the individual test is `Circle`. So this is a suite of intercept tests and we plan to do this on all shapes. Can you add `TEST (IntercectTest, Rectangle)` and `TEST (IntercectTest, Triangle)` using the supplied example?

The tests will be compiled at the same time as your code is compiled. You can run all tests with minimal reporting using `make test` or all tests individually from the build directory using `./test/utest`.

#### TASK

Add more tests for the area using a different suite `AreaTest`, you will now need to compare values, and as values are floats the unit test needs to call `ASSERT_NEAR` and example is ` ASSERT_NEAR(a,b,1e-5)` where `a` and `b` are compared to a precision of `1e-5`. Finally, can you create a test for shape processing? Which method do we need to test and how? 

There is a very comprehensive guide called [Googletest Primer](https://github.com/google/googletest/blob/master/docs/primer.md) .

## Using PfmsConnector, PfmsHog and Cell Library 

This code uses libraries supplied for this subject which are part of your system, like `vector` or `iostream` are. At the beginning of semester when you customised Ubuntu for PFMS you have enabled packages for the subject. You can execute below to install those packages needed for Assignment 1.

```bash
sudo apt-get update
sudo apt-get install pfms-ros-integration ros-humble-pfms
```

We will use [a1_snippets in starter](./starter/a1_snippets) , look at the  [CMakeLists.txt](./starter/a1_snippets/CMakeLists.txt)  it is using `find_package(pfms2ros)` to find the package that contains libraries needed.

- **pfmsconnector** - to interact with the simulator and get data or drive the platforms
- **pfmshog** - to move platforms or objects
- **cell** - to create and draw cells in the visualiser

The code we have supplied shows how to connect to a platform (`ACKERMAN`  or ``SKIDSTEER`)  via a `shared_pointer` to `PfmsConnector`object. We can then use the `send` `read` methods to send the correct command to the platform. These would be `pfms::commands::Ackerman` or `pfms::commands::SkidSteer`. We can  obtain data from the laser and sonar sensor attached to the platform.

When running (executing) the code, you will need to have the gazebo simulator running. For this task we can use the assignment 1 `launch` file

```bash
ros2 launch pfms a1.launch.py
```

#### Example Files

We have two platform types available with examples on how to command them:
- **command_ackerman.cpp** - demonstrates how to send commands to the ACKERMAN platform
- **command_skidsteer.cpp** - demonstrates how to send commands to the SKIDSTEER platform

There are also examples on how to receive sensor data from a platform (shown for ACKERMAN):
- **receive_laser.cpp** - shows how to receive and process laser scanner data
- **receive_sonar.cpp** - shows how to receive and process sonar sensor data

Additionally:
- **move_items.cpp** - demonstrates how to move items using **pfmshog**
- **visualise_cells.cpp** - shows how to create and visualise cells in the visualiser, where cell occupancy is colour coded in the visualiser.

#### Task

Create a new cpp file that combines moving the `ACKERMAN` very slowly forward, reading the `Laser` and detecting the closest object in the readings. Attempt to convert the position reported into `global world coordinates` using the position of the platform. Refer to Assignment 1 specification to acertian where the laser is on the platform and what other ifnromation you might need for the task. 
