Week 8 Tutorial 
=========================
Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material, please raise them in the next tutorial session.

In order to run the unit tests, they have been developed against **pfms2ros** library (version 3.1.1 or higher).

If you execute `dpkg -l pfms-ros-integration` it should show below:

```bash
=================================================================================
ii  pfms-ros-integration 3.1.1        amd64        Implementation of library for PFMS to abstract communication with ROS and using Gazebo simulator.
```

If your not on 3.1.1 or above

```bash
sudo apt update
sudo apt install --reinstall pfms-ros-integration ros-humble-pfms
```

While developing the unit tests you do not need to run the simulator. However prior to running the unit test run the simulator `ros2 launch pfms a1.launch.py` in a terminal window. Then run the unit test from the `build` directory.


-------

Unit Testing 
===============================

We will use Google's C++ unit testing framework. A good starting point for using googletest is to read the [primer] document. 

Our examples today revolve around making Unit Tests for an example of Assignment 1, we are testing the functionality of code according to specifications. We will be designing tests that test the code as per expected output. 

We will be using a library built by one of your colleagues for Assignment 1 and testing it with the specifications at hand. You have been provided the compiled libraries which will be installed in your system the first time you execute the cmake command for ex01.  All the tests are in the `marking` folder.

To compile the unit tests from within folder `starter/ex01`

```bash
mkdir build
cd build
cmake ..
make
```

Task 1
-------

The [test_constructors.cpp](./starter/ex01/marking/test_constructors.cpp) needs to  check variables have been initialised correctly on creation of `Sonar` and `Laser` class. 

What functions should we check? Can we guarantee the use of the class will call any function of the class in a particular order? Use the getters to check the values are set.

**TASK**: 

Implement the constructor tests  for  `Sonar` in the `TEST(ConstructorTest, SonarSkidSteer)` and `TEST(ConstructorTest, SonarAckerman)` and for `Laser` in the `TEST(ConstructorTest, LaserSkidSteer)` and `TEST(ConstructorTest, LaserAckerman)`.  Run tests via via `./marking/testConstructors`

**NOTE:**

You are required to implement these tests for A2 of your derived classes from `Controller`


Task 2
-------

The [test_ranger_interface.cpp](./starter/ex01/test/test_ranger_interface.cpp) needs to the functions of `RangerInterface` class. 

What functions should we check? Can we guarantee the use of the class will call any function of the class in a particular order? Use the getters to check the values are set.

**TASK**: 

Implement the  `TEST(RangerInterface, Simple)`  to check functions of `RangerInterface`.  Run tests via via `./marking/testRangerInterface`

Task 3
-------

The [test_fusion_interface.cpp](./starter/ex01/test/test_fusion_interface.cpp) needs to the functions of `FusionInterface` class. 

What functions should we check? 

**TASK**: 

Implement the  `TEST(FusionInterface, Simple)`  to check functions of `FusionInterface`.  Run tests via via `./marking/testFusionInterface`

## Task 1-3 Summary

You can copy the marking folder into your own a1 submission. To enable tests you just need to add a few lines to the bottom of the CMakeLists.txt in your a1 submission.

```cmake
# To enable testsing we instaed set BUILD_TEST to ON by default
set(MARKING_TESTS ON)
if(MARKING_TESTS)
    add_subdirectory(marking)
endif()
```

Task 4
-------

Examine the [test_audi.cpp](./starter/ex01/test/test_audi.cpp). This unit test puts under scrutiny the `Ackerman` class calculation of `distance` and `time` to target. Here the unit tests are part of `TEST_F` `AckermanTest` group.  The difference with `TEST_F` is that they allow to setup variables that are persistent between all tests via `SetUpTestCase` and `TearDownTestCase`. This is slightly beyond scope of the subject, though if interested examine the [ackerman_test.h](./starter/ex01/test/ackerman_test.h).

The unit tests examine the public interfaces of the `Ackerman` class and we have 2 unit tests that we have developed for two goals. 

**TASKS** to complete are:

- [ ] Test `Ackerman` going in a straight line, often division by zero is not handled well so it's worth testing in `TEST_F(AckermanTest, StraightLine)`.

- [ ] Create a unit tests for another location `TEST_F(AckermanTest, YourTest)` for which you have computed the distance and time [from a separate source](https://www.omnicalculator.com/math/arc-length).


This test example needs to be used with your a2 code. As your being examined on how you develop a base and derived class, we can not provide examples for you to test against.

**Outside of class, contemplate how you could test your Assignment 2**    


[primer]: https://github.com/google/googletest/blob/master/docs/primer.md
