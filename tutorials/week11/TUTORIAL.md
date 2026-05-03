Week 11 Tutorial Questions
=========================

To use the packages this week you will need to install a few dependencies (these are specific to TASK 3 and Assessment 3)

```bash
sudo apt update
sudo apt install ros-humble-grid-map-core ros-humble-grid-map-msgs ros-humble-grid-map-ros ros-humble-grid-map-rviz-plugin 
sudo apt install ros-humble-sjtu-drone-bringup ros-humble-sjtu-drone-description
```

Before you get started, make sure you link the `wk11_starter` folder to your `ros2_ws` workspace folder, (ie if your path is <YOURGIT>/tutorial/week11/wk11_starter then execute:

```bash
cd ~/ros2_ws/src
ln -s <YOURGIT>/tutorial/week11/wk11_starter
```

Check that the link is in teal colour, rather than red (red means it is incorrect). On WSL there is an issue with the `ln -s` where the `-` appears an invalid symbol, just type in `ln -s`. 

Compile packages using `colcon` , the `--executor sequential` will not use multi cores, which can sometimes cause issues with your PC.

```bash
cd ~/ros2_ws
colcon build --symlink-install --executor sequential
```

The very first time your build a package it gets added to ROS and therefore you need to only once  `source ~/ros2_ws/install/setup.bash`.

**Focus of exercises is using services and topics, and is broken down in 3 tasks**

[TASK 1 - Complete solution that introduces Services on an example of two nodes communicating via a service](./TASK1.md)

[TASK 2 - We insert a service into our quad demo example of Week 10, and internally use it to drive a state machine](./TASK2.md)

[TASK 3 - We revisit topics, and introduce the Sonar and GridMap message types ](./TASK3.md)
