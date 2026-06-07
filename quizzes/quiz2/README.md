Quiz 2
======

In both quizzes DO NOT CHANGE ANY OF THE EXISTING function declarations. You can add other private functions, but your code will only be tested against the currently defined public functions.

Part A
------

**Preamble**

We are in charge of a drag race and will design a class `Analysis` to handle this. The class has two constructors (one of which takes a visualiser). 

You will need to utilise the car library which is provided for the Task (you don't need to write it), refer to classes [CarInterface](./a/dep/include/carinterface.h) which is the interface class and examine [Car](./a/dep/include/car.h). You will need to pass the cars via `CarInterface*` to the [Analysis](./a/analysis.h) class for further processing, look at the constructor of `Analysis`. 

We have a `demoRace` that is just a small demo to show how to accelerate and decelerate the cars.  This allows to have a feel for how to control the cars.  You have to keep accelerating to go faster, and decelerate to slow down (functions available), refer to class [Car](./a/dep/include/car.h) . We create some cars to run the demo race (We show how to create an object of `Car`), the demo shows the car on a circular track as they race.  You have to keep accelerating to go faster, and decelerate to slow down (functions available), refer to class [Car](./a/dep/include/car.h) . The cars only accelerate to a top speed that depends on the car parameters, and decelerate to zero. 

**TASK 1 - Standard drag race**

Create a drag racing algorithm, where each vehicle will race a distance of 100m and then stop. So the logic for `dragRace` is:

  1. Accelerating each car 
  2. When all cars cover 100m the race is finished, we declare the winner as the first car to cover (travel) 100m.  An example is below, the vector returned has value {4,42,21} in order of crossing the 100m line

| Car ID | Place                   |
| ------ | ----------------------- |
| 4      | 1st to cover 100m        |
| 42     | 2nd to cover 100m        |
| 21     | 3rd (last) to cover 100m |

HINTS: (1) use a loop to do this by going over a container of vehicles (2) what do we need to query at beginning of race (3) what function needs to be called during race 

**TASK 2 - Stop all cars**

Though we did a drag race, we didn't stop the vehicles (decelerate them to zero speed). Create a function `stopAllCars` in the `Analysis` class to achieve this. This function needs to stop (bring speed to zero) for all cars.

HINTS: (1) use a loop to do this by going over a container of vehicles, (2) what other information needs to be stored to know if to further decelerate each vehicle, (3) remember dangers of comparing float numbers to zero?

**TASK 3 -  Zero - Top Speed - Zero race**

Initial drag race (in TASK 2) favoured high horse power, we have designed another race that is more about power to weight ratios and breaking force. All cars have to reach their individual top speed and then go back to zero speed.  So the logic for `zeroTopZeroRace` is

 1. Accelerate all cars
 2. As each individual vehicle reaches top speed decelerate that car, keep other cars that have not reached their top speed accelerating
 3. When all vehicles have stopped, terminate the race

 Return the order of vehicles that reached zero speed (the car MUST have reached top speed before coming back to zero speed)

Example below, , the vector returned has value {1,43,16} 

| Car ID |            |
| ------ | ---------- |
| 16     | 3rd (last) |
| 1      | 1st        |
| 43     | 2nd        |


HINTS: (1) use a loop to do this by going over a container of vehicles (2) what function(s) needs to be called during race (3) what other information needs to be stored to know when to decelerate each vehicle and where would you store this

**TASK 4- Sorting cars by odometry**

Implement the `sortByOdometry` sorting algorithm, that will sort the vehicles per kilometres already travelled in their odometry, sorted in ascending order. Think of the odometry as hoping into your car and turning the key, the odo will show how many km's your car has travelled thus far.

Example below, the vector returned has value {10,22,7} as this orders the cars per odometry in ascending order.

| Car ID | Odometry | 
| ------ | -------- | 
| 10     | 12788.9  | 
| 7      | 68833.1  | 
| 22     | 56686    | 


Part B
------

1. TASK: Implement function `void populateContainer(std::deque<double>& container, unsigned int num_values, double element)` that accepts a container and modifies it by adding user specified number of `elements` to the front of container. The actual `element` to be added is also supplied by user (for instance num_values =4 element =-1.5) ; this would result in four elements of -1.5 are added to beginning of deque)  [container_ops.cpp](./b/container_ops.cpp)
