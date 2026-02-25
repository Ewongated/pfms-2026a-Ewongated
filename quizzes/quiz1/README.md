Quiz 1
======

Part A
------

The code supplied will not build until you complete TASK 1.

1) TASK
The `Charger` class is missing a special member function. This function needs to enable creating an object of `Charger` class with the `model`, `batteryLevel`, and `charging` initialized with values supplied when creating an object of the class. You will need to add the declaration of this member function in the [header of Charger class](./a/charger.h) as well as implement this function in [implementation file of Charger class](./a/charger.cpp).

2) TASK
Implement the method `recharge` in `Charger` class. This function increments `batteryLevel` by 25 (the `batteryLevel` can not exceed 100). The function returns a `bool` indicating if a charger is being charged. When a charger is completely recharged, its `batteryLevel` should at 100 and `charging` should by false.

3) TASK
Implement [lowestBatteryLevel](./a/processing.h) function that returns the `Charger` who has the lowest battery level in the `fleet`. There could be multiple chargers with the same battery level, therefore the return type is a vector.

4) TASK
Implement [eligibleForRecharge](./a/processing.h) function that returns the `chargers` from the `fleet` that need to be recharged. The return type is a vector. The criteria is that they have a battery level lower than the specified `batteryCutoff` and are not currently charging. The return vector needs to be organized in the order of lowest to highest battery level (the lowest is at the beginning and highest at the end of the vector).

Part B
------

The code supplied will not build until you complete TASK 1.

1. TASK
The following activities all fall within this single TASK
* Modify the `Rectangle` class so it inherits from the base class of shape [shape](./b/shape.h). 
* Correct the access specifiers of base class [shape](./b/shape.h) so that the implementation of the [`Rectangle` constructor](./b/rectangle.cpp) can access the `description_` member variable of `Shape`. This access should be restricted to within the `Rectangle` object. View access specifiers to understand how to acheive this`
* Enable the  `Rectangle` class to have a special member function that enables the `Rectangle` to on creation have `width_` , `height_`  initialised with values supplied by user of the class. This function needs to set the `description_` on creation of the class to be either `square` or `rectangle` depending on the `width` and  `heigth` supplied by the user. Look at the existing `setHeightWidth` function for ideas.
