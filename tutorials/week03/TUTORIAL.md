Week 3 Tutorial Questions
=========================
Work through these questions to understand what is going on in each example. If you have any questions about the material please raise them in the next tutorial session. 

The code will not compile without making the required changes in Ex01.

Inheritance - Ex01
--------------------

Refer to base class `Shape` and the existing `Circle` class as well as the declared but not implemented `Rectangle` class. 

Implement:

* `Circle` and `Rectangle` as derived classes of `Shape`
* Implement the constructor method of `Rectangle`
* Implement the `setHeightWidth` method of `Rectangle`
* Implement  `getArea` method in `Circle` and `Rectangle`

Consider: Which members (data/methods) should belong to the base class and which to the derived class.

* Implement `setCentre` and `offsetCentre` methods.

Consider: Where should this function be placed (both the declaration and implementation)?

In your `main.cpp`  

* create a rectangle object of `Rectangle` class with width and height (5.0 and 3.5 respectively). 
* create a circle object from `Circle` class with a radius of 3.0. 
* Show the area as well as the description of both objects
* Create a pointer `*rect2`to a `Rectangle` object with width and height (0.2 and 0.3 respectively). HINT: What can we use as the pointer type?
* Show the area as well as the description of the object the pointer is pointing to?

Finally, consider if the constructor for shape should take in a description. And if so, how would we implement this given Shape is the base class and takes arguments? What is the advantage of having a default constructor that takes no parameters? 

Inheritance and Pure Virtual - Ex02
------------------

We now want to extend our previous code with a `bool checkIntercept(double x, double y)` method that determines if the shape intersects a (x,y) point provided.  Determine where to implement the method (base or derived classes).  Consider if it needs information only from the shape itself or if it needs more information about the specific shape. Implement the method.

Then in Main

* Create a Rectangle and Circle. How can we store both of them in one vector?
* After solving the previous point, write a program that accepts a point *(x,y location)* and displays all shapes that intersect that point (the point is within the area of the shape)?

Consider: What will be the type for this vector? Are there any issues with completing item 2 here?  
