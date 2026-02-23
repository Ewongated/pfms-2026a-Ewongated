Week 1 : Pre-work
=========================

Please complete refresher of pointers, arrays and enums.

Ex01 - Pointers & Reference
--------------------

Pointers are essential to C++ and are used to store the memory address of another variable. They are declared by using the asterisk (*) before the variable name.  The data type of the pointer must be the same as the data type of the variable you're storing the address of.  When passed as a parameter to a function, the function can modify the value of the variable by using the pointer.

Work with [pointers.cpp](./starter/pointers.cpp), follow the in-code prompts on what to complete, examine changes between variables. 

The goal is here to understand how a pointer `*` and reference`&` enable us to change values of variables they are pointing/referring to. 

Ex02 - C Arrays
--------

C Arrays are a collection of elements of the same data type. They are declared with the following syntax: ` <data type> <array name> [<array size>]`

 The array size must be a constant value.   The elements of the array are accessed with the following syntax: `<array name>[<index>]`.  The index is a unsigned integer value that represents the position of the element in the array. The first element is at index 0, last element is at index `<array size> - 1`.  The elements of the array are contiguous in memory.

Work with [arrays.cpp](./starter/arrays.cpp),  follow the in-code prompts on what to complete. 

There are several limitations imposed with arrays, think about what they all are? We will discuss in class. 

Ex03 - Enums + switch/case statements
--------

Enums ([enumeration](https://en.cppreference.com/w/cpp/language/enum)) allows us to set a distinct type whose value is restricted to a range of values. This is useful when we know the permissible set of values, allows us to define them by name and refer to them for readability of our code. 

Work with [enum_example.cpp](./starter/enum_exammple.cpp)

In the example you will find two enums, one for `color` that has a range of values `red,green` and another for `altitude`, which we specify to be a char of either `high, low`   

TASKS

* Add two more values to color `yellow (value 1) and blue (value 21)`. 
* Add the values to the switch statement in function `std::ostream& operator<<(std::ostream& os, color c)`
* Add a default case in switch statement `std::ostream& operator<<(std::ostream& os, color c)`

We here show the power of C++ and the `operator<<` . If your coming from C and have use printf, shelve that. C++ allows us to use the insertion (`<<`) operator to write [standard data types](https://en.cppreference.com/w/cpp/language/types) to terminal or to a file using `<<` (here we write to std::cout  - standard out to terminal). If we have custom types (such as enums or structs) then we need to create our own `operator<<` for them, but can then in the future simply print a variable using `<<`.

