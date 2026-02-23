// Includes std::cout and friends so we can output to console
#include <iostream>

// You can use a macro (#define) to represent the array size
//# define ARRAY_SIZE 10
static const int ARRAY_SIZE=10; // This is much better, as it is a constant and can't be changed, and is of known type

// Every executable needs a main function which returns an int
int main () {
    // Create an array x of doubles with 10 elements
    // Populate the elements of array on creating of array, each element [i] has value i (INITIALISER LIST)
    double x[ARRAY_SIZE] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

    // Can you create a loop to populate elements of x (each x[i] =i), how to code end of loop?) – (USE MACRO)
    for (int i = 0; i<2000; i++) {
        x[i] = i;
        std::cout << "x[" << i << "] = " << x[i] << std::endl;
    }

    // Can you use a pointer and loop to initialise elements of array (*ip++)
    //
    // Two concepts here
    // 1) loop termination
    // ip is a double pointer, so the loop termination is when
    // ip points to a memory address which is x+ARRAY_SIZE, ip can't be comapred to ARRAY_SIZE
    // 2) the increment step
    // As we have to do assignment as well as incrementing pointer we use a trick
    // we assign to *ip ip-x which increases as we go through the array (x is always the zeroth element)
    // And we also incraese the ip pointer, the location it points to by one

    // We can also use a counter variable to assign the value
    // and increment it as well as the pointer
    for (double *ip = x, ct = 0.0; ip<(x+ARRAY_SIZE); ip++, ct++) {
        *ip = ct;
        std::cout << "*ip = " << *ip << std::endl;
    }

    // Buit even better we use the pointer arithmetic to assign the value
    for (double *ip = x; ip<(x+ARRAY_SIZE); ip++) {
        *ip = ip-x; // ip-x is the index of the element, as x is the base address
        std::cout << "*ip = " << *ip << std::endl;
    }

    // Main function should return an integer
    return 0;
}
