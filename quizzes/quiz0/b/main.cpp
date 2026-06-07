#include "sample.h"
#include <iostream>

int main () {


//  Create an object `sample` of `Sample` class

//  Display to standard output the value of parameter `value_` from the `sample` object.

    Sample s(3);
    std::cout << "Value: " << s.readValue() << std::endl;

    s.setValue(12);
    std::cout << "Value: " << s.readValue() << std::endl;
    return 0;
}
