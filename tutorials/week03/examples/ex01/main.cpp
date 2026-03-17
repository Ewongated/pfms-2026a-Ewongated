#include <iostream>
#include <vector>

#include "rectangle.h"
#include "circle.h"

int main () {

    Rectangle rectangle(5.0, 3.5);
    std::cout << "The area of rectangle is " << rectangle.getArea() << std::endl;
    std::cout << "It is a " << rectangle.getDescription() << std::endl;

    Circle circle(3.0);
    std::cout << "The area of circle is " << circle.getArea() << std::endl;
    std::cout << "It is a " << circle.getDescription() << std::endl;

    circle.setCentre(3.0,3.0);
    circle.offset(1.0,1.0);

    // We can create a pointer to the base class, the new creates a pointer to the derived class (Rectangle)
    Shape* rectPtr = new Rectangle(0.2,0.3);
    std::cout << "The area of " << rectPtr->getDescription() << " is " << rectPtr->getArea() << std::endl;
    //Or we can dereference the pointer and call the function
    std::cout << "The area of " << (*rectPtr).getDescription() << " is " << (*rectPtr).getArea() << std::endl;

}
