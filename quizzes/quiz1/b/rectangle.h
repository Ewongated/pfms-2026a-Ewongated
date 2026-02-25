#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"

// TASK:
//Modify the file rectangle [Rectangle] so it inherits from the base class of Shape and is a derived class of Shape. 
class Rectangle
{


public:
//    TASK 1
//    The Rectangle class needs to have a special member function that enables the Rectangle to on creation have width_ , height_ initialised 
//    with values supplied when creating object of class. This function needs to set the description_ on creation of the class to be either square or 
//    rectangle depending on the width and heigth supplied by the user.

    /**
     * @brief Function that sets width and height
     * @param width in [m]
     * @param height in [m]
     */
    void setHeightWidth(double width, double height);

    /**
     * @brief Function that returns area
     * @return area in [m2]
     */
    double getArea (void);
private:

    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Why are these member varaibles in Rectangle, and not in shape?
    double width_; //!< width of rectangle
    double height_;//!< height of rectangle
};

#endif // RECTANGLE_H
