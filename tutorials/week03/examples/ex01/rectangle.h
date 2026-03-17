#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"

class Rectangle : public Shape
{
public:
    //* @brief Constructor for the rectangle
    //* @param width the width of the rectangle
    //* @param height the height of the rectangle
    Rectangle(double width, double height);

    //* @brief Set the width of the rectangle
    //* @param width the width of the rectangle
    //* @param height the height of the rectangle
    void setHeightWidth(double width, double height);
    
    //* @brief Get the area of the rectangle
    //* @return the area of the rectangle 
    double getArea (void);
private:

    //* @brief Update the description of the rectangle, called internally when the width or height is set (including in the constructor)
    void updateDescription(void);

    double width_; //!< width of rectangle
    double height_;//!< height of rectangle
};

#endif // RECTANGLE_H
