#ifndef CIRCLE_H
#define CIRCLE_H

#include "shape.h"

class Circle : public Shape
{
public:
    //* @brief Constructor for the circle
    //* @param radius the radius of the circle
    Circle(double radius);

    //* @brief Set the radius of the circle
    //* @param radius the radius of the circle  
    void setRadius(double radius);

    //* @brief Get the area of the circle
    //* @return the area of the circle
    double getArea ();

    bool checkIntercept(double x, double y);//! return true if the point is within the circle

private:
    double radius_;//<! radius of the circle
};

#endif // CIRCLE_H
