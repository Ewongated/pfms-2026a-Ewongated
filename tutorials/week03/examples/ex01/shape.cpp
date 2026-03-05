#include "shape.h"

Shape::Shape(std::string description) :
    description_(description)
{
    centreX_ = 0;
    centreY_ = 0;
}

void Shape::setCentre(double x, double y)
{
    centreX_=x;
    centreY_=y;
}


std::string Shape::getDescription()
{
    return description_;
}

void Shape::offset(double x, double y)
{
    centreX_ += x;
    centreY_ += y;
}

double Shape::getArea()
{
    return 0;
}