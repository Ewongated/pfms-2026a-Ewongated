#include "circle.h"
#include <cmath>

Circle::Circle(double radius):
    radius_(radius)/// This is the best form of initialisation for a constructor, via an initialiser list
{
    recalculateArea();
    recalculatePerimeter();
}

void Circle::setRadius(double radius)
{
    radius_ = radius;
    recalculateArea();
    recalculatePerimeter();
}

double Circle::getArea()
{
    return area_;
}

double Circle::getPerimeter()
{
    return perimeter_;
}

void Circle::setArea(double area){
    area_=area;
    radius_= std::pow(area_/M_PI,0.5);
    recalculatePerimeter();
}
void Circle::setPerimeter(double perimeter){
    perimeter_ = perimeter;
    radius_= perimeter_/(2*M_PI);
    recalculateArea();
}

void Circle::recalculatePerimeter(){
    perimeter_ = M_PI*2*radius_;
    double x;
}


void Circle::recalculateArea(){
    area_ = radius_*radius_*M_PI;
}

