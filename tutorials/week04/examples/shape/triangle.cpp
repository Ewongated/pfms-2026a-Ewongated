#include "triangle.h"
#include <iostream>

Triangle::Triangle(double width, double height):
    width_(width), height_(height)
{
    description_ = "isoc triangle";
    updateVerticies();
}

void Triangle::setHeightWidth(double width, double height)
{
    width_ = width;
    height_ = height;
}

double Triangle::getArea()
{
    return width_ * height_ * 0.5;
}

void Triangle::setCentre(double x, double y)
{
    Shape::setCentre(x, y);
    updateVerticies();
}

bool Triangle::checkIntercept(double x, double y){

    //! Internally the function calculate the barycentric coordinates
    double denominator = ((y2_ - y3_)*(x1_ - x3_) + (x3_ - x2_)*(y1_ - y3_));
    double a = ((y2_ - y3_)*(x - x3_) + (x3_ - x2_)*(y - y3_)) / denominator;
    double b = ((y3_ - y1_)*(x - x3_) + (x1_ - x3_)*(y - y3_)) / denominator;
    double c = 1 - a - b;

    //! Using the barycentric coordinates we wheck if the point is inside the triangle
    return (a >= 0) && (b >= 0) && (c >= 0);
}

void Triangle::updateVerticies()
{
    //! Calculate the vertices of the triangle based on the centre and the width and height
    //! of the triangle and updated the private member variables x1_, y1_, x2_, y2_, x3_, y3_
    x1_ = centreX_;
    y1_ = centreY_ + 0.5 * height_;
    x2_ = centreX_ - 0.5 * width_;
    y2_ = centreY_ - 0.5 * height_;
    x3_ = centreX_ + 0.5 * width_;
    y3_ = centreY_ - 0.5 * height_;

}