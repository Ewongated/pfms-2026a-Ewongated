#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "shape.h"

/*!
 *  \ingroup   ac_shape Triangle
 *  \brief     Triangle class
 *  \sa        Shape
 *  \details
 *  This class is the an isoseles traingle.\n
 */
class Triangle : public Shape
{
public:
    /**
     * @brief Constructor for Triangle, the triangle is an isosceles triangle, centred at 0,0
     * @param width in [m]
     * @param height in [m]
     */
    Triangle(double width, double height);

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
    double getArea ();

    /**
     * @brief Function that checks if shape intercepts a point, uses barycentric coordinates to determine is point is within triangle
     * @param point location x in [m]
     * @param point location y in [m]
     */
    bool checkIntercept(double x, double y);

    /**
     * @brief Function that sets centre of shape and updates verticies
     * @param x location x in [m]
     * @param y location y in [m]
     */
    void setCentre(double x, double y);

private:
    void updateVerticies();

private:
    double width_; //!< width of triangle
    double height_;//!< height of triangle
    double x1_; //!< verticies of triangle, top vertex of the isosceles triangle. 
    double y1_; //!< verticies of triangle, top vertex of the isosceles triangle.
    double x2_; //!< verticies of triangle, bottom left vertex of the isosceles triangle.
    double y2_; //!< verticies of triangle, bottom left vertex of the isosceles triangle.
    double x3_; //!< verticies of triangle, bottom right vertex of the isosceles triangle.
    double y3_; //!< verticies of triangle, bottom right vertex of the isosceles triangle.
};


#endif // TRIANGLE_H
