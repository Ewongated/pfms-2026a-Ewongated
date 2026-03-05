#ifndef SHAPE_H
#define SHAPE_H

#include <string>
#include <cmath> // Needed for math access to PI

/*!
 *  \ingroup   ac_shape Shape
 *  \brief     Shape base class
 *  \details
 *  This class is the base class for all shapes.\n
 */
class Shape
{
public:
    //* @brief Constructor for the shape
    //* sets the centre of the shape to 0,0
    Shape(std::string description);

    //* @brief set the centre of the shape
    //* @param x the x coordinate of the centre
    //* @param y the y coordinate of the centre 
    void setCentre(double x, double y);

    //* @brief get the description of the shape
    //* @return the description of the shape as a string
    std::string getDescription();

    //* @brief offset the shape by x and y
    //* @param x the x offset
    //* @param y the y offset   
    void offset(double x, double y);

    //* @brief get the area of the shape
    virtual double getArea(); // while present in this example, it does not make sense to have one in the base class (it should be pure virtual, which we do in example 2)

protected:
    std::string description_;//!< description of shape
protected: // NOTE: We needed to change the access to the values to allow evaluating wether the point interescts the shape.
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // SHAPE_H
