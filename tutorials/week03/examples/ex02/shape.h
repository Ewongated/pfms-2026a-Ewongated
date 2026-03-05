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

    virtual double getArea() =0;
    virtual bool checkIntercept(double x, double y) =0; //This is a pure virtual function, making the class abstract and the derived classes must implement this function

protected:
    std::string description_;//!< description of shape
protected: // NOTE: We needed to change the access to the values to allow evaluating wether the point interescts the shape.
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // SHAPE_H
