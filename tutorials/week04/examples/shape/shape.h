#ifndef SHAPE_H
#define SHAPE_H

#include <string>

/*!
 *  \ingroup   ac_shape Shape
 *  \brief     Shape base class
 *  \details
 *  This class is the base class for all shapes.\n
 */
class Shape
{
public:
    Shape();
    /**
     * @brief Function that sets centre of shape
     * @param x location x in [m]
     * @param y location y in [m]
     */
    void setCentre(double x, double y);

    /**
     * @brief Offset the centre of the shape, from current centre location
     * @param x in [m]
     * @param y in [m]
     */
    void offsetCentre(double x, double y);

    /**
     * @brief Function that returns description of shape
     * @return description of shape
     */
    std::string getDescription();

    /**
     * @brief Function that returns area, pure virtual function delegated to derived classes
     * @return area in [m2]
     */
    virtual double getArea() = 0; //!< We have made this pure virtual so object of class can not be instantiated
    
    /**
     * @brief Function that checks if shape intercepts a point, pure virtual function delegated to derived classes
     * @param point location x in [m]
     * @param point location y in [m]
     */
    virtual bool checkIntercept(double x, double y) =0;//!< We have made this pure virtual as well, you only need one pure virtual to guarantee object of class can not be instantiated

protected:
    std::string description_;//!< description of shape
    double area_; // The area
    double perimeter_; // The perimeter
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // SHAPE_H
