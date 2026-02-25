#ifndef SHAPE_H
#define SHAPE_H

#include <string>

/*!
 *  \ingroup   ac_shapre Shape
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
     * @param x in [m]
     * @param y in [m]
     */
    void setCentre(double x, double y);
    /**
     * @brief Returns description of shape
     * @return description
     */
    std::string getDescription();

// TASK: 
// Change the access specifier here that the Rectabngle class (which is derived class) can access this member variable
// The variable should not be made public (to be accessed by an object of the Rectangle or Shape class)
private:
    std::string description_;//!< description of shape


private:
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // SHAPE_H
