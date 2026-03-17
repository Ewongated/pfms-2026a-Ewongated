#ifndef CIRCLE_H
#define CIRCLE_H
#include "shape.h"

// Declaration of the Circle class
class Circle : public Shape{

// Public members are accessible from outside the class (ie. in main)
public:
    /**
     * @brief Constructor for Circle, centred at 0,0
     * @param radius in [m]
     */    
    Circle(double radius);

    /**
     * @brief Function that sets radius
     * @param radius in [m]
     */
    void setRadius (double radius);

    /**
     * @brief Function that returns area
     * @return area in [m2]
     */
    double getArea();
    
    /**
     * @brief Function that returns perimeter
     * @return perimeter in [m]
     */
    double getPerimeter();
    
    /**
     * @brief Function that sets area, will adjust radius and perimeter
     * @param area in [m2]
     */
    void setArea(double area);

    /**
     * @brief Function that sets perimeter, will adjust radius and area
     * @param perimeter in [m]
     */
    void setPerimeter(double perimeter);

    /**
     * @brief Function that checks if shape intercepts a point, will use the radius and centre of circle to determine if point is within circle
     * @param point location x in [m]
     * @param point location y in [m]
     */
    bool checkIntercept(double x, double y);

// Private members are only accessible from within methods of the same class
private:
    void recalculateArea(); //! Recalculates area again from the stored radius
    void recalculatePerimeter(); //! Recalculates perimeter again from the stored radius


    // This class has a double to represent the radius of the Circle
    // The trailing underscore is used to differentiate the member varibles
    // ..from local varibles in our code, this is not compulsary but HIGHLY recommended
    double radius_;

};

#endif // CIRCLE_H
