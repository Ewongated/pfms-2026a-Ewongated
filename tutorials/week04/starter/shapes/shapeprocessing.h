#ifndef SHAPEPROCESSING_H
#define SHAPEPROCESSING_H

#include "shape.h"
#include <vector>

using std::vector;

class ShapeProcessing
{
public:
  ShapeProcessing(vector<Shape*> shape);

  /**
  This function indicates if the point intersects all of the shapes, removing
  the shapes that have been intercected from the internal shape container
  @param[in]    x position
  @param[in]    y position
  @param[out]   bool indicating we have intesected a shape
  */
  bool checkIntersections(double x, double y);

  /**
   * @brief getNumShapes
   * @return the number of shapes in the internal container
   */
  unsigned int getNumShapes();

private:
  vector<Shape*> shape_; //!< internal container of vector of Shape*

};



#endif // SHAPEPROCESSING_H
