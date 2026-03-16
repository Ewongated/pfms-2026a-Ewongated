#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>
#include <memory> //For unique_ptr

//header files needed from our libraries
#include "rectangle.h"
#include "circle.h"
#include "triangle.h"
#include "shapeprocessing.h"
using std::cout;


TEST (ShapeProcessingTest, Intersect) {

  vector<Shape* > shapes;
  shapes.push_back(new Rectangle(3.0,3.0));
  shapes.push_back(new Circle(1.5));
  shapes.push_back(new Triangle(3.0,3.0));  

  std::unique_ptr<ShapeProcessing> sp = std::make_unique<ShapeProcessing>(shapes);
  ASSERT_FALSE(sp->checkIntersections(2.0,2.0));
  ASSERT_EQ(sp->getNumShapes(),3);
  ASSERT_FALSE(sp->checkIntersections(0.2,1.4));
  ASSERT_EQ(sp->getNumShapes(),1);
  ASSERT_TRUE(sp->checkIntersections(0.0,0.0));
  ASSERT_EQ(sp->getNumShapes(),0);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
