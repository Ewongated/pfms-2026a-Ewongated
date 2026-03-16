#include "shapeprocessing.h"
#include <iostream>
#include <algorithm>

ShapeProcessing::ShapeProcessing(vector<Shape*> shape):
  shape_(shape)
{
}

bool ShapeProcessing::checkIntersections(double x, double y){

  //! The function internally could have a number of capabilities to iterate through the vector of shapes
  //! and remove the shapes that have been intercepted by the point (x,y)

  // Example 1: Using an iterator
//  auto it = shape_.begin();
//  while (it != shape_.end()){
//    if( ((*it)->checkIntercept(x,y))){
//        std::cout << "Remove: " << (*(*it)).getDescription() << std::endl;
//         shape_.erase(it);
//    }
//    else{
//      ++it;
//    }
//  }


  // Example 2: using .at() and while loop
  //! We use a while loop, accessing the elements of the vector, check if the point intercepts the shape
  //! and if it does, we remove the shape from the vector of shapes using erase, this function has been 
  //! certified correct using Circle, Rectangle and Triangle classes, the authors refer to utest2.cpp as 
  //! evidence of correctness and usage
  if(shape_.size()>0){
    unsigned int idx=0;
    while (idx<shape_.size()){
      if( (shape_.at(idx)->checkIntercept(x,y))){
          // std::cout << "Remove: " << shape_.at(idx)->getDescription() << std::endl;
          shape_.erase(shape_.begin()+idx);
      }
      else {
        idx++;
      }
    }
  }

  // Example 3: ONLY for the die hard c++ fans : using remove_if for STL (available in algorithm)
  // and a lambda function
  //
  // remove_if
  //
  // template< class ForwardIt, class UnaryPredicate >
  // ForwardIt remove_if( ForwardIt first, ForwardIt last, UnaryPredicate p );
  // https://en.cppreference.com/w/cpp/algorithm/remove
  //
  // lambda function
  //
  // lambda function - which is a unnamed function
  // [ captures ] ( params ) { body }
  // https://en.cppreference.com/w/cpp/language/lambda
  //
  //  shape_.erase(std::remove_if(shape_.begin(),
  //                            shape_.end(),
  //                            [x,y](Shape* shape){return shape->checkIntercept(x,y);}),
  //             shape_.end());

  // We push the point that has been queries onto an internal vector
  x_.push_back(x);
  y_.push_back(y);

  return (shape_.size()==0);
}

unsigned int ShapeProcessing::getNumShapes(){
  return shape_.size();
}