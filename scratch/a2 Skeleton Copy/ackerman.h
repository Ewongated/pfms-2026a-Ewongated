#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"

class Ackerman: public Controller
{
public:
  //Default constructor should set all sensor attributes to a default value
  Ackerman();
  ~Ackerman();


private:
  Audi audi_; //!< Audi object to compute steering and distance to goal
};

#endif // ACKERMAN_H
