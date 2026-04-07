#ifndef LASER_H
#define LASER_H

#include "ranger.h"

class Laser: public Ranger
{
public:
  //Constructor 
  Laser(pfms::PlatformType type);
  ~Laser();

 
protected:


};

#endif // LASER_H
