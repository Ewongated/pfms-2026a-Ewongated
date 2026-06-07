#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <pfmsconnector.h>
#include <thread>

class Controller: public ControllerInterface
{
  //Default constructors should set all attributes to a default value
  Controller();

  //See controllerinterface.h for more information
};

#endif // CONTROLLER_H
