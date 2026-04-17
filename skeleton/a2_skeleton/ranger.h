#ifndef RANGER_H
#define RANGER_H

#include "rangerinterface.h"
#include "pfms_types.h"
#include "pfmsconnector.h"
#include <cmath>

class Ranger: public RangerInterface
{
public:
  //Constructors to connect to the sensor on the respective platform 
  Ranger(pfms::PlatformType type);


protected:

  std::shared_ptr<PfmsConnector> pfmsConnectorPtr_; //!< Pointer to the PFMS connector for communication with the platform
  pfms::PlatformType type_; //!< Stores the type of platform the ranger sensor is connected to

};

#endif // RANGER_H
