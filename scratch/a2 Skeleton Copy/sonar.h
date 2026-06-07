#ifndef SONAR_H
#define SONAR_H
#include "ranger.h"

//Read Data
class Sonar : public Ranger
{
public:
    Sonar(pfms::PlatformType type);

    std::vector<double>      getData()              override;

private:
};

#endif // SONAR_H