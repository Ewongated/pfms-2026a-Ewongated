#ifndef LASER_H
#define LASER_H
#include "ranger.h"
//Read Data
class Laser : public Ranger
{
public:
    Laser(pfms::PlatformType type);

    std::vector<double> getData() override;
    bool setAngularResolution(double angularResolution);

private:
};

#endif // LASER_H