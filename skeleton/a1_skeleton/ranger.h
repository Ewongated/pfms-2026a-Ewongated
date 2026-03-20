#ifndef RANGER_H
#define RANGER_H
#include "rangerinterface.h"
#include "pfms_types.h"
#include "pfmsconnector.h"
#include <cmath>

class Ranger : public RangerInterface
{
public:
    // Implemented once in ranger.cpp, inherited by Sonar and Laser
    double                   getAngularResolution() override;
    pfms::nav_msgs::Odometry getSensorPose()        override;
    double                   getFieldOfView()       override;
    double                   getMaxRange()          override;
    double                   getMinRange()          override;
    pfms::RangerType         getSensingMethod()     override;

    // getData() remains pure virtual - each sensor reads different data
    virtual std::vector<double> getData() = 0;

protected:
    pfms::PlatformType       platformType_;
    pfms::nav_msgs::Odometry sensorPose_;
    double                   angularResolution_;
    double                   fieldOfView_;
    double                   maxRange_;
    double                   minRange_;
    pfms::RangerType         sensingMethod_;
    double                   sensorForwardOffset_;
    double                   sensorLateralOffset_;

    void computeSensorPose(const pfms::nav_msgs::Odometry& platformOdo)
    {
        double yaw = platformOdo.yaw;
        sensorPose_.position.x = platformOdo.position.x
                                + sensorForwardOffset_ * std::cos(yaw)
                                - sensorLateralOffset_ * std::sin(yaw);
        sensorPose_.position.y = platformOdo.position.y
                                + sensorForwardOffset_ * std::sin(yaw)
                                + sensorLateralOffset_ * std::cos(yaw);
        sensorPose_.position.z = platformOdo.position.z;
        sensorPose_.yaw        = yaw;
        sensorPose_.time       = platformOdo.time;
    }
};

#endif // RANGER_H