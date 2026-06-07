#include "tf2.h"
#include "tf.h"
#include <cmath>

namespace tf2 {

    // Helper: extract yaw from a Pose's quaternion (2D — rotation about Z only)
    static double yawFromPose(const Pose& pose) {
        const auto& q = pose.orientation;
        // Standard quaternion-to-yaw for pure Z rotation:
        // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    Point local2Global(RangeBearingStamped rangeBearing, Pose aircraft)
    {
        double yaw = yawFromPose(aircraft);

        // World angle = aircraft heading + radar bearing to target
        double worldAngle = normaliseAngle(yaw + rangeBearing.bearing);

        Point p;
        p.x = aircraft.position.x + rangeBearing.range * std::cos(worldAngle);
        p.y = aircraft.position.y + rangeBearing.range * std::sin(worldAngle);
        p.z = 0;
        return p;
    }

    RangeBearingStamped global2local(Point globalEnemy, Pose aircraft)
    {
        double yaw = yawFromPose(aircraft);

        double dx = globalEnemy.x - aircraft.position.x;
        double dy = globalEnemy.y - aircraft.position.y;

        RangeBearingStamped rbstamped = {0, 0, 0};
        rbstamped.range   = std::sqrt(dx * dx + dy * dy);
        // World angle to enemy minus aircraft yaw = bearing in drone frame
        rbstamped.bearing = normaliseAngle(std::atan2(dy, dx) - yaw);
        return rbstamped;
    }

    double normaliseAngle(double theta) {
      if (theta > (2 * M_PI))
        theta = theta - (2 * M_PI);
      else if (theta < 0)
        theta = theta + (2 * M_PI);
      if (theta > M_PI){
          theta = -( (2* M_PI) - theta);
      }
      return theta;
    }

} 