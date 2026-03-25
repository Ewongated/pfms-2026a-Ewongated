#include "tf2.h"
#include "tf.h"
#include <cmath> // for trig operations

namespace tf2 {

    //! @todo
    //! TASK 1 - Refer to README.md and the Header file for full description
    Point local2Global(RangeBearingStamped rangeBearing, Pose aircraft)
    {
        Point p;
        return p;

    }

    //! @todo
    //! TASK 2 - Refer to README.md and the Header file for full description
    RangeBearingStamped global2local(Point globalEnemy, Pose aircraft)
    {
        RangeBearingStamped rbstamped = {0, 0,0};
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
