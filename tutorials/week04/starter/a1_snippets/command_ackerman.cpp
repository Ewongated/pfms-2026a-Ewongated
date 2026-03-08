// Helper utility to send comamnd to ugv
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This executable shows the use of the pipes library to send commands
// and receive odometry from the ugv platform

#include "pfms_types.h"
#include "pfmsconnector.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>

using std::cout;
using std::endl;
using pfms::commands::Ackerman; // we state this here to be able to refer to Ackerman commands instead of the full name

int main(int argc, char *argv[]) {

    double x=10.0;
    double y=5.0;

    double brake = 0.0;
    double steering = 0.0;
    double throttle = 0.1;

    //! Created a pointer to the pfmsconnector class, which we will use to send and receive messages from the platform
    pfms::PlatformType type = pfms::PlatformType::ACKERMAN;
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr = std::make_shared<PfmsConnector>(type);
    pfms::nav_msgs::Odometry odo; // We will use this to store odometry

    
    //! We aim to drivie the Ackerman platform towards the goal, stopping at the goal withon a tolerance
    //! We will use the odometry to check if we are close to the goal
    //! We will use the Ackerman command to drive the platform
    //! We will use the pfmsconnector to send commands and receive odometry data

    // This creates a command for the Ackerman platform (refer pfms_types.h for more details)
    for (unsigned int i=0;i<50;i++){
        Ackerman cmd {
                    i++, // This is the sequence number (refer to pfms_types.h)
                    brake,
                    steering,
                    throttle,
                    };


        // This sends the command to the platform
        pfmsConnectorPtr->send(cmd);
        //! This slows down the loop to 100Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        //! This reads odometry from the platform
        bool OK  =  pfmsConnectorPtr->read(odo);

        if(OK){
            std::cout << 
                odo.time << " " <<
                odo.position.x << " " <<
                odo.position.y << " " <<
                odo.yaw << " " <<
                odo.linear.x << " " <<
                odo.linear.y << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

   return 0;
}
