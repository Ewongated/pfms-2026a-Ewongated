#include "pfmsconnector.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>

using std::cout;
using std::endl;
using pfms::commands::SkidSteer;

int main(int argc, char *argv[]) {

    int repeats = 100;
    double turn_l_r = 0.0;
    double move_f_b = 1.0;

    if(argc !=4){
         cout << " Not arguments given on command line." << endl;
    }
    else{
        repeats = atoi(argv[1]);
        turn_l_r = atof(argv[2]);
        move_f_b = atof(argv[5]);
        cout << "Using arguments for: " << argv[0] << endl;
    }    

    cout << "<repeats>" << repeats << 
        " <turn_l_r>" << turn_l_r << 
        "<move_f_b>" << move_f_b << endl;


    pfms::PlatformType type = pfms::PlatformType::SKIDSTEER;
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr = std::make_shared<PfmsConnector>(type);
    pfms::nav_msgs::Odometry odo;

    unsigned long i = 0;

    /* We loop sending same message for the number of times requested */
    for(i = 0; i < repeats; i ++) {
        // We take the arguments supplied on command line and place them in the uav command message
        SkidSteer cmd {i,turn_l_r,move_f_b}; 
        // Sending the commands
        pfmsConnectorPtr->send(cmd);
        // We wait for a short time, just to enable remaining of system to respond, recommended to wait
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        bool OK =  pfmsConnectorPtr->read(odo);
        // We check if the odometry returned indicates the system is running
        if(!OK){
            break;
        }
        std::cout << "i time x,y,yaw,vx,vy: " <<
            i << " " <<
            odo.time << " " <<
            odo.position.x << " " <<
            odo.position.y << " " <<
            odo.yaw << " " <<
            odo.linear.x << " " <<
            odo.linear.y << " " <<
            odo.linear.z << std::endl;
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }

    return 0;
}
