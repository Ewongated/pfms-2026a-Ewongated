// Program to move ACKERMAN platform slowly forward while detecting closest laser object
// Converts closest object position to global world coordinates

#include "pfms_types.h"
#include "pfmsconnector.h"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <limits>
#include <iomanip>

using std::cout;
using std::endl;

// Laser offset from odometry reference point (meters)
const double LASER_OFFSET_X = 3.725;  // 3.725m forward from odometry point
const double LASER_OFFSET_Y = 0.0;     // centered (no lateral offset)

int main(int argc, char *argv[]) {

    if(argc != 2){
        std::cout << "Not enough arguments given on command line." << std::endl;
        std::cout << "usage: " << argv[0] << " <iterations>" << std::endl;
        return 0;
    }

    // Create pointer to PfmsConnector for ACKERMAN platform
    pfms::PlatformType type = pfms::PlatformType::ACKERMAN;
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr = std::make_shared<PfmsConnector>(type);
    
    // Wait for connection to establish
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Command parameters - moving very slowly forward
    double brake = 0.0;
    double steering = 0.0;
    double throttle = 0.07;  // Very slow forward motion

    pfms::nav_msgs::Odometry odo;
    pfms::sensor_msgs::LaserScan laserScan;

    for (unsigned int i = 0; i < atoi(argv[1]); i++){
        
        // Create and send Ackerman command
        pfms::commands::Ackerman cmd {
            i,          // sequence number
            brake,
            steering,
            throttle
        };
        pfmsConnectorPtr->send(cmd);

        // Read odometry
        bool odoOK = pfmsConnectorPtr->read(odo);
        
        // Read laser data
        bool laserOK = pfmsConnectorPtr->read(laserScan);

        if(odoOK && laserOK){
            
            // Find closest object in laser scan
            double min_range = std::numeric_limits<double>::max();
            int min_index = -1;
            
            for(unsigned int j = 0; j < laserScan.ranges.size(); j++){
                double range = laserScan.ranges[j];
                
                // Check if range is valid (between min and max)
                if(range >= laserScan.range_min && range <= laserScan.range_max){
                    if(range < min_range){
                        min_range = range;
                        min_index = j;
                    }
                }
            }

            if(min_index >= 0){
                // Calculate angle of closest point
                double angle = laserScan.angle_min + min_index * laserScan.angle_increment;
                
                // Convert from polar to Cartesian in laser frame
                double x_laser = min_range * cos(angle);
                double y_laser = min_range * sin(angle);
                
                // Transform from laser frame to vehicle frame
                double x_vehicle = x_laser + LASER_OFFSET_X;
                double y_vehicle = y_laser + LASER_OFFSET_Y;
                
                // Transform from vehicle frame to global frame using odometry
                double cos_yaw = cos(odo.yaw);
                double sin_yaw = sin(odo.yaw);
                
                double x_global = odo.position.x + x_vehicle * cos_yaw - y_vehicle * sin_yaw;
                double y_global = odo.position.y + x_vehicle * sin_yaw + y_vehicle * cos_yaw;
                
                // Output results
                cout << std::setprecision(6) << std::fixed;
                cout << "i: " << i << endl;
                cout << "  Vehicle (x, y, yaw): " 
                     << odo.position.x << ", " 
                     << odo.position.y << ", " 
                     << odo.yaw << endl;
                cout << "  Closest Object:" << endl;
                cout << "    Range: " << min_range << " m" << endl;
                cout << "    Angle: " << angle << " rad (" << (angle * 180.0 / M_PI) << " deg)" << endl;
                cout << "    Laser Frame (x, y): " << x_laser << ", " << y_laser << endl;
                cout << "    Vehicle Frame (x, y): " << x_vehicle << ", " << y_vehicle << endl;
                cout << "    Global Coordinates (x, y): " << x_global << ", " << y_global << endl;
                cout << endl;
            }
            else{
                cout << "i: " << i << " - No valid laser readings" << endl;
            }
        }
        else{
            if(!odoOK) cout << "i: " << i << " - No odometry data available" << endl;
            if(!laserOK) cout << "i: " << i << " - No laser data available" << endl;
        }

        // Sleep to control loop rate (10Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // The car will keep moving, you do know how to stop the car curtosey of quiz2a
    // Just check how you would get the velocity, or detect that it's moving, and then send a brake command to stop it (throttle = 0, brake = 1.0)
    // Of course seding brake once will not do this, justlike the quiz you need to think about how to send the brake command repeatedly until the car is stopped, and then you can exit the program
    
    return 0;
}
