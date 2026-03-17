#include "pfmsconnector.h"
#include <vector>
#include <iostream>
#include <thread>

int main(int argc, char *argv[]) {

    if(argc !=2){
        std::cout << " Not arguments given on command line." << std::endl;
        std::cout << " usage: " << argv[0] << " <repeats>" << std::endl;
        return 0;
    }

    //! Created a pointer to a Pipe (This is consumer and
    pfms::PlatformType type = pfms::PlatformType::ACKERMAN;   
    std::unique_ptr<PfmsConnector> pfmsConnectorPtr = std::make_unique<PfmsConnector>(type);
    std::this_thread::sleep_for (std::chrono::seconds(5)); 

     for (unsigned int i=0;i<atoi(argv[1]);i++){
         pfms::sensor_msgs::LaserScan laserScan;
         try{
            bool OK = pfmsConnectorPtr->read(laserScan);
            if(OK){
                std::cout << "i seq x,y,yaw,vx,vy: " <<
                        i << " " << std::setprecision(14) <<
                        laserScan.time << " " << std::setprecision(6) << 
                        laserScan.angle_max << " " <<
                        laserScan.angle_min << " " <<
                        laserScan.angle_increment << " " <<
                        laserScan.range_max << " " <<
                        laserScan.range_min << " ";

                //Here we will show only the first 25 ranges
                //but in real application you would use all of them
                for (unsigned j=0;j<25;j++){
                        std::cout << laserScan.ranges.at(j) << " ";
                }
                std::cout << std::endl;
            }
            else{
                std::cout << "No laser data available" << std::endl;
            }
            std::this_thread::sleep_for (std::chrono::milliseconds(100));
        }
        catch(const std::exception& e){
            std::cerr << e.what() << '\n';
        }
     }

    return 0;
}
