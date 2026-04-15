#include <vector>
#include <iostream>
#include <thread> // added to delay the execution of the code (for visualisation)

#include "laser.h"
#include "sonar.h"
#include "fusion.h"
#include "cell.h"
#include <random>

int main(int argc, char *argv[]) {


    std::vector<RangerInterface*> rangers;
    // Create instances of Laser and Sonar sensors from ACKERMAN platform
    Laser laser (pfms::PlatformType::ACKERMAN);
    Sonar sonar (pfms::PlatformType::ACKERMAN);
    rangers.push_back(&laser);
    rangers.push_back(&sonar);

    // Add the sensors to fusion
    Fusion fusion(rangers);

    // Create a few cells to test the fusion
    std::vector<pfms::Cell*> cells;
    cells.push_back(new pfms::Cell(17,6.5,2.0));
    cells.push_back(new pfms::Cell(4,-5,1.0));
    cells.push_back(new pfms::Cell(12,2,0.5));
    cells.push_back(new pfms::Cell(8,2,1.0));

    double x, y;

    for (unsigned int i=0; i<cells.size(); i++) {

        cells.at(static_cast<unsigned long>(i))->getCentre(x,y);
        std::cout << "Cell #" << i+1 << " is located at " << x << ", " << y << " with side length "
                  << cells.at(i)->getSide() << std::endl;

    }

    int cnt = 0;
    for (auto i : rangers) {
        auto sensor_type = i->getSensingMethod();
        auto sensor_location = i->getSensorPose();
        auto sensor_resolution = i->getAngularResolution();

        std::cout << "\nsensor #" << ++cnt << "\ntype is: " << sensor_type << "\nlocation is: ("
                  << sensor_location.position.x << ", " << 
                  sensor_location.position.y << ", " << 
                  (180/M_PI)*sensor_location.yaw
                  << ") \nsensor resolution is: " << sensor_resolution << std::endl;
    }

    // Set the cells to fusion
    fusion.setCells(cells);

    while (true) {
        // Grab the data from the sensors and fuse it
        fusion.grabAndFuseData();
        std::cout << " " << std::endl;
        for (int i=0; i<cells.size(); i++) {
            std::cout << "Cell #" << i+1 << " state is: " << cells.at(static_cast<unsigned long>(i))->getState() << std::endl;
            cells.at(static_cast<unsigned long>(i))->setState(pfms::cell::UNKNOWN);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}
