#include "pfmsconnector.h"
#include <vector>
#include <iostream>
#include <thread>
#include "cell.h"

int main(int argc, char *argv[]) {


    pfms::PlatformType platform;
    platform = pfms::PlatformType::SKIDSTEER;
    pfms::nav_msgs::Odometry odo {0,2.0,0,0,0,0.0,0.0};

    //! Created a pointer to data processing
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr = std::make_shared<PfmsConnector>(platform);

    std::vector<pfms::Cell> cells;

    for (int i = 0; i < 10; ++i) {
        pfms::Cell cell(5+i, 2, 0.5);
        cells.push_back(cell);
    }

    for (auto cell : cells ) {
        pfmsConnectorPtr->send(cell);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Wait for 1 second before sending the next cell
    }

    for (auto cell : cells ) {
        // Draw from the possible cell::State values
        cell.setState(static_cast<pfms::cell::State>(rand() % 4)); // Random
        pfmsConnectorPtr->send(cell);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Wait for 1 second before sending the next cell
    }

    return 0;
}
