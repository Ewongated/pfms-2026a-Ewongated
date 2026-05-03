#include "pfms_types.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

#include "ackerman.h"
#include "skidsteer.h"
#include "mission.h"
#include "logger.h"

using namespace pfms::nav_msgs;

/**
 * @brief Entry point for the a2 mission executable.
 *
 * Usage: ./a2 <ackerman_file> <skidsteer_file> [-advanced]
 *
 * Reads goal lists from two text files, creates one Ackerman and one
 * Skidsteer controller, hands them to Mission, then polls mission status
 * at 200 ms until both platforms reach 100% completion.
 */
int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cout << "Usage:   " << argv[0] << " <ackerman_file> <skidsteer_file> [-advanced]\n";
        std::cout << "Example: " << argv[0] << " ../data/ACKERMAN.TXT ../data/SKIDSTEER.TXT\n";
        std::cout << "Advanced:" << argv[0] << " ../data/ACKERMAN.TXT ../data/SKIDSTEER.TXT -advanced\n";
        return 1;
    }

    const std::string ackerman_filename  = argv[1];
    const std::string skidsteer_filename = argv[2];

    pfms::MissionObjective objective = pfms::MissionObjective::BASIC;
    if (argc >= 4 && std::strcmp(argv[3], "-advanced") == 0) {
        objective = pfms::MissionObjective::ADVANCED;
        std::cout << "Advanced Mode Activated\n";
    }

    std::vector<pfms::geometry_msgs::Point> ackermanPoints;
    std::vector<pfms::geometry_msgs::Point> skidsteerPoints;

    if (!logger::loadPoints(ackerman_filename, ackermanPoints)) {
        std::cout << "Could not load points from file: " << ackerman_filename << "\n";
        return 0;
    }
    if (!logger::loadPoints(skidsteer_filename, skidsteerPoints)) {
        std::cout << "Could not load points from file: " << skidsteer_filename << "\n";
        return 0;
    }

    std::cout << "Ackerman goals:  " << ackermanPoints.size()  << "\n";
    std::cout << "Skidsteer goals: " << skidsteerPoints.size() << "\n";

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Skidsteer());
    controllers.push_back(new Ackerman());
    controllers.front()->setTolerance(0.5);
    controllers.back()->setTolerance(0.5);

    Mission mission(controllers);
    mission.setMissionObjective(objective);
    mission.setGoals(skidsteerPoints, pfms::PlatformType::SKIDSTEER);
    mission.setGoals(ackermanPoints,  pfms::PlatformType::ACKERMAN);

    mission.execute(true); // non-blocking

    bool OK = false;
    while (!OK) {
        std::vector<unsigned int> progress = mission.status();
        if (progress.front() == 100 && progress.back() == 100) {
            OK = true;
        } else {
            std::cout << "progress ... " << progress.front() << "% "
                      << progress.back() << "%\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    std::cout << "Mission complete.\n";

    for (auto* controller : controllers) {
        delete controller;
    }

    return 0;
}
