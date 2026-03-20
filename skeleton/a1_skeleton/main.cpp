#include <vector>
#include <iostream>
#include <thread>
#include <cmath>
#include "laser.h"
#include "sonar.h"
#include "fusion.h"
#include "cell.h"

int main(int argc, char *argv[]) {

    // Note: run with simulator already set up:
    // ACKERMAN at (0,2,0), box1 at (10,2,0.25), box2 at (10,2,1.05)

    Laser laser(pfms::PlatformType::ACKERMAN);
    Sonar sonar(pfms::PlatformType::ACKERMAN);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    auto laserData = laser.getData();
    auto pose = laser.getSensorPose();

    double sx = pose.position.x, sy = pose.position.y, syaw = pose.yaw;
    double fovRad = laser.getFieldOfView() * M_PI / 180.0;
    double angRes = laser.getAngularResolution() * M_PI / 180.0;
    double startAngle = syaw - fovRad / 2.0;
    double minR = laser.getMinRange();

    std::cout << "Sensor: (" << sx << "," << sy << ") yaw="
              << (180/M_PI)*syaw << "deg" << std::endl;

    int finiteCount = 0;
    for (auto r : laserData) {
        if (std::isfinite(r) && r >= minR) finiteCount++;
    }
    std::cout << "Finite readings: " << finiteCount << " / "
              << laserData.size() << std::endl;

    // Check the three failing cells
    struct CellInfo { double cx, cy, side; std::string name; std::string expect; };
    std::vector<CellInfo> failCells = {
        {5,  -2, 1, "cell1(5,-2)",  "FREE"},
        {4,   4, 1, "cell5(4,4)",   "FREE"},
        {9.8, 2, 1, "cell0(9.8,2)", "OCCUPIED"},
    };

    for (auto& fc : failCells) {
        double half = fc.side / 2.0;
        std::cout << "\n--- " << fc.name << " (expect " << fc.expect << ") ---" << std::endl;

        int passThrough=0, endInside=0, endPast=0, endBefore=0;

        for (size_t i = 0; i < laserData.size(); ++i) {
            double range = laserData.at(i);
            if (!std::isfinite(range) || range < minR) continue;

            double angle = startAngle + i * angRes;
            double ex = sx + range * std::cos(angle);
            double ey = sy + range * std::sin(angle);

            // Slab test
            double dx = ex-sx, dy = ey-sy;
            double tEntry=0.0, tExit=1.0;
            bool hit = true;

            if (std::abs(dx) < 1e-10) {
                if (sx < fc.cx-half || sx > fc.cx+half) hit=false;
            } else {
                double t1=(fc.cx-half-sx)/dx, t2=(fc.cx+half-sx)/dx;
                if(t1>t2) std::swap(t1,t2);
                tEntry=std::max(tEntry,t1); tExit=std::min(tExit,t2);
                if(tEntry>tExit) hit=false;
            }
            if (hit) {
                if (std::abs(dy) < 1e-10) {
                    if (sy < fc.cy-half || sy > fc.cy+half) hit=false;
                } else {
                    double t1=(fc.cy-half-sy)/dy, t2=(fc.cy+half-sy)/dy;
                    if(t1>t2) std::swap(t1,t2);
                    tEntry=std::max(tEntry,t1); tExit=std::min(tExit,t2);
                    if(tEntry>tExit) hit=false;
                }
            }

            if (!hit) continue;
            passThrough++;

            bool inCell = (ex>=fc.cx-half && ex<=fc.cx+half &&
                           ey>=fc.cy-half && ey<=fc.cy+half);
            if (inCell) {
                endInside++;
                std::cout << "  INSIDE ray[" << i << "] angle="
                          << (180/M_PI)*angle << "deg range=" << range
                          << " end=(" << ex << "," << ey << ")" << std::endl;
            } else if (tExit < 1.0) {
                endPast++;
                if (endPast <= 3)
                    std::cout << "  PAST ray[" << i << "] angle="
                              << (180/M_PI)*angle << "deg range=" << range
                              << " end=(" << ex << "," << ey << ")"
                              << " tExit=" << tExit << std::endl;
            } else {
                endBefore++;
                if (endBefore <= 3)
                    std::cout << "  BEFORE ray[" << i << "] angle="
                              << (180/M_PI)*angle << "deg range=" << range
                              << " tEntry=" << tEntry << " tExit=" << tExit << std::endl;
            }
        }
        std::cout << "Summary: passThrough=" << passThrough
                  << " endInside=" << endInside
                  << " endPast=" << endPast
                  << " endBefore=" << endBefore << std::endl;
    }

    return 0;
}