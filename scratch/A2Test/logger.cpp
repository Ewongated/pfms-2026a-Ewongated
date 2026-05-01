#include "logger.h"
#include <iostream>

namespace logger {

/**
 * @brief Reads a list of 3-D points from a whitespace-delimited text file.
 *
 * Each line must contain three values: x y z.
 * The output vector is cleared before reading.
 *
 * @param fileName Path to the input file
 * @param points   Output vector of points
 * @return true if the file was opened and at least one point was read
 */
bool loadPoints(std::string fileName, std::vector<pfms::geometry_msgs::Point>& points)
{
    std::ifstream file(fileName, std::ios::in);

    points.clear();
    if (!file.is_open()) {
        std::cerr << "Cannot open " << fileName << std::endl;
        return false;
    }

    std::string line;
    while (file.is_open()) {
        if (std::getline(file, line)) {
            std::stringstream lineStream(line);
            pfms::geometry_msgs::Point pt;
            lineStream >> pt.x;
            lineStream >> pt.y;
            lineStream >> pt.z;
            points.push_back(pt);
        } else {
            file.close();
        }
    }
    return true;
}

/**
 * @brief Saves a list of 3-D points to a comma-delimited text file.
 *
 * Each line contains: x,y,z
 * BUG FIX: original wrote point.y twice instead of point.x then point.y.
 *
 * @param fileName Path to the output file (overwritten if exists)
 * @param points   Points to save
 * @return true if the file was opened and written successfully
 */
bool savePoints(std::string fileName, std::vector<pfms::geometry_msgs::Point> points)
{
    std::ofstream file;
    file.open(fileName, std::ios::out | std::ios::trunc);

    if (!file.is_open()) {
        std::cerr << "Cannot open " << fileName << std::endl;
        return false;
    }

    for (const auto& point : points) {
        // FIX: was `point.y << "," << point.y` — x was never written
        file << point.x << "," << point.y << "," << point.z << std::endl;
    }
    file.close();
    return true;
}

} // namespace logger
