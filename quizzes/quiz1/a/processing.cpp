#include "processing.h"
#include <algorithm>
#include <limits>

// Implement function according to specification in header file
std::vector<Charger> lowestBatteryLevel(std::vector<Charger> fleet) {
    std::vector<Charger> result;
    if (fleet.empty()) return {};

    //find lowest
    unsigned int lowest = fleet[0].getBatteryLevel();
    for (auto& c : fleet)
        if (c.getBatteryLevel() < lowest)
            lowest = c.getBatteryLevel();
    
    //pushback all lowest values
    for (auto& c : fleet)
        if (c.getBatteryLevel() == lowest)
            result.push_back(c);

    return result;
}

// Implement function according to specification in header file
std::vector<Charger> eligibleForRecharge(std::vector<Charger> fleet, unsigned int batteryCutoff) {
    std::vector<Charger> result;
    if (fleet.empty()) return {};

    for (auto& c : fleet)
        if (c.getBatteryLevel() < batteryCutoff && !c.getChargingStatus())
            result.push_back(c);

    // Sort Start,End, 
    std::sort(result.begin(), result.end(), [](Charger& a, Charger& b) {
        return a.getBatteryLevel() < b.getBatteryLevel();
    });

    return result;
}