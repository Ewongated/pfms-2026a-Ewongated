#ifndef PROCESSING_H
#define PROCESSING_H
#include "charger.h"
#include <vector>

/**
 * @brief TASK 3) Returns the `Charger` who has the lowest battery level in the `fleet`.
 * @param fleet vector of Charger
 * @return The vector of Chargers with the lowest battery level, if multiple Chargers 
 * have the same battery level they all need to be returned in the same order as the original vector.
 */
std::vector<Charger> lowestBatteryLevel(std::vector<Charger> fleet);

/**
 * @brief TASK 4) Function that returns Chargers that need to be recharged
 * with a battery level lower than the specified `batteryCutoff` and are not currently operational
 * @param fleet vector of Charger
 * @param batteryCutoff the battery level below which Chargers are eligible for recharge
 * @return The vector of Chargers eligible for recharge, needs to be organized in the order of 
 * lowest to highest battery level (the lowest is at the beginning and highest at the end of 
 * the vector).
 */
std::vector<Charger> eligibleForRecharge(std::vector<Charger> fleet, unsigned int batteryCutoff);

#endif // PROCESSING_H