#include "charger.h"

// 1) TASK The `Charger` class is missing a special member function. 
// This function needs to enable crteating an object as per 
// the specification given in the README.md file. 


std::string Charger::getModel(void) {
    return model_;
}

unsigned int Charger::getBatteryLevel(void) {
    return batteryLevel_;
}

bool Charger::getChargingStatus(void) {   
    return charging_;
}

// Implement function according to specification in header file
bool Charger::recharge() {
    return charging_;
}