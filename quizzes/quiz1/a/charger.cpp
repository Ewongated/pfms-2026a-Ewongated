#include "charger.h"

// 1) TASK The `Charger` class is missing a special member function. 
// This function needs to enable crteating an object as per 
// the specification given in the README.md file. 

Charger::Charger(std::string model, unsigned int batteryLevel, bool charging)
    : model_(model), batteryLevel_(batteryLevel), charging_(charging) {}

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
    charging_ = true;
    batteryLevel_ += 25;
    if (batteryLevel_ >= 100) {
        batteryLevel_ = 100;
        charging_ = false;
    }
    return charging_;
}