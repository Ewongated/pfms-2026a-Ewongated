#ifndef CHARGER_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define CHARGER_H

#include <string>

class Charger {
public:
  //    1) TASK The `Charger` class is missing a special member function. 
  //   This function needs to enable crteating an object as per 
  //    the specification given in the README.md file. 
  
  /**
   * @brief Function that obtains model
   * @return model of the Charger
   */
  std::string getModel(void);

  /**
   * @brief Function that retruns the battery level
   * @return battey level as int (0-100)
   */
  unsigned int getBatteryLevel(void);

  /**
   * @brief Function that retruns charging status
   * @return true if not charging, false otherwise
   */
  bool getChargingStatus(void);

  /**
   * @brief TASK 2) Function that charges the battery, will raise battery level by 25 when called. 
   * Battery level can not exceed 100.
   * @return true if battery is charged (battery at 100), false otherwise.
   */
  bool recharge();

private:
  std::string model_; //!< model of the Charger
  unsigned int batteryLevel_; //!< batteru level of the Charger
  bool charging_;  //!< currently charging (true - , false - otherwise)
};


#endif // CHARGER_H
