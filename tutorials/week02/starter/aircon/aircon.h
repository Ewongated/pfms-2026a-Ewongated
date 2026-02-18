#ifndef AIRCON_H
#define AIRCON_H

#include <chrono>
#include <string>

using namespace std::chrono;//Let's us refer to system_clock without stating std::chrono

// Aircon unit has a few states
namespace aircon{
    enum State
    {
        IDLE,       //Neither heating nor cooling
        COOLING,    // Cooling
        HEATING     // Heating
    };
}

class Aircon
{
public:
    /**
     * @brief Creates aircon, defaults to running it for 3minutes, 
     * Requires executing run function to peform task, becuase we do not know threading yet :)
     */
    Aircon();
   
private:
   
};

#endif // AIRCON_H
