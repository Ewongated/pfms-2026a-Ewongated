// Includes std::cout and friends so we can output to console
#include <iostream>
// Random number
#include <random>
// thread and chrono are for time and sleeping respectively
#include <chrono>
#include <thread>

using namespace std::chrono;//Let's us refer to system_clock without stating std::chrono

//Question to reflect on - Shoudl we define a tempertaure tolerance? (for our controller,
//and if so, what would this be related to)?

// Aircon unit has a few states
namespace aircon{
    enum State
    {
        IDLE
    };
}

//We create a structure that has all the variables belonging to the AirCon
struct AirCon{
    aircon::State state;
    bool heating_active;
};

// Determine the elapsed time in ms
double elapsedMs(system_clock::time_point start_time){
    //We compute time between current time and supplied start time, and return elapsed Ms
    return duration_cast<milliseconds>(system_clock::now() - start_time).count();
}

// Sleep for specified duration in ms
void sleepMs(int ms){
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Heating (would result in increase of room temperature)
void heating(AirCon& aircon){

}

// Cooling (would result in decrease of room temperature)
void cooling(AirCon& aircon){

}


// Every executable needs a main function which returns an int
int main (int argc,char** argv) {

    AirCon aircon;
    aircon.state = aircon::State::IDLE; // We need to assign IDLE here on startup


    return 0;
}
