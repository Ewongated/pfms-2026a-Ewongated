// Includes std::cout and friends so we can output to console
#include <iostream>
//For random numbers
#include <random> 
// chrono for seed
#include <chrono>
// Include the header file with the definition of the Sensor structure
#include "sensor.h"

// Ex01. Returns a bool value if the double is greater than zero
// and the square value instead of initial passed value
bool squareOfCheckPositive(double &value) {
    bool is_positive = value > 0.0;
    value *= value;
    return is_positive;
}

// Ex02 Create an function named `randomize` that accepts a structure `Sensor` and populates data with random values (mean 5.0 standard deviation 3.0)
void randomize(Sensor &sensor) {
    long seed = std::chrono::system_clock::now().time_since_epoch().count(); // This seeds random number generator
    std::default_random_engine generator(seed); // This is the random number generator
    std::normal_distribution<double> distribution(5.0, 3.0); // This is the distribution with mean 5.0 and standard deviation 3.0
    for (unsigned int i = 0; i<sensor.num_samples; i++) {
        sensor.data[i] = distribution(generator);
    }
}


// Every executable needs a main function which returns an int
int main () {

    double x = 2.0;


    // Ex01.
    double y = x;
    std::cout << x << " is ";
    bool result = squareOfCheckPositive(y);
    if (result) {
        std::cout << "positive ";
    } else {
        std::cout << "not positive ";
    }
    std::cout << "and its square is " << y << std::endl;

    // Ex02.
    Sensor sensor2 { 2, { 7, -12}}; //Create data of size two with elements noted
    randomize(sensor2);

    return 0;
}




