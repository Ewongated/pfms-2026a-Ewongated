
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include "arrays.h"
#include <iostream>
#include <iterator>
#include <algorithm>

#include <cstring> //for memcpy example

#include <cmath>

// thread and chrono are for time and sleeping respectively - added for class
#include <chrono>
#include <thread>
using namespace std::chrono;//Let's us refer to system_clock without stating std::chrono

// function to populate array with random numbers
void populateWithRandomNumbers(double x[], unsigned int& array_size, unsigned int num_elements) {

    //we select a seed for the random generator, so it is truly random (never the same seed)
    long int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // create a normal distribution with means zero and standard deviation 10
    std::normal_distribution<> distribution(0,10.0);
    // generate the required amount of random numbers
    for (unsigned int i=array_size; i<array_size+num_elements; i++) {
        x[i] = distribution(generator);
    }
    //let's update the array size
    array_size+=num_elements;
}


//1) TASK: Implement function `assignArrayToVector`that assigns elements of array `x` to a vector named `vec`
void assignArrayToVector(double x[], unsigned int arraySize, std::vector<double> &myVec) {
    for (size_t i = 0; i < arraySize; ++i) {
        myVec.push_back(x[i]);
    }
}



//2) TASK: Implement function `removeNumbersLargerThan` that accepts vector `myVec` and removes elements of `myVec` greater than a `limit`
void removeNumbersLargerThan(std::vector<double> &myVec, double limit){
    myVec.erase(
        std::remove_if(myVec.begin(), myVec.end(), 
            [limit](double x){ return x > limit; }),
        myVec.end()
    );
}


//3) TASK: Implement function `computeMeanAndStdDev` that computes the mean and standard deviation of `myVec` and returns the `Stats` structure with these values.
Stats computeMeanAndStdDev(std::vector<double> myVec){
    Stats result;
    int n = myVec.size();

    // Compute mean
    double sum = 0.0;
    for(double x : myVec){
        sum += x;
    }
    result.mean = sum / n;

    // Compute standard deviation
    double squaredDiffSum = 0.0;
    for(double x : myVec){
        squaredDiffSum += (x - result.mean) * (x - result.mean);
    }
    result.std_dev = std::sqrt(squaredDiffSum / n);

    return result;
}

//4) TASK:  Implement function `returnVecWithNumbersSmallerThan` that returns a vector containing elements of `myVec` vector that are less than `limit`.
std::vector<double> returnVecWithNumbersSmallerThan(std::vector<double> myVec, double limit){
    removeNumbersLargerThan(myVec, limit);
    return myVec;
}


