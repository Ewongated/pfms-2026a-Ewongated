#include "databuffer.h"
#include <random>
#include <chrono>
#include <thread>
#include <iostream>
#include <algorithm> // Add this include for std::sort and std::inplace_merge

DataBuffer::DataBuffer(double minVal, double maxVal,unsigned int trimSize):
    minVal_(minVal),maxVal_(maxVal),trimSize_(trimSize)
{
    //! @todo 
    //! Start all the threads
    //! Syntax is
    //! thread_name = new std::thread(&ClassName::functionName,this)
    //! where ClassName is the name of the class and functionName is the name of the function to be called
    //! this is a pointer to the current object
    removeValuesThread_ = new std::thread(&DataBuffer::removeValues,this);
    trimLengthThread_ = new std::thread(&DataBuffer::trimLength,this);
    running_=true;
}

DataBuffer::~DataBuffer(){
    //! @todo 
    //! Join all threads, delete pointers
}

void DataBuffer::removeValues(){
    while (running_) {
        //! @todo 
        //! Given the vector is ordered, implement code to `removeValues` and make it thread safe 
        //! lock the mutex while working on vector and unlock when finished and before the sleep

        //! @todo Implement removing values in vector, less than `minVal`and greater than `maxVal`

        std::this_thread::sleep_for (std::chrono::milliseconds(20));
    }
}

void DataBuffer::addValues(vector<double> values){
        //! @todo 
        //! Implement the `addValues` function, add to and sort the vector `val_`
        //! Make it thread safe, lock the mutex while working on vector and unlock 
        //! when finished 

}

void DataBuffer::trimLength(){

    while (running_) {
        //! @todo 
        //! Implement the `trimLength` function that removes elements from vector if it is grown over `trimSize_` length
        //! lock the mutex while working on vector and unlock when finished and before the sleep

        std::this_thread::sleep_for (std::chrono::milliseconds(20));
    }
  
}

vector<double> DataBuffer::getValues(void){
    std::unique_lock<std::mutex> lck (mtx_);
    return val_;
}
