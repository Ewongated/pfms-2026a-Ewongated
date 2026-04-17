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
    running_ = false;
    removeValuesThread_->join();
    trimLengthThread_->join();
}

void DataBuffer::removeValues(){
    while (running_) {
        //! @todo 
        //! Given the vector is ordered, implement code to `removeValues` and make it thread safe 
        //! lock the mutex while working on vector and unlock when finished and before the sleep

        // We use unique lock rather than locking and unlocking the mutex directly
        // http://www.cplusplus.com/reference/mutex/unique_lock/
        std::unique_lock<std::mutex> lck (mtx_);

        //! @todo Implement removing values in vector, less than `minVal`and greater than `maxVal`
        val_.erase(std::remove_if(val_.begin(), val_.end(), [this](double x){return x < minVal_ || x > maxVal_;}), val_.end());

        lck.unlock(); // unlcok before sleeping here
        std::this_thread::sleep_for (std::chrono::milliseconds(20));
    }
}

void DataBuffer::addValues(vector<double> values){
        //! @todo 
        //! Implement the `addValues` function, add to and sort the vector `val_`
        //! Make it thread safe, lock the mutex while working on vector and unlock 
        //! when finished 
        // We use unique lock rather than locking and unlocking the mutex directly
        std::unique_lock<std::mutex> lck (mtx_);
        // We can use std::sort to sort the incoming values, for more effecient merging
        std::sort(values.begin(), values.end()); // Sort the incoming values

        // We can use std::inplace_merge to merge the sorted values into the existing vector
        val_.insert(val_.end(), values.begin(), values.end());    
        std::inplace_merge(val_.begin(), val_.end() - values.size(), val_.end());
        
        //We don't have to lck.unlock() as we are using unique_lock
}

void DataBuffer::trimLength(){

    while (running_) {
        //! @todo 
        //! Implement the `trimLength` function that removes elements from vector if it is grown over `trimSize_` length
        //! lock the mutex while working on vector and unlock when finished and before the sleep

        std::unique_lock<std::mutex> lck (mtx_);
        //! @todo 
        //! How to trim length to `trimSize`
        //! If the size of the vector is greater than `trimSize_`, then erase the first elements until the size is equal to `trimSize_`
        if(val_.size() > trimSize_){
            val_.erase(val_.begin(), val_.begin() + (val_.size() - trimSize_));
        }

        lck.unlock();
        std::this_thread::sleep_for (std::chrono::milliseconds(20));
    }
  
}

vector<double> DataBuffer::getValues(void){
    std::unique_lock<std::mutex> lck (mtx_);
    return val_;
}
