#include "dataprocessing.h"
#include <iostream> // Only here for showing the code is working
#include <thread>

DataProcessing::DataProcessing() :
  running_(false)
{
  radars_.clear();
  // Start thread to find closest reading
  threads_.push_back(new std::thread(&DataProcessing::findClosestReading, this));
  threads_.push_back(new std::thread(&DataProcessing::findClosestReadingOnNewData, this));
}

DataProcessing::~DataProcessing() {
  running_ = false; // Set the running flag to false
  for (auto& thread : threads_) {
    if (thread->joinable()) {
      thread->join(); // Join the thread
    }
    delete thread; // Delete the thread pointer
  }
}


void DataProcessing::setRadars(std::vector<Radar*> radars){

  //Assign the radars to the class
  radars_.clear(); // Clear the existing radars
  radars_.resize(radars.size()); // Resize the vector to hold the new radars
  radars_.assign(radars.begin(), radars.end()); // Copy the new radars into the vector
  //radars_=radars;

  // Start a thread per radar
  for (unsigned int i=0 ; i< radars_.size();i++){
    pfms::Data* data = new pfms::Data(); // Allocate memory for the data structure
    dataVec_.push_back(data);
    // Create a thread for each radar to aquire data
    threads_.push_back(new std::thread(&DataProcessing::getData, this, i));
  }

  std::cout << "DataProcessing: Number of radars: " << radars_.size() << std::endl;
  std::cout << "DataProcessing: Number of threads: " << threads_.size() << std::endl;

  running_ = true; // Set the running flag to true
}

void  DataProcessing::getData(unsigned int radarIndex) {

  // let's grab the radar and data from the vector
  Radar* radar = radars_.at(radarIndex);
  pfms::Data* data = dataVec_.at(radarIndex);

  while (true) {
    try{
      std::unique_lock<std::mutex> lock(data->mtx);
      data->ranges = radar->getData();
      data->cv.notify_all(); // Notify the condition variable
    }
    catch (const std::exception& e) {
      std::cerr << "Error acquiring data from radar " << radarIndex << ": " << e.what() << std::endl;
    }
    // Sleep for a minimal time to let other threads run
 
    std::unique_lock<std::mutex> lock(mtx);
    newData_ = true; // Set the new data flag to true, this didn't need to be locked, as atomic, but needed to wake other threads
    cv.notify_all(); // Notify all that there is new data
    lock.unlock(); // Unlock the mutex

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void DataProcessing::findClosestReading() {

  while (!running_) {
    std::cout << "No yet running: " << __func__ << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }


  while (true) {
    // We need the start time to calculate the elapsed time, using high_resolution_clock to get the most precise time, will use it to achive specfic run-time
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    //Let's get a local copy of all data and then search it, this decouples the aspect of
    //getting data from the aspect of processing it
    std::vector<std::vector<double>> rangesVec; 

    // We get the a copy of all the data from radars
    for (unsigned int i = 0; i < radars_.size(); i++) {

      pfms::Data* data = dataVec_.at(i);
      std::unique_lock<std::mutex> lock(data->mtx);

      // Copy the data from the radar
      rangesVec.push_back(data->ranges);
    }

    // We find the closest reading from the vector of vectors using the function
    double distance = findMinRange(rangesVec);

    // We have the distance here and should do any processing we need to control the robot here, before we sleep

    // Sleep to maintain a 50ms loop rate
    std::chrono::high_resolution_clock::time_point tEnd = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = tEnd - t1;
    if (elapsed.count() < runTime_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(runTime_ - static_cast<std::chrono::milliseconds::rep>(elapsed.count())));
    }

    // Output the elapsed time for the loop
    tEnd = std::chrono::high_resolution_clock::now();
    elapsed = tEnd - t1;
    std::cout << "[50ms] R: " << distance << " dt: " << elapsed.count() << " ms" << std::endl;
  }
}


void DataProcessing::findClosestReadingOnNewData() {

  while (!running_) {
    std::cout << "No yet running: " << __func__ << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }


  while (true) {
    // We need the start time to calculate the elapsed time, using high_resolution_clock to get the most precise time, will use it to achive specfic run-time
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    //Let's get a local copy of all data and then search it, this decouples the aspcet of
    //getting data from the aspect of processing it
    std::vector<std::vector<double>> rangesVec; 

    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this](){return newData_.load();}); // Wait for new data to be available, as we use atomic we need the .load() to get the value

    // We get the a copy of all the data from radars
    for (unsigned int i = 0; i < radars_.size(); i++) {

      pfms::Data* data = dataVec_.at(i);
      std::unique_lock<std::mutex> lock(data->mtx);

      // Copy the data from the radar
      rangesVec.push_back(data->ranges);
    }

    // We find the closest reading from the vector of vectors using the function
    double distance = findMinRange(rangesVec);

    // We have the distance here and should do any processing we need to control the robot here, before we sleep

    // Output the elapsed time for the loop
    std::chrono::high_resolution_clock::time_point tEnd = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = tEnd - t1;
    std::cout << "[new] R: " << distance << " dt: " << elapsed.count() << " ms" << std::endl  << std::flush;
  }
}



double DataProcessing::findMinRange(std::vector<std::vector<double>> rangesVec) {
  double distance = -1; // Default value if no readings are available
  for (auto ranges : rangesVec) {
    for (auto elem : ranges) {
      if (elem < distance || distance == -1) { // Here we take into consideration that -1 was not a valid reading to obtain one
        distance = elem;
      }
    }
  }
  return distance;
}