#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <csignal> // For signal handling

#include "radar.h"
#include "dataprocessing.h"

// Global atomic flag to indicate program termination
std::atomic<bool> running(true);

// Signal handler for CTRL+C, this alows us to terminate the program gracefully
// This function will be called when the signal is received
// We use a signal handler to catch the CTRL+C signal and set the running flag to false
void signalHandler(int signal) {
  if (signal == SIGINT) {
    std::cout << "\nCTRL+C pressed. Exiting program..." << std::endl;
    running = false; // Set the flag to false to terminate the program
  }
}


int main (void){

  //! We create a vector of pointers to Radar
  std::vector<Radar*> radars;
  //! Push back 3 radars
  radars.push_back(new Radar);
  radars.push_back(new Radar);
  // radars.push_back(new Radar);

  //! Start thread of each radar
  for (auto radar : radars){
    radar->start();
  }

  //! Created a pointer to data processing
  std::shared_ptr<DataProcessing> dataProcessingPtr(new DataProcessing());

  std::this_thread::sleep_for (std::chrono::milliseconds(1000));
  //! Pass the radars
  dataProcessingPtr->setRadars(radars);

  //! Keep the program alive until CTRL+C is pressed
  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to reduce CPU usage
  }

  //! Call destructors for each radar
  for (auto radar : radars){
    delete radar;
  }

  //! How could we avoid having to delete the radar objects? We have seen some smart pointers, but we could also use a vector of unique pointers

  return 0;
}

