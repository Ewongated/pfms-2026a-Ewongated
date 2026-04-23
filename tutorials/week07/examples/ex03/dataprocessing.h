#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H

#include <vector>
#include "radar.h"
#include <thread>
#include "data.h"

class DataProcessing
{
public:
  DataProcessing();

  ~DataProcessing();

  //! \brief Function to set the radars, we use a vector of pointers to allow us to use the same function for all radars and do an assignment for a copy
  //! \param radars Vector of pointers to radar objects
  void setRadars(std::vector<Radar*> radars);
  //! \brief Function to find the closest reading from the radar data at a fixed rate @sa runTime_
  void findClosestReading();
  //! \brief Function to find the closest reading from the radar data when new data arrives @sa newData_ , @sa cv , @sa mtx
  void findClosestReadingOnNewData();

private:
  //! \brief Function to get data from the radar, we store a local copy of the data to allow us to process at our own pace and avoid blocking
  //! \param radarIndex Index of the radar to get data from, in order to be able to use the same function for all radars
  void getData(unsigned int radarIndex);
 
  //! \brief Function to find the closest reading from a vector of vectors, assumes ranges are only positive
  double findMinRange(std::vector<std::vector<double>> rangesVec);

  std::vector<Radar*> radars_; //!< Radar objects, we use a vector of pointers to allow us to use the same function for all radars
  std::vector<pfms::Data*> dataVec_; //!< Data objects, we use a vector of pointers 
  std::vector<std::thread*> threads_; //!< Threads 

  std::mutex mtx; //!< mutex to trigger any waiting threads on sensor data arriving 
  std::condition_variable cv; //!< condition variable to trigger any waiting threads on sensor data arriving
  std::atomic<bool> newData_; //!< atomic boolean to indicate new data has arrived
  std::atomic<bool> running_; //!< atomic boolean to indicate the thread is running

  static const int runTime_ = 50; // Run time in ms

};

#endif // DATAPROCESSING_H

