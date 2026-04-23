#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "pfms_types.h"
#include "pfmsconnector.h"

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

class Ackerman
{
public:
  //Default constructor should set all sensor attributes to a default value
  Ackerman();
  ~Ackerman();

  void drive(void);
  void setCommand(double brake,double steer,double throttle);

private:
  unsigned int seq_;

  std::thread* thread_;
  std::atomic<bool> running_;       // We use this to indicate the thread should be runnings
  std::mutex mtx_;                  // Mutex used to lock internal member variables

  pfms::PlatformType type_;
  std::shared_ptr<PfmsConnector> pfmsConnectorPtr_;
  pfms::nav_msgs::Odometry odo_;
  double distanceTravelled_;
  pfms::commands::Ackerman cmd_;

};

#endif // ACKERMAN_H
