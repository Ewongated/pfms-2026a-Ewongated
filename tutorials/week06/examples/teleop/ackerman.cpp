#include "ackerman.h"
#include <iostream>
#include <cmath>

using std::cout;
using std::endl;

Ackerman::Ackerman() :
    cmd_{0,0,0,0}
{
    type_=pfms::PlatformType::ACKERMAN;
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(type_);
    running_ = true;
    thread_ = new std::thread(&Ackerman::drive,this);
}

Ackerman::~Ackerman()
{
    running_=false;
    thread_->join();
    delete thread_;
}

void Ackerman::drive(void){
    
    while(running_){    
        std::unique_lock<std::mutex> lck(mtx_);
        pfms::commands::Ackerman cmd = cmd_;
        mtx_.unlock();

        pfmsConnectorPtr_->send(cmd);
        std::this_thread::sleep_for (std::chrono::milliseconds(50));
    }

}

void Ackerman::setCommand(double brake,double steer,double throttle){

    std::unique_lock<std::mutex> lck(mtx_);
    cmd_.seq++;
    cmd_.brake=brake;
    cmd_.steering=steer;
    cmd_.throttle=throttle;
}
