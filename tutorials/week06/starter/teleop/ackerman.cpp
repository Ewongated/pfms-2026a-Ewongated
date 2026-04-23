#include "ackerman.h"
#include <iostream>
#include <cmath>

using std::cout;
using std::endl;

Ackerman::Ackerman() :
    cmd_{0,0,0,0}
{
    //! @todo
    // Create a `pfmsConnectorPtr_` and start a thread on the `drive` function in the constructor of `Ackerman`. 
    
}

Ackerman::~Ackerman()
{
    //! @todo
    //! Force thread to stop and join thread in destructor.
}

void Ackerman::drive(void){
    
    while(running_){    
        //! @todo
        //! lock the mutex, use `Ackerman` command cmd_  and send it via the `pfmsConnectorPtr_`

        std::this_thread::sleep_for (std::chrono::milliseconds(50));
    }

}

void Ackerman::setCommand(double brake,double steer,double throttle){
   //! @todo
   //! lock the mutex, store the values passed into cmd_
   
}
