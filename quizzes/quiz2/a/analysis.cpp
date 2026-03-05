#include "analysis.h"
#include <algorithm>

using std::vector;
using std::pair;

Analysis::Analysis(std::vector<CarInterface*> cars) :
    cars_(cars),raceDisplay_(nullptr)
{

}

Analysis::Analysis(std::vector<CarInterface*> cars,std::shared_ptr<DisplayRace> raceDisplay) :
    cars_(cars),raceDisplay_(raceDisplay)
{

}


//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::dragRace(double distance){

    std::vector<unsigned int> order(cars_.size(),0);

    return order;
}


//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
void Analysis::stopAllCars(){

}

//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::zeroTopZeroRace(){

    std::vector<unsigned int> order(cars_.size(),0);//Creating a vector, same size as cars with all zeros

    return order;
}


//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::sortByOdometry(){

    std::vector<unsigned int> order(cars_.size(),0);//Creating a vector, same size as cars with all zeros

    return order;
}



// Demo code
void Analysis::demoRace(){


    //This is an example of how to draw 3 cars moving
    // accelerate 300 times
    unsigned int count=0;

    while(count < 300){
        for(auto car : cars_){
          car->accelerate();
        }
        if(raceDisplay_!=nullptr){
            raceDisplay_->updateDisplay();
        }
        count++;
    }

    // decelerate 600 times
    count =0;
    while(count < 600){
        for(auto car : cars_){
          car->decelerate();
        }
        if(raceDisplay_!=nullptr){
            raceDisplay_->updateDisplay();
        }
        count++;
    }

}
