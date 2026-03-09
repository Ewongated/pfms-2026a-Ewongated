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
std::vector<unsigned int> Analysis::dragRace(double distance) {
    std::vector<unsigned int> order(cars_.size(), 0);
    std::vector<bool> finished(cars_.size(), false);
    unsigned int NumComplete = 0;

    std::vector<double> startOdo(cars_.size());
    for (size_t i = 0; i < cars_.size(); i++) {
        startOdo[i] = cars_[i]->getOdometry();
    }

    while (NumComplete < cars_.size()) {
        for (size_t i = 0; i < cars_.size(); i++) {
            if ((cars_[i]->getOdometry() - startOdo[i]) < distance) {
                cars_[i]->accelerate();
            }
            if ((cars_[i]->getOdometry() - startOdo[i]) >= distance) {
                if (finished[i] == false) {
                    finished[i] = true;
                    order[NumComplete] = cars_[i]->getID();
                    NumComplete++;
                }
            }
        }
    }
    return order;
}


//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
void Analysis::stopAllCars() {
    std::vector<bool> stopped(cars_.size(), false);
    unsigned int NumStopped = 0;

    while (NumStopped < cars_.size()) {
        for (size_t i = 0; i < cars_.size(); i++) {
            if (stopped[i]) continue;

            if (cars_[i]->getCurrentSpeed() > 1e-1) {
                cars_[i]->decelerate();
            } else {
                stopped[i] = true;
                NumStopped++;
            }
        }

        if (raceDisplay_ != nullptr) {
            raceDisplay_->updateDisplay();
        }
    }
}

//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::zeroTopZeroRace() {
    std::vector<unsigned int> order(cars_.size(), 0);
    unsigned int NumComplete = 0;

    // track each car's phase: false = still accelerating, true = now decelerating
    std::vector<bool> decelerating(cars_.size(), false);
    std::vector<bool> done(cars_.size(), false);

    while (NumComplete < cars_.size()) {
        for (size_t i = 0; i < cars_.size(); i++) {
            if (done[i]) continue;

            if (!decelerating[i]) {
                if (cars_[i]->getCurrentSpeed() >= cars_[i]->getTopSpeed()) {
                    decelerating[i] = true;   // reached top speed, now decelerate
                } else {
                    cars_[i]->accelerate();
                }
            }

            if (decelerating[i]) {
                if (cars_[i]->getCurrentSpeed() > 1e-1) {
                    cars_[i]->decelerate();
                } else {
                    // back to zero after reaching top speed - race complete 
                    done[i] = true;
                    order[NumComplete] = cars_[i]->getID();
                    NumComplete++;
                }
            }
        }

        if (raceDisplay_ != nullptr) {
            raceDisplay_->updateDisplay();
        }
    }

    return order;
}


//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::sortByOdometry() {
    std::vector<unsigned int> order(cars_.size(), 0);   
    std::vector<std::pair<double, unsigned int>> odoList;
    for (auto car : cars_) {
        odoList.push_back({car->getOdometry(), car->getID()});
    }

    // sort ascending by odometry
    std::sort(odoList.begin(), odoList.end(),
              [](const pair<double, unsigned int>& a,
                 const pair<double, unsigned int>& b) {
                  return a.first < b.first;
              });

    // fill order vector with sorted IDs
    for (size_t i = 0; i < odoList.size(); i++) {
        order[i] = odoList[i].second;
    }

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
