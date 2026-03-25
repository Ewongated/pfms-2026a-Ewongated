#include <iostream>
#include <vector>

#include "display.h"
#include "tf.h"
#include "tf2.h"
#include "types.h"
#include "analysis.h"

using std::vector;

void printInfo(RangeBearingStamped rb,Point bogie){

         std::cout << rb.timestamp << " [r,b]=[" << rb.range <<
                      "," << rb.bearing*180/M_PI << "]" << std::endl;

         std::cout << "pose [x,y]=[" << bogie.x << "," << bogie.y << "]" << std::endl;


}
int main (void) {

    //In this example we will create a display object with 5 bogies
    Display display(5);

    //! The aircraft is at the origin and oriented at 45 degrees
    Pose aircraft;
    aircraft.position = {0,0,0};
    aircraft.orientation = tf::yawToQuaternion(M_PI/4);

    //! The bogies are at the following positions
    std::vector<Point> bogies;
    {
        //! We can scan and supply the aircraft position as argument, it will return all 
        //! the bogies in the aircraft's frame
        std::vector<RangeBearingStamped> rbVec = display.scan(aircraft);
        for (auto rb : rbVec){
            Point bogie = tf2::local2Global(rb,aircraft);
            printInfo(rb,bogie);
            bogies.push_back(bogie);
        }
    }

    //! We now have the bogies in the global frame
    //! We can now perform analysis on the bogies
    Analysis analysis(bogies);
    //! We can get the time to impact for each bogie
    vector<double> times = analysis.timeToImpact(aircraft);

    AdjacencyList graph = analysis.exportGraph();

    int node =0;
    for (auto edges : graph){
        std::cout << node << std::endl;
        for(auto edge : edges){
            std::cout << edge.second << " " << edge.first << std::endl;
        }
    }

    return 0;
}
