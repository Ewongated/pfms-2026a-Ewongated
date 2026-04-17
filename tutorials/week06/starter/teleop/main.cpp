#include "ackerman.h"
#include <vector>
#include "pfms_types.h"
#include <iostream>

using std::vector;
using std::cout;
using std::cin;
using std::endl;

int main(int argc, char *argv[]) {
 
    Ackerman ackerman;
    double brake=0,steering=0,throttle=0;

    cout << "You will be able to control" << endl;
    cout << "Speed via i/k." << endl;
    cout << "Steering via j/l." << endl;
    cout << "Emergency stop is o" << endl;
    cout << "Exit is . or ESC key" << endl;

    std::this_thread::sleep_for (std::chrono::seconds(5));

   char c;
   bool run=true;
   // Set the terminal to raw mode
    while(run) {
        system("stty raw");
        c = getchar(); 
        // terminate when "." is pressed
        system("stty cooked");
        system("clear");
        //std::cout << c << " was pressed."<< std::endl;
        switch(c){
            case '.' :
            case 27 : // ESC key
                system("stty cooked");
                run=0;
                break;
            case 'i' :
                throttle+=0.05;
                brake=0;
                break;
            case 'k' :
                throttle-=0.05;
                if(throttle <0){
                    brake+=1000;
                }
                break;
            case 'o' :
                throttle=0;
                brake = 8000;
                break;                
            case 'j' :
                steering+=0.2;
                break;
            case 'l' :
                steering-=0.2;
                break;
            default:
                if(throttle>0){
                    throttle-=0.01;
                }
                else if(throttle<0){
                    brake+=100;
                }
                break;
        }
        if(run){
            // We set the commands inside Ackerman here
            ackerman.setCommand(brake,steering,throttle);
        }
    }

    return 0;
}
