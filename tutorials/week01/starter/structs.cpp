// Includes std::cout and friends so we can output to console
#include <iostream>

//Why not #define ?
static const int max_size =100;

//http://www.cplusplus.com/doc/tutorial/structures/
struct Sensor{
    int num_samples;
    double data[max_size]; //Why have we picked this size here?
};
// We will introduces classes later
// - To protect data and ensure integirty (access specifiers)
// - To embed data and functionality together
// - To ensure the container is safe to use on ititialisation (constructor)

void print_struct(Sensor sensor){
  //! Loop to print elements of data in struct (let's acess each element location
    for (unsigned int i=0;i<sensor.num_samples;i++){
        std::cout << sensor.data[i] << " ";
    }
    std::cout << std::endl;
}


// Every executable needs a main function which returns an int
int main (int argc,char** argv) {

    //We create sensor variable of Sensor type
    Sensor sensor;

    //Ex02 
    //Initialise the structure with 5 samples (the values can be any values you choose)

    //Function called to print variable
    print_struct(sensor);

    //Now create sensor2 of Sensor type using list initalisation (aggregate initialisation)
    //https://en.cppreference.com/w/cpp/language/list_initialization
    //https://en.cppreference.com/w/cpp/language/aggregate_initialization

    //Function called to print variable
    //print_struct(sensor2);


    return 0;
}




