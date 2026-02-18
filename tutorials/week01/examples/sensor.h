//Why not #define ?
static const int max_size =100;

//http://www.cplusplus.com/doc/tutorial/structures/
struct Sensor{
    int num_samples;
    double data[max_size]; //Why have we picked this size here?
};
