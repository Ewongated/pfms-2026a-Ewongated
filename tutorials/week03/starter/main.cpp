#include <iostream>
#include "circle.h"

using std::cout;
using std::endl;

int main(){

    Circle circ(5.0);

    cout << circ.getDescription() << " has area:" << circ.getArea() << std::endl;

}
