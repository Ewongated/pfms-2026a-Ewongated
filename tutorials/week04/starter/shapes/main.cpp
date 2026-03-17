
#include <iostream>
#include <limits>
#include <random>

#include "circle.h"
#include "rectangle.h"
#include "triangle.h"

#include "shapeprocessing.h" // Q) Why do we only need to include this header?

using std::cout;
using std::cin;
using std::endl;

const double size=5.0;
const double maxLength=20.0;

int main () {

int numTriangles=0,numRectangles=0,numCircles=0;

    std::vector<Shape*> shapes;  

    cout << "Enter number of circles:" << endl;
    cin >> numCircles;
    cout << "Enter number of triangles:" << endl;
    cin >> numTriangles;
    cout << "Enter number of rectangles:" << endl;
    cin >> numRectangles;


    //Let's use a random number generator for the legths
    std::random_device generator;
    std::uniform_real_distribution<double> distribution(-maxLength+(size/2),maxLength-(size/2));

    for(long int num=0;num<numTriangles;num++){
      shapes.push_back(new Triangle(size, size));
    }

    for(long int num=0;num<numRectangles;num++){
      shapes.push_back(new Rectangle(size, size));
    }

    for(long int num=0;num<numCircles;num++){
      shapes.push_back(new Circle(size));
    }

    unsigned int num=0;
    for (auto s : shapes) {
        std::cout << " #" << num++ << shapes.at(shapes.size()-1)->getDescription() <<
                     " has area " <<  (*s).getArea() << endl;
        s->setCentre(distribution(generator),distribution(generator));
    }

    ShapeProcessing sp(shapes);
    bool shapesRemaining=true;

    double x=0,y=0;

    //Keep asking user for x and y until there are no remaining shapes
    while(shapesRemaining){
        cout << "Enter position x y, example 0.1 0.2" << endl;
        cin >> x >> y;
        bool hit=sp.checkIntersections(x,y);
        if(hit){
            cout << "You Hit Some Shapes!" << endl;
        }else{
            cout << "You Missed!" << endl;
        }
        cout << "Remaining shapes: " << sp.getNumShapes() << endl;
        if (sp.getNumShapes()==0){
            shapesRemaining=false;
        } 
    }

    

    return 0;
}
