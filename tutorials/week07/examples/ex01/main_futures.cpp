#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <random>    // random number generation
#include <algorithm> // algorithms for sorting
#include <iostream>
#include <future> //future and promise


using namespace std;


// Function that performs some computation and returns a result
//recursive function for fibonacci
int fibonacci(int n)
{
    //if n is zero or one return the number
    if(n<=1)
    {
        return n;
    }
    //recursive call to n-1 and n-2 
    std::this_thread::sleep_for (std::chrono::milliseconds(1));
    return fibonacci(n-1)+fibonacci(n-2);
}


int main() {
    // Create a promise object
    std::promise<int> promise;

    // Get the future associated with the promise
    std::future<int> future = promise.get_future();

    // Start a thread that will fulfill the promise
    std::thread t([&promise]() {
        int result = fibonacci(20);
        promise.set_value(result); // Set the result of the computation
    });

    // Do other work while the computation is happening...
    while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        // Do some other work while waiting here

        std::cout << "Waiting for the result..." << std::endl;
    }

    // Get the result from the future (this will block until the result is available)   
    int result = future.get();

    std::cout << "Result: " << result << std::endl;

    t.join(); // Wait for the thread to finish
    
    return 0;
}