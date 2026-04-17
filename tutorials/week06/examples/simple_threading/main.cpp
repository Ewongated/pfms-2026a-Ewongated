#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <vector>
//#include <functional>

void addNum(int id, std::vector<int>& vec, std::mutex &mtx) {
  unsigned int count=0;
  while (count++ < 50) {
      
      //! @todo Lock the mutex to ensure that the vector is not modified by another thread
      // mtx.lock();
      std::unique_lock<std::mutex> lock(mtx); // This will automatically release the lock when it goes out of scope, or we can unlock
      vec.push_back(id*50+count);

      //! @todo Unlock the mutex to allow other threads to modify the vector
      lock.unlock(); 
      //mtx.unlock();

      // This delay is included to avoid hard-looping and consuming too much cpu
      std::this_thread::sleep_for (std::chrono::microseconds(10));
  }
}

int main ()
{
    // A vector to store the numbers
    std::vector<int> numVec;
    // We will use this mutex to synchonise access to num
    std::mutex numMutex;
    // We will create 4 threads to add numbers to the vector
    int id=0;

    // Create the threads
    std::thread t_0(addNum,id++,ref(numVec),ref(numMutex));
    std::thread t_1(addNum,id++,ref(numVec),ref(numMutex));
    std::thread t_2(addNum,id++,ref(numVec),ref(numMutex));
    std::thread t_3(addNum,id++,ref(numVec),ref(numMutex));


    // Wait for the threads to finish (they will not in this implementation)
    t_0.join();
    t_1.join();
    t_2.join();
    t_3.join();

    // Print the vector to the console
    for (int num : numVec) {
      std::cout << num << " ";
    }
    std::cout << std::endl;

    return 0;
}



