#ifndef DATA_H
#define DATA_H

#include <vector>
#include <mutex>
#include <condition_variable>

namespace pfms {

    struct Data{
        std::vector<double> ranges; //!< vector of data from the radar
        std::mutex mtx; //!< mutex 
        std::condition_variable cv; //!< condition variable
    };

}

#endif // DATA_H
