#include "analysis.h"
#include <iostream>
#include <thread>
#include <chrono>

Analysis::Analysis(std::shared_ptr<Radar> radarPtr):
    radarPtr_(radarPtr)
{
}

void Analysis::computeScanningSpeed(unsigned int samples, double& scanningSpeed){
    auto start = std::chrono::steady_clock::now();

    for (unsigned int i = 0; i < samples; ++i) {
        radarPtr_->getData();
    }

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    scanningSpeed = elapsed.count() / static_cast<double>(samples);

    std::cout << "Scan period: " << scanningSpeed << " sec/scan\n";
}