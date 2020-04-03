#include "sigMeter.hpp"

#include <exception>

#define GAIN GAIN_TWOTHIRDS
#define DEFAULT_CHANNEL 0

bool SigMeter::begin(){
    ads.begin();
    ads.setGain(GAIN_TWOTHIRDS);
    return true;
}

int16_t SigMeter::measure(size_t samples){
    std::lock_guard<std::mutex> lockGuard(measure_mtx); // Lock measure_mtx, so noone can try to measure simultaneusly
    
    if(samples == 0)
        throw std::invalid_argument("Samples cannot be zero");
    int32_t measured = 0;
    for(int i = 0; i < samples; i++){
        measured += ads.readADC_SingleEnded(DEFAULT_CHANNEL);
    }
    measured /= samples;

    return (int16_t)measured;
}