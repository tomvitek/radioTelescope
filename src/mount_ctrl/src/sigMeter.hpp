#ifndef __RATEL_SIG_METER__
#define __RATEL_SIG_METER__

#include "ADS1115/Adafruit_ADS1015.h"
#include <mutex>

class SigMeter{
public:
    /**
     * @brief Initializes connection to ADS1115
     * 
     * @return true Initialization has been successful
     * @return false Initialization has been unsuccessful
     */
    virtual bool begin();
    /**
     * @brief Retrieves data from ADS1115
     * 
     * @param samples Count of measurement's samples.
     * @return int16_t Measured signal strengh
     */
    virtual int16_t measure(size_t samples = 1);
private:
    Adafruit_ADS1115 ads;
    /**
     * @brief Mutex to prevent communication with the ADS1115 on two threads simultaniously
     */
    std::mutex measure_mtx;
};

#endif