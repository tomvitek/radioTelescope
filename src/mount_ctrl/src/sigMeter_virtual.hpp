#ifndef __RATEL_SIG_METER_VIRTUAL__
#define __RATEL_SIG_METER_VIRTAUL__

#include "sigMeter.hpp"
#include "mount.hpp"
#include <memory>
#include <array>

#define SKY_DATA_MAX 20000
#define SKY_DATA_WIDTH 180
#define SKY_DATA_HEIGHT 90

class SigMeter_virtual : public SigMeter{
public:
    /**
     * @brief Construct a new SigMeter_virtual object
     * 
     * @param mount Pointer to mount object, which is used to get current position data
     */
    SigMeter_virtual(Mount* mount);
    /**
     * @brief Initializes SigMeter_virtual.
     * 
     * @return true Initialization was successful
     * @return false Initialization was unsuccessful
     */
    virtual bool begin();
    /**
     * @brief Simulates measure method on real SigMeter. Returns data interpolated from pre-generated random data based on current mount's position
     * 
     * @param samples Samples of measurement. Doesn't have any effect on a virtual SigMeter
     * @return int16_t Interpolated data, in range from 0 to SKY_DATA_MAX
     */
    virtual int16_t measure(size_t samples = 1);
private:
    /**
     * @brief Indicates, wherether SigMeter_virtual has been initialized or not. If sigMeter isn't initialized by begin() function, it shouldn't allow to measure data
     */
    bool initialized = false;
    /**
     * @brief Pointer to current mount. Used to get current position data in measure function
     */
    Mount* mount;
    /**
     * @brief Data of signal strengths on different parts of sky. Generated randomly in begin function
     */
    std::array<int16_t, SKY_DATA_WIDTH * SKY_DATA_HEIGHT> skyData;
};

#endif