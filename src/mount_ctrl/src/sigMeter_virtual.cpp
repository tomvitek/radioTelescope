#include "sigMeter_virtual.hpp"
#include <random>
#include <iostream>

SigMeter_virtual::SigMeter_virtual(Mount* mount){
    this->mount = mount;
}

bool SigMeter_virtual::begin(){
    // Generate random data for measurement
    for(int i = 0; i < SKY_DATA_WIDTH * SKY_DATA_HEIGHT; i++){
        skyData[i] = std::rand() % SKY_DATA_MAX;
    }
    initialized = true;
    return true;
}

int16_t SigMeter_virtual::measure(size_t samples){
    if(!initialized)
        return 0;
    Position currentPos = mount->getPosition();

    // ************************************
    // Interpolate from current position
    // ************************************
    // Derive position in skyData's perspective
    double posX = currentPos.hr * SKY_DATA_HEIGHT / 360;
    double posY = (currentPos.dec + 90) * SKY_DATA_WIDTH / 180;
    // First interpolate two x-values on default y positions:
    double x1 = truncf64(posX);
    double x2 = x1 + 1;
    double y1 = truncf64(posY);
    double y2 = y1 + 1;
    double v1 = (x2 - posX) * skyData[y1 * SKY_DATA_HEIGHT + x1] + (posX - x1) * skyData[y1 * SKY_DATA_HEIGHT + x2];
    double v2 = (x2 - posX) * skyData[y2 * SKY_DATA_HEIGHT + x1] + (posX - x1) * skyData[y2 * SKY_DATA_HEIGHT + x2];
    // Interpolate v1 and v2 on y axis
    int16_t v = (y2 - posY) * v1 + (posY - y1) * v2;
    
    return v;
}