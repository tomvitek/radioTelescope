/**
 * @file eqMount_virtual.cpp
 * @author Tomáš Vítek (tomas.vitek.tvt@gmail.com)
 * @brief Implementation of the EQMount_virtual class. Simulates connected EQ mount from SkyWatcher. UNDER CONSTRUCTION!
 * @version 0.1
 * @date 2020-03-01
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "eqMount_virtual.hpp"
#include <mutex>
#include <ctime>
#include <cmath>
#include <iostream>

#define VIRTUAL_MOUNT_DEFAULT_TIMER_FREQ 64935
#define VIRTUAL_MOUNT_DEFAULT_CPR 9024000 // As for EQ6 Pro

// Parameters of mounts
#define MOUNT_EQ6_T1_FREQ 64935
#define MOUNT_EQ6_CPR 9024000
#define MOUNT_EQ6_HIGHSPEED_RATIO 32
#define MOUNT_EQ6_GOTO_TARGET_PRESET 24
#define MOUNT_EQ6_R_PRO_T1_FREQ 64935
#define MOUNT_EQ6_R_PRO_HIGHSPEED_RATIO 32
#define MOUNT_EQ6_R_PRO_CPR 9024000
#define MOUNT_EQ6_R_PRO_GOTO_TARGET_PRESET 24

EQMount_virtual::EQMount_virtual(EQMOUNT_VIRUTAL_MOUNT mount) {
    // Set parameters of the mount according to the type chosen
    switch(mount){
        case EQMOUNT_VIRUTAL_MOUNT::EQ6:
            timer_freq = MOUNT_EQ6_T1_FREQ;
            cpr = MOUNT_EQ6_CPR;
            highspeed_ratio = MOUNT_EQ6_HIGHSPEED_RATIO;
            goto_timer_preset = MOUNT_EQ6_GOTO_TARGET_PRESET;
            break;
        case EQMOUNT_VIRUTAL_MOUNT::EQ6_R_PRO:
            timer_freq = MOUNT_EQ6_R_PRO_T1_FREQ;
            cpr = MOUNT_EQ6_R_PRO_CPR;
            highspeed_ratio = MOUNT_EQ6_R_PRO_HIGHSPEED_RATIO;
            goto_timer_preset = MOUNT_EQ6_R_PRO_GOTO_TARGET_PRESET;
            break;
        default: // Set default parameters to EQ6
            timer_freq = MOUNT_EQ6_T1_FREQ;
            cpr = MOUNT_EQ6_CPR;
            highspeed_ratio = MOUNT_EQ6_HIGHSPEED_RATIO;
            goto_timer_preset = MOUNT_EQ6_GOTO_TARGET_PRESET;
            break;
    }
}

EQMount_virtual::~EQMount_virtual() {}

bool EQMount_virtual::begin(){
    pos_ra_raw = 0;
    pos_dec_raw = cpr / 4;
    connected = true;
    last_sim_time = std::chrono::steady_clock::now();
    return true;
}

bool EQMount_virtual::target_goto(Position pos){
    simulate();
    stop();
    std::lock_guard<std::mutex> lockGuard(sim_mtx);
    if(!connected)
        return false;
    // Compute raw target position
    goto_target_pos_ra = pos.hr / 360 * cpr;
    goto_target_pos_dec = pos.dec / 360 * cpr;
    motion_stat |= MOTION_STAT::GOTO_RA | MOTION_STAT::GOTO_DEC;
    return true;
}

bool EQMount_virtual::move(Velocity vel){
    simulate();
    stop();
    std::lock_guard<std::mutex> lockGuard(sim_mtx);
    if(!connected)
        return false;
    // Compute raw velocity
    timer_preset_ra = 360 * timer_freq / cpr / fabs(vel.raps);
    timer_preset_dec = 360 * timer_freq / cpr / fabs(vel.decps);
    // Assign correct motion direction in motion_stat
    if(vel.raps > 0)
        motion_stat |= MOTION_STAT::TRACKING_RA_CCW;
    else
        motion_stat &= ~MOTION_STAT::TRACKING_RA_CCW;
    if(vel.decps > 0)
        motion_stat |= MOTION_STAT::TRACKING_DEC_NORTH;
    else
        motion_stat &= ~MOTION_STAT::TRACKING_DEC_NORTH; 
    
    // Set motion status of both channels to tracking
    motion_stat |= MOTION_STAT::TRACKING_RA | MOTION_STAT::TRACKING_DEC;
    return true;
}

void EQMount_virtual::setTimeout(uint16_t timeout){
    // Does nothing on a virtual mount
}

double EQMount_virtual::getNearestHSpeed(double vel, bool highspeedTracking){
    uint32_t hspeed_raw = (highspeedTracking ? highspeed_ratio : 1) * timer_freq * 360 / fabs(vel) / cpr;
    double hspeed = (highspeedTracking ? highspeed_ratio : 1) * timer_freq * 360.0 / hspeed_raw / cpr;
    return hspeed;
}

double EQMount_virtual::getNearestLSpeed(double vel, bool highspeedTracking){
    uint32_t lspeed_raw = ceil((highspeedTracking ? highspeed_ratio : 1) * timer_freq * 360 / fabs(vel) / cpr);
    double lspeed = (highspeedTracking ? highspeed_ratio : 1) * timer_freq * 360.0 / lspeed_raw / cpr;
    return lspeed;
}

double EQMount_virtual::getNearestSpeed(double vel, bool highspeedTracking){
    double lspeed = getNearestLSpeed(vel, highspeedTracking);
    double hspeed = getNearestHSpeed(vel, highspeedTracking);
    double resultSpeed = fabs(hspeed) - fabs(vel) > fabs(vel) - fabs(lspeed) ? lspeed : hspeed;
    return resultSpeed;
}

double EQMount_virtual::getNormalTrackingMaxSpeed(){
    return 360 * timer_freq / cpr;
}

void EQMount_virtual::moveHighSpeed(double speedRa, double speedDec, bool overrideSpeedLimits){
    simulate();
    stop();
    std::lock_guard<std::mutex> lockGuard(sim_mtx);
    if(!connected)
        return;
    // Compute raw speeds (timer interrupts)
    timer_preset_ra = 360 * timer_freq / cpr * highspeed_ratio / fabs(speedRa);
    timer_preset_dec = 360 * timer_freq / cpr * highspeed_ratio / fabs(speedDec);
    // Protect from too high speeds if possible
    if(timer_preset_ra < goto_timer_preset && !overrideSpeedLimits)
        timer_preset_ra = goto_timer_preset;
    if(timer_preset_dec < goto_timer_preset && !overrideSpeedLimits)
        timer_preset_dec = goto_timer_preset;
    
    // Assign correct motion direction in motion_stat
    if(speedRa > 0)
        motion_stat |= MOTION_STAT::TRACKING_RA_CCW;
    else
        motion_stat &= ~MOTION_STAT::TRACKING_RA_CCW;
    if(speedDec > 0)
        motion_stat |= MOTION_STAT::TRACKING_DEC_NORTH;
    else
        motion_stat &= ~MOTION_STAT::TRACKING_DEC_NORTH; 

    motion_stat |= MOTION_STAT::TRACKING_RA | MOTION_STAT::TRACKING_DEC | MOTION_STAT::HIGHSPEED_TRACKING_RA | MOTION_STAT::HIGHSPEED_TRACKING_DEC;
}

bool EQMount_virtual::stop(){
    return stop(3);
}

bool EQMount_virtual::stop(uint8_t channel){
    simulate();
    std::lock_guard<std::mutex> lockGuard(sim_mtx);
    if(channel & EQMOUNT_CH_RA)
        motion_stat &= ~(MOTION_STAT::TRACKING_RA | MOTION_STAT::HIGHSPEED_TRACKING_RA | MOTION_STAT::GOTO_RA);
    if(channel & EQMOUNT_CH_DEC)
        motion_stat &= ~(MOTION_STAT::TRACKING_DEC | MOTION_STAT::HIGHSPEED_TRACKING_DEC | MOTION_STAT::GOTO_DEC);
    return true;
}

bool EQMount_virtual::getGotoFinished(){
    simulate();
    std::lock_guard<std::mutex> lockGuard(sim_mtx);
    return !(motion_stat & (MOTION_STAT::GOTO_RA | MOTION_STAT::GOTO_DEC));
}

bool EQMount_virtual::getGotoFinished(uint8_t channel){
    simulate();
    std::lock_guard<std::mutex> lockGuard(sim_mtx);
    bool result = true;
    if(channel & EQMOUNT_CH_RA) // Right ascension
        result = result && !(motion_stat & MOTION_STAT::GOTO_RA);
    if(channel & EQMOUNT_CH_DEC) // Declination
        result = result && !(motion_stat & MOTION_STAT::GOTO_RA);
    return result;
}

Position EQMount_virtual::getPosition(){
    simulate();
    std::lock_guard<std::mutex> lockGuard(sim_mtx);
    Position pos;
    pos.hr = pos_ra_raw * 360.0 / cpr;
    pos.dec = pos_dec_raw * 360.0 / cpr;
    return pos;
}

long  EQMount_virtual::getFirmwareVersion(){
    return 0;
}

void EQMount_virtual::simulate(){
    if(!connected) // If mount isn't connected, don't simulate anything
        return;
    // lock sim_mtx to allow simulate() to be called from different threads
    std::lock_guard<std::mutex> lockGuard(sim_mtx);
    
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    int64_t delta = std::chrono::duration_cast<std::chrono::microseconds>(now - last_sim_time).count();
    last_sim_time = std::chrono::steady_clock::now();

    // Verbose:
    //std::cout << "Starting simulation..." << std::endl << "\tPosition: " << pos_ra_raw << "\t" << pos_dec_raw << "\n\tMotion mode: " << std::hex << (uint16_t)motion_stat << std::dec << "\n\tDelta: " << delta << "\n\tT1 preset: \t" << timer_preset_ra << '\t' << timer_preset_dec << std::endl;
    
    // Simulate tracking movement
    if(motion_stat & MOTION_STAT::TRACKING_RA)
        pos_ra_raw += delta * timer_freq / timer_preset_ra / 1000000 * (motion_stat & MOTION_STAT::TRACKING_RA_CCW ? 1 : -1) * (motion_stat & MOTION_STAT::HIGHSPEED_TRACKING_RA ? highspeed_ratio : 1);        
    if(motion_stat & MOTION_STAT::TRACKING_DEC)
        pos_dec_raw += delta * timer_freq / timer_preset_dec / 1000000 * (motion_stat & MOTION_STAT::TRACKING_DEC_NORTH ? 1 : -1) * (motion_stat & MOTION_STAT::HIGHSPEED_TRACKING_DEC ? highspeed_ratio : 1);
    
    if(motion_stat & MOTION_STAT::GOTO_RA){
        int32_t deltaRa = delta * timer_freq / goto_timer_preset * highspeed_ratio / 1000000 * (pos_ra_raw < goto_target_pos_ra ? 1 : -1);
        // Check if distance traveled by GOTO doesn't exceed target position
        if((pos_ra_raw + deltaRa >= goto_target_pos_ra && pos_ra_raw <= goto_target_pos_ra) || (pos_ra_raw + deltaRa <= goto_target_pos_ra && pos_ra_raw >= goto_target_pos_ra)){
            pos_ra_raw = goto_target_pos_ra;
            motion_stat &= ~MOTION_STAT::GOTO_RA;
        }
        else
            pos_ra_raw += deltaRa;
    }
    if(motion_stat & MOTION_STAT::GOTO_DEC){
        int32_t deltaDec = delta * timer_freq / goto_timer_preset * highspeed_ratio / 1000000 * (pos_dec_raw < goto_target_pos_dec ? 1 : -1);
        // Check if distance traveled by GOTO doesn't exceed target positon
        if((pos_dec_raw + deltaDec >= goto_target_pos_dec && pos_dec_raw <= goto_target_pos_dec) || (pos_dec_raw + deltaDec <= goto_target_pos_dec && pos_dec_raw >= goto_target_pos_dec)){
            pos_dec_raw = goto_target_pos_dec;
            motion_stat &= ~MOTION_STAT::GOTO_DEC;
        }
        else
            pos_dec_raw += deltaDec;
    }
}
