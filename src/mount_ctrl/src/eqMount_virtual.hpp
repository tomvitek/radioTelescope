/**
 * @file eqMount_virtual.hpp
 * @author Tomáš Vítek (tomas.vitek.tvt@gmail.com)
 * @brief Header for simulated EQ mount from SkyWatcher. UNDER CONSTRUCTION!
 * @version 0.1
 * @date 2020-03-01
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __MOUNT_CTRL_EQMOUNT_VIRTUAL
#define __MOUNT_CTRL_EQMOUNT_VIRTUAL

#include "eqMount.hpp"
#include <thread>
#include <future>

#define EQMOUNT_CH_RA 1
#define EQMOUNT_CH_DEC 2

enum EQMOUNT_VIRUTAL_MOUNT{
    EQ6,
    EQ6_R_PRO
};

class EQMount_virtual : public EQMount{
public:
    EQMount_virtual(EQMOUNT_VIRUTAL_MOUNT mount = EQMOUNT_VIRUTAL_MOUNT::EQ6);
    ~EQMount_virtual();
    virtual bool begin();

    /**
     * @brief Set timeout of the communication with real mount. Completely useless on a virtual one, though :)
     * 
     * @param timeout 
     */
    void setTimeout(uint16_t timeout);

    /**
     * @brief  Sets mount's target to the given position and starts slewing to it.
     * @note   
     * @param  pos: Position of the slewing target
     * @retval 
     */
    virtual bool target_goto(Position pos);
    /**
     * @brief  Starts moving the mount with specified angular velocity.
     * @note   Moving speed won't be always the same as specified one, because it 
     * @param  vel: Velocity of the mount to move.
     * @retval 
     */
    virtual bool move(Velocity vel);

    double getNearestHSpeed(double speed, bool highSpeedTracking);
    double getNearestLSpeed(double speed, bool highSpeedTracking);
    double getNearestSpeed(double speed, bool highSpeedTracking);

    double getNormalTrackingMaxSpeed();
    void moveHighSpeed(double speed_ra, double speed_dec, bool overrideSpeedLimits = false);
    /**
     * @brief  Send stop command to the mount and waits until motor on both axes are truly stopped
     * @note   See stop(uint8_t) for stopping only single axis.
     * @retval Returns true if everything was successful.
     */
    virtual bool stop();
    /**
     * @brief  Send stop command for single axis to the mount and waits until motor on specified axis doesn't stop
     * @note   See stop() for stopping both axis at the same time
     * @param  channel: Channel (axis) required to be stopped. CH_RA for right ascension, CH_DEC for declination.
     * @retval Returns true if everything was successful.
     */
    virtual bool stop(uint8_t channel);

    virtual bool getGotoFinished();
    /**
     * @brief  Check if goto command has finished
     * @note   
     * @param  channel: (optional) CH_RA for right ascension, CH_DEC for declination, 3 for both (default)
     * @retval 
     */
    virtual bool getGotoFinished(uint8_t channel);
    virtual Position getPosition();

    long getFirmwareVersion();
private:
    enum MOTION_STAT{
        GOTO_DEC = 0x1,
        GOTO_RA = 0x2,
        TRACKING_RA = 0x4,
        TRACKING_DEC = 0x8,
        HIGHSPEED_TRACKING_RA = 0x10,
        HIGHSPEED_TRACKING_DEC = 0x20,
        TRACKING_RA_CCW = 0x40,
        TRACKING_DEC_NORTH = 0x80
    };
    int16_t motion_stat;
    /**
     * @brief Simulates mount's position from latest simulated position to now
     * 
     */
    void simulate();
    /**
     * @brief Time of latest simulated position
     * 
     */
    std::chrono::steady_clock::time_point last_sim_time;
    /**
     * @brief Mutex for locking properties when simulating
     * 
     */
    std::mutex sim_mtx;
    int32_t pos_ra_raw;
    int32_t pos_dec_raw;
    int32_t goto_target_pos_ra;
    int32_t goto_target_pos_dec;
    int32_t goto_timer_preset;
    bool connected = false;
    uint32_t timer_preset_ra;
    uint32_t timer_preset_dec;
    uint32_t timer_freq;
    uint32_t cpr;
    uint32_t highspeed_ratio;
    
};

#endif