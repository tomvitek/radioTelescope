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


class EQMount_virtual : public EQMount{
public:
    EQMount_virtual();
    ~EQMount_virtual();
    virtual bool begin();
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
    uint32_t pos_ra_raw;
    uint32_t pos_dec_raw;
    bool goto_running;
    bool highspeed_running;
    uint32_t timer_preset;
    uint32_t timer_freq;
};

#endif