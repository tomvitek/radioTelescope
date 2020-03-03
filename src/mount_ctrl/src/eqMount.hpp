/**
 * @file eqMount.hpp
 * @author Tomáš Vítek (tomas.vitek.tvt@gmail.com)
 * @brief Interface for controlling EQMounts from SkyWatcher (either real or virtual)
 * @version 0.1
 * @date 2020-03-01
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __MOUNT_CTRL_EQ_MOUNT
#define __MOUNT_CTRL_EQ_MOUNT

#include <stdint.h>
#include "mount.hpp"

class EQMount : public Mount{
public:
    virtual bool begin() = 0;
    virtual void setTimeout(uint16_t timeout) = 0;

    /**
     * @brief  Sets mount's target to the given position and starts slewing to it.
     * @note   
     * @param  pos: Position of the slewing target
     * @retval 
     */
    virtual bool target_goto(Position pos) = 0;
    /**
     * @brief  Starts moving the mount with specified angular velocity.
     * @note   Moving speed won't be always the same as specified one, because it 
     * @param  vel: Velocity of the mount to move.
     * @retval 
     */
    virtual bool move(Velocity vel) = 0;

    virtual double getNearestHSpeed(double speed, bool highSpeedTracking) = 0;
    virtual double getNearestLSpeed(double speed, bool highSpeedTracking) = 0;
    virtual double getNearestSpeed(double speed, bool highSpeedTracking) = 0;

    virtual double getNormalTrackingMaxSpeed() = 0;
    virtual void moveHighSpeed(double speed_ra, double speed_dec, bool overrideSpeedLimits = false) = 0;
    /**
     * @brief  Send stop command to the mount and waits until motor on both axes are truly stopped
     * @note   See stop(uint8_t) for stopping only single axis.
     * @retval Returns true if everything was successful.
     */
    virtual bool stop() = 0;
    /**
     * @brief  Send stop command for single axis to the mount and waits until motor on specified axis doesn't stop
     * @note   See stop() for stopping both axis at the same time
     * @param  channel: Channel (axis) required to be stopped. CH_RA for right ascension, CH_DEC for declination.
     * @retval Returns true if everything was successful.
     */
    virtual bool stop(uint8_t channel) = 0;

    /**
     * @brief Returns true if the mount ended the goto move and is currently tracking or not moving at all
     */
    virtual bool getGotoFinished() = 0;
    /**
     * @brief  Check if goto command has finished
     * @note   
     * @param  channel: (optional) CH_RA for right ascension, CH_DEC for declination, 3 for both (default)
     * @retval 
     */
    virtual bool getGotoFinished(uint8_t channel) = 0;
    virtual Position getPosition() = 0;

    virtual long getFirmwareVersion() = 0;

    virtual ~EQMount() {};
};

#endif