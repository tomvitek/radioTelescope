/**
 * @file eqMount_real.hpp
 * @author Tomáš Vítek (tomas.vitek.tvt@gmail.com)
 * @brief Control of a mount from SkyWatcher
 * @version 0.1
 * @date 2020-03-01
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef __EQ_MOUNT_CTRL__
#define __EQ_MOUNT_CTRL__

#include "serial.hpp"
#include "eqMount.hpp"

#define CH_RA 1
#define CH_DEC 2

#define ERROR_UNKNOWN_COMMAND 0
#define ERROR_COMMAND_LENGTH 1
#define ERROR_MOTOR_NOT_STOPED 2
#define ERROR_INVALID_CHARACTER 3
#define ERROR_NOT_INITIALIZED 4
#define ERROR_MC_SLEEPING 5
#define ERROR_PEC_TRAINING_RUNNING 7
#define ERROR_NO_PEC_DATA 8
#define ERROR_UNKNOWN 9
#define ERROR_OK 10
#define ERROR_TIMEOUT 11


class EQMount_real : public EQMount{
public:
    EQMount_real();
    ~EQMount_real();
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
    
protected:
    bool initialized = false;
    bool waitForResponse();
    /**
     * @brief  Inquire 32-bit value from mount.
     * @note   
     * @param  command: Command for the mount (without ':' and '\r')
     * @retval 
     */
    uint32_t inquireVal32(const char* command);
    /**
     * @brief  Inquire 16-bit value from mount.
     * @note   
     * @param  command: Command for the mount (without ':' and '\r')
     * @retval 
     */
    uint16_t inquireVal16(const char* command);
    uint8_t inquireVal8(const char* command);
    /**
     * @brief  Sends command to the mount.
     * @note   
     * @param  command: Command for the mount (without ':' and '\r')
     * @retval Returns false if response contains an error.
     */
    bool sendCommand(const char* command);
    bool sendCommand(const char commandStart, const uint8_t channel);
    bool sendCommand24(const char* commandStart, uint32_t data);
    bool sendCommand16(const char* commandStart, uint16_t data);
    uint8_t getStatus(uint8_t channel);

    /**
     * @brief  Sets motion mode to the mount
     * @note   
     * @param  axis: 1 for RA, 2 for DEC, 3 for both
     * @param  tracking: 0 for goto, 1 for tracking
     * @param  fast: 0 for slow, 1 for fast
     * @param  medium: 0 for slow/fast, 1 for medium
     * @param  slow_goto: 0 for normal got, 1 for slow goto
     * @param  ccw: 0 for clockwise, 1 for counterclockwise direction
     * @param  south: 0 for north, 1 for south direction
     * @param  goto_coarse: 0 for normal goto, 1 for coarse goto
     * @retval Returns true if the command was successful
     */
    bool setMotionMode(uint8_t channel, bool tracking, bool fast, bool medium, bool slow_goto, bool ccw, bool south, bool goto_coarse);

    void cleanAfter();
    void reorderData(char* data, uint8_t dataLength);
    Serial eqPort;
    /**
     * Is set to true when last command ended without error and to false when there was an error.
     */
    bool lastCommandState = true;
    /**
     * Error codes:
     * 0 - unknown command
     * 1 - command length error
     * 2 - motor not stopped
     * 3 - invalid character
     * 4 - not initialized
     * 5 - motor controller sleeping
     * 7 - PEC training is running
     * 8 - no valid PEC data
     * 9 - unknown error
     * 10 - everything ok
     * 11 - timeout
     */
    uint8_t errorCode = 10;

    long firmwareVersion;
    bool azMount;
    uint16_t timeout = 1000;
    
    /**
     * Counts per revolution of RA and DEC axis. Basically number of steps on these axises
     */
    uint32_t cpr_ra, cpr_dec;
    /**
     * Frequency of T1 timer.
     */
    uint32_t t1_freq;
    /**
     * Preset of T1 timer in motor controller. T1 timer creates interrupts with freqency of t1_freq / t1_preset. Each interrupt counts for one motor step.
     */
    uint32_t t1_preset;
    uint32_t brake_steps_ra;
    uint32_t brake_steps_dec;
    uint16_t high_speed_ratio;
    uint32_t sideral_tracking_preset;
    Velocity lastVel;
    
};

#endif