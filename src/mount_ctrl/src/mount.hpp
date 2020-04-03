/**
 * @file eqMount_virtual.hpp
 * @author Tomáš Vítek (tomas.vitek.tvt@gmail.com)
 * @brief Interface for basic operations with all mounts
 * @version 0.1
 * @date 2020-03-01
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef __MOUNT_CTRL_MOUNT
#define __MOUNT_CTRL_MOUNT

class Position{
public:
    /**
     * @brief Construct a new Position object
     * 
     * @param hr Hour angle
     * @param dec Declination
     */
    Position(double hr = 0.0, double dec = 0.0);
    /**
     * @brief  Hour angle
     * @note   
     * @retval None
     */
    double hr;
    /**
     * @brief  Right ascension
     * @note   
     * @retval None
     */
    double dec;
};

class Velocity{
public:
    /**
     * @brief Angular velocity in right ascension axis in degrees per second 
     * @note   
     * @retval None
     */
    double raps;
    /**
     * @brief  Angular velocity in declination axis in degrees per second
     * @note   
     * @retval None
     */
    double decps;

    Velocity(double speed_ra, double speed_dec);
    Velocity();
};

class Mount{
public:
    /**
     * @brief Starts the communication with the mount
     * 
     * @return true Initialization and connection has been successful
     * @return false Initialization and connection has been unsuccessful
     */
    virtual bool begin() = 0;
    /**
     * @brief Sets mount's target to a given position and starts slewing to it
     * 
     * @param pos Position of the target
     * @return true goto command was successful
     * @return false goto command was unsuccessful
     */
    virtual bool target_goto(Position pos) = 0;
    /**
     * @brief Starts moving the mount by given angular velocity
     * 
     * @param vel Velocity
     * @return true move command was successful
     * @return false move command was unsuccessful
     */
    virtual bool move(Velocity vel) = 0;
    /**
     * @brief Stops the mount
     * 
     * @return true Stop command was successful
     * @return false Stop command was unsuccessful
     */
    virtual bool stop() = 0;
    /**
     * @brief Returns true, if the goto command has been finished and false if it is still running
     */
    virtual bool getGotoFinished() = 0;
    /**
     * @brief Returns mount's current position
     * 
     * @return Position 
     */
    virtual Position getPosition() = 0;
};

#endif