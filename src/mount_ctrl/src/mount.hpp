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
    virtual bool begin() = 0;
    virtual bool target_goto(Position pos) = 0;
    virtual bool move(Velocity vel) = 0;
    virtual bool stop() = 0;
    virtual bool getGotoFinished() = 0;
    virtual Position getPosition() = 0;
};

#endif