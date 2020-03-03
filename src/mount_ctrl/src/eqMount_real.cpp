/**
 * @file eqMount_real.cpp
 * @author Tomáš Vítek (tomas.vitek.tvt@gmail.com)
 * @brief Implementation for control of EQMount from SkyWatcher.
 * @version 0.1
 * @date 2020-03-01
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "eqMount_real.hpp"
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <string>

#define MOUNT_RESPONSE_TIME 20

enum CHANNEL_STATUS{
    MODE_TRACKING = 0x1,
    ROT_CCW = 0x2,
    VEL_FAST = 0x4,
    MOTOR_RUNNING = 0x8,
    BLOCKED = 0x10,
    INIT = 0x20,
    SWITCH_ON = 0x40,
    FAILED = 0x80
};

Velocity::Velocity(double speed_ra, double speed_dec){
    raps = speed_ra;
    decps = speed_dec;
}

Velocity::Velocity(){
    raps = 0.0;
    decps = 0.0;
}

void EQMount_real::reorderData(char* data, uint8_t dataLength){
    char data_backup[dataLength];
    memcpy(data_backup, data, sizeof(data_backup));

    for(int i = 0; i < dataLength; i += 2){
        data[i] = data_backup[dataLength - i - 2];
        data[i + 1] = data_backup[dataLength - i - 1];
    }
}

EQMount_real::EQMount_real(){
    
}

EQMount_real::~EQMount_real(){
    if(initialized)
        eqPort.closePort();
}

bool EQMount_real::begin(){
    std::cout << "Initializing the mount ... " << std::endl;
    eqPort.begin(9600);
    // Reqest motor controller firmware version
    //eqPort.print(":e1\r");
    
    //if(!waitForResponse())
       // return false;

    // TODO firmwareVersion = ;

    // Get CPRs

    
    cpr_ra = inquireVal32("a1");
    cpr_dec = inquireVal32("a2");
    std::cout << "CPR: " << cpr_ra << std::endl;
    // Get timer interrup frequency
    t1_freq = inquireVal32("b1");

    // Get high speed ratio
    high_speed_ratio = inquireVal8("g1");

    // Get brake steps (whatever they are)
    brake_steps_ra = inquireVal32("c1");
    brake_steps_dec = inquireVal32("c2");

    // Get 1X tracking period
    sideral_tracking_preset = inquireVal32("D1");

    // Set positon of ra and dec (if mount is not initialized already)
    if(!(getStatus(1) & CHANNEL_STATUS::INIT))
        sendCommand24("E1", 0x800000);
    if(!(getStatus(2) & CHANNEL_STATUS::INIT))
        sendCommand24("E2", cpr_dec / 2 + 0x800000);

    // Set both axis to initialized
    sendCommand("F1");
    sendCommand("F2");
    initialized = true;
    return true;
}

void EQMount_real::setTimeout(uint16_t timeout){
    this->timeout = timeout;
}

bool EQMount_real::target_goto(Position pos){
    bool result;
    // Check status of motors. If they are running, stop them
    if(!stop())
        return false;

    Position currentPos = getPosition();
    bool moveRa = abs(pos.hr - currentPos.hr) != 0;
    bool moveDec = abs(pos.dec - currentPos.dec) != 0;
    if(moveRa) setMotionMode(1, 0, abs(pos.hr - currentPos.hr) > 2.0, 0, 0, 0, 1, 0);
    if(moveDec) setMotionMode(2, 0, abs(pos.dec - currentPos.dec) > 2.0, 0, 0, 0, 1, 0);
    // Convert position from degrees to steps
    uint32_t ra_raw = pos.hr * cpr_ra / 360 + 0x800000;
    uint32_t dec_raw = (pos.dec + 90) * cpr_dec / 360 + 0x800000;

    if(moveRa) sendCommand24("S1", ra_raw);
    if(moveDec) sendCommand24("S2", dec_raw);

    // Start motion
    if(moveRa) sendCommand("J1");
    if(moveDec) sendCommand("J2");

    return !result;
}

bool EQMount_real::move(Velocity vel){
    uint8_t statusRa = getStatus(1);
    uint8_t statusDec = getStatus(2);

    // Determine if stop of the motor is required (when it is currently set to goto, high speed tracking or is moving in a opposite direction)
    bool stopRequiredRa = !(statusRa & CHANNEL_STATUS::MODE_TRACKING) || (statusDec & CHANNEL_STATUS::VEL_FAST) || vel.raps == 0.0;
    bool stopRequiredDec = !(statusDec & CHANNEL_STATUS::MODE_TRACKING) || (statusDec & CHANNEL_STATUS::VEL_FAST) || vel.decps == 0.0;
    // Stop motion if neccessary
    if(vel.raps / fabs(vel.raps) != lastVel.raps / fabs(lastVel.raps))
        stopRequiredRa = true;
    if(vel.decps / fabs(vel.decps) != lastVel.decps / fabs(lastVel.decps))
        stopRequiredDec = true;
    
    if(stopRequiredRa)
        stop(1);
    if(stopRequiredDec)
        stop(2);

    // Check if axes are requiered to move.
    bool moveRa = vel.raps != 0;
    bool moveDec = vel.decps != 0;

    // Compute raw speeds an both axes
    uint32_t vel_ra_raw(100), vel_dec_raw(100); // Set default values to some random non-zero number, so if speed in given axis is zero, it won't matter in checking for high speed
    if(moveRa) vel_ra_raw = ceil(t1_freq * 360 / fabs(vel.raps) / cpr_ra);
    if(moveDec) vel_dec_raw = ceil(t1_freq * 360 / fabs(vel.decps) / cpr_dec);

    // Set motion modes on both axes to correct values. If stop wasn't required, motion mode is already correct.
    if(moveRa && stopRequiredRa) setMotionMode(CH_RA, 1, 0, 0, 0, 0, vel.raps > 0.0, 0);
    if(moveDec && stopRequiredDec) setMotionMode(CH_DEC, 1, 0, 0, 0, 0, vel.decps < 0.0, 0);
    
    // Set t1_preset on both axis to their correct value for specified speed
    if(moveRa) sendCommand24("I1", vel_ra_raw);
    if(moveDec) sendCommand24("I2", vel_dec_raw);

    // Move, mount!
    if(moveRa) sendCommand("J1");
    if(moveDec) sendCommand("J2");

    lastVel = vel;

    return true;
}

double EQMount_real::getNearestLSpeed(double speed, bool highSpeedTracking){
    uint32_t lspeed_raw = ceil((highSpeedTracking ? high_speed_ratio : 1) * t1_freq * 360 / fabs(speed) / cpr_ra);
    double lspeed = (highSpeedTracking ? high_speed_ratio : 1) * t1_freq * 360.0 / lspeed_raw / cpr_dec;
    return lspeed;
}

double EQMount_real::getNearestHSpeed(double speed, bool highSpeedTracking){
    uint32_t hspeed_raw = (highSpeedTracking ? high_speed_ratio : 1) * t1_freq * 360 / fabs(speed) / cpr_ra;
    double hspeed = (highSpeedTracking ? high_speed_ratio : 1) * t1_freq * 360.0 / hspeed_raw / cpr_dec * speed / fabs(speed);
    return hspeed;
}

double EQMount_real::getNearestSpeed(double speed, bool highSpeedTracking){
    double lspeed = getNearestLSpeed(speed, highSpeedTracking);
    double hspeed = getNearestHSpeed(speed, highSpeedTracking);
    double resultSpeed = fabs(hspeed) - fabs(speed) > fabs(speed) - fabs(lspeed) ? lspeed : hspeed;
    return resultSpeed;
}

double EQMount_real::getNormalTrackingMaxSpeed(){
    return t1_freq * 360.0 / cpr_dec;
}

void EQMount_real::moveHighSpeed(double speed_ra, double speed_dec, bool overrideSpeedLimits){
    // Stop motion if neccessary
    stop();

    // Check if axes are requiered to move.
    bool moveRa = speed_ra != 0;
    bool moveDec = speed_dec != 0;

    // Compute raw speeds an both axes
    uint32_t vel_ra_raw(100), vel_dec_raw(100); // Set default values to some random non-zero number, so if speed in given axis is zero, it won't matter in checking for high speed
    if(moveRa) vel_ra_raw = high_speed_ratio * t1_freq * 360 / fabs(speed_ra) / cpr_ra;
    if(moveDec) vel_dec_raw = high_speed_ratio * t1_freq * 360 / fabs(speed_dec) / cpr_dec;

    // Check if speed in high speed tracking isn't greater than 24, which might be a bit too much for the mount. (mount + too great speed = possibly dead mount)
    if(((!overrideSpeedLimits) && (vel_ra_raw < 24)) || ((!overrideSpeedLimits) && (vel_dec_raw < 24))){
        std::cout << "Too high speed!" << std::endl;
        return;
    }

    // Set motion modes on both axes to correct values.
    if(moveRa) setMotionMode(CH_RA, 1, 1, 0, 0, 0, speed_ra < 0.0, 0);
    if(moveDec) setMotionMode(CH_DEC, 1, 1, 0, 0, 0, speed_dec < 0.0, 0);
    
    // Set t1_preset on both axis to their correct value for specified speed
    if(moveRa) sendCommand24("I1", vel_ra_raw);
    if(moveDec) sendCommand24("I2", vel_dec_raw);

    // Move, mount!
    if(moveRa) sendCommand("J1");
    if(moveDec) sendCommand("J2");

    return;
}

bool EQMount_real::stop(){
    bool result;
    result = !sendCommand("K1");
    result |= !sendCommand("K2");

    while(getStatus(CH_RA) & CHANNEL_STATUS::MOTOR_RUNNING)
        std::this_thread::sleep_for(std::chrono::milliseconds(MOUNT_RESPONSE_TIME));
    while(getStatus(CH_DEC) & CHANNEL_STATUS::MOTOR_RUNNING)
        std::this_thread::sleep_for(std::chrono::milliseconds(MOUNT_RESPONSE_TIME));

    return !result;
}

bool EQMount_real::stop(uint8_t channel){
    bool result;
    eqPort.print(":K");
    eqPort.print(channel);
    eqPort.print('\r');
    result = waitForResponse();
    cleanAfter();

    while(getStatus(channel) & CHANNEL_STATUS::MOTOR_RUNNING)
        std::this_thread::sleep_for(std::chrono::milliseconds(MOUNT_RESPONSE_TIME));

    return result;
}

Position EQMount_real::getPosition(){
    uint32_t hr_raw = inquireVal32("j1");
    uint32_t dec_raw = inquireVal32("j2");
    Position pos;
    pos.hr = (((double)hr_raw) - 0x800000) * 360 / cpr_ra;
    pos.dec = (((double)dec_raw) - 0x800000) * 360 / cpr_dec - 90.0;

    return pos;
}

bool EQMount_real::getGotoFinished(){
    return getGotoFinished(3);
}

bool EQMount_real::getGotoFinished(uint8_t channel){
    // When goto is finished, channel status is automatically set to tracking.
    if(channel == 3) // If channel 3 is received as parameter, check for both channels (default)
        return (getStatus(1) & CHANNEL_STATUS::MODE_TRACKING) && (getStatus(2) & CHANNEL_STATUS::MODE_TRACKING);
    else
        return getStatus(channel) & CHANNEL_STATUS::MODE_TRACKING;
}

bool EQMount_real::waitForResponse(){
    // Wait until it responses or times out 
    std::this_thread::sleep_for(std::chrono::milliseconds(MOUNT_RESPONSE_TIME));
    
    // Get the first character of the response. If it is not '=' (which is the default character for response), raise an error and return false
    uint8_t returnCode = 0;
    eqPort.readData(&returnCode, sizeof(returnCode));

    if(returnCode != '='){
        if(returnCode == '!'){ // Check for error sent by mount
            eqPort.readData(&errorCode, 1);
            errorCode -= '0';
            std::cout << "MOUNT ERROR RECEIVED: " << errorCode << std::endl;
        }
        else{
            errorCode = 9;
            std::cout << "MOUNT - UNKNOWN ERROR: " << (uint32_t)returnCode << std::endl;
        }
        lastCommandState = false;
        return false;
    }

    lastCommandState = true;
    errorCode = 10;
    return true;
}

uint32_t EQMount_real::inquireVal32(const char* request){
    eqPort.print(":");
    eqPort.print(request);
    eqPort.print("\r");
    if(!waitForResponse()) return 0;

    char response[7];
    eqPort.readData(response, sizeof(response));
    reorderData(response, 6);
    uint32_t val = strtoul(response, nullptr, 16);
    return val;
}

uint16_t EQMount_real::inquireVal16(const char* request){
    eqPort.print(":");
    eqPort.print(request);
    eqPort.print('\r');
    if(!waitForResponse()) return 0;

    char response[5];
    eqPort.readData(response, sizeof(response));
    reorderData(response, 4);
    uint16_t val = strtol(response, nullptr, 16);
    return val;
}

uint8_t EQMount_real::inquireVal8(const char* request){
    eqPort.print(":");
    eqPort.print(request);
    eqPort.print('\r');
    if(!waitForResponse()) return 0;

    char response[3];
    eqPort.readData(response, sizeof(response));
    
    uint8_t val = strtol(response, nullptr, 16);
    return val;
}

bool EQMount_real::sendCommand(const char* command){
    eqPort.print(':');
    eqPort.print(command);
    eqPort.print('\r');
    bool result = waitForResponse();
    cleanAfter(); // Read carriage return character
    return result;
}

bool EQMount_real::sendCommand(const char commandStart, const uint8_t channel){
    eqPort.print(':');
    eqPort.print(commandStart);
    eqPort.print(channel);
    eqPort.print('\r');
    bool result = waitForResponse();
    cleanAfter(); // Read carriage return character
    return result;
}

bool EQMount_real::sendCommand24(const char* commandStart, uint32_t dataForConversion){
    uint8_t data_text[7];
    for(int i = 0; i < 7; i++) // Set all bytes to '0', so when small number arrives, there won't be random characters
        data_text[i] = '0';

    data_text[0] = (dataForConversion & 0xF00000) >> 20;
    data_text[1] = (dataForConversion & 0xF0000) >> 16;
    data_text[2] = (dataForConversion & 0xF000) >> 12;
    data_text[3] = (dataForConversion & 0xF00) >> 8;
    data_text[4] = (dataForConversion & 0xF0) >> 4;
    data_text[5] = (dataForConversion & 0xF);

    eqPort.print(":");
    eqPort.print(commandStart);
    // Send reordered data translated to HEX to eqPort
    for(int i = 5; i >= 0; i-= 2){
        eqPort.printHex(data_text[i - 1]);
        eqPort.printHex(data_text[i]);
    }
    eqPort.print('\r');

    bool result = waitForResponse();
    cleanAfter();

    return result;
}

bool EQMount_real::sendCommand16(const char* commandStart, uint16_t data){
    uint8_t data_text[4];
    for(int i = 0; i < 4; i++) // Set all bytes to '0', so when small number arrives, there won't be random characters
        data_text[i] = '0';

    data_text[0] = (data & 0xF000) >> 12;
    data_text[1] = (data & 0xF00) >> 8;
    data_text[2] = (data & 0xF0) >> 4;
    data_text[3] = (data & 0xF);
    
    eqPort.print(":");
    eqPort.print(commandStart);
    eqPort.printHex(data_text[2]);
    eqPort.printHex(data_text[3]);
    eqPort.printHex(data_text[0]);
    eqPort.printHex(data_text[1]);
    eqPort.print('\r');

    bool result = waitForResponse();
    cleanAfter();

    return result;
}

uint8_t EQMount_real::getStatus(uint8_t channel){
    eqPort.print(":f");
    eqPort.print(channel);
    eqPort.print("\r");
    if(!waitForResponse())
        return CHANNEL_STATUS::FAILED;
    
    uint8_t status = 0;
    // Read 3 bytes of status info from the mount
    uint8_t statusRaw[3];
    eqPort.readData(statusRaw, sizeof(statusRaw));

    cleanAfter();

    // Convert to real numbers from ASCII
    statusRaw[0] -= '0';
    statusRaw[1] -= '0';
    statusRaw[2] -= '0';

    // Interpret received values
    status |= statusRaw[0] & 0x1 ? CHANNEL_STATUS::MODE_TRACKING : 0;
    status |= statusRaw[0] & 0x2 ? CHANNEL_STATUS::ROT_CCW : 0;
    status |= statusRaw[0] & 0x4 ? CHANNEL_STATUS::VEL_FAST : 0;
    status |= statusRaw[1] & 0x1 ? CHANNEL_STATUS::MOTOR_RUNNING : 0;
    status |= statusRaw[1] & 0x2 ? CHANNEL_STATUS::BLOCKED : 0;
    status |= statusRaw[2] & 0x1 ? CHANNEL_STATUS::INIT : 0;
    status |= statusRaw[2] & 0x2 ? CHANNEL_STATUS::SWITCH_ON : 0;

    return status;
}

void EQMount_real::cleanAfter(){
    char uselessBytes[1];
    eqPort.readData(uselessBytes, sizeof(uselessBytes));
}

bool EQMount_real::setMotionMode(uint8_t axis, bool tracking, bool fast, bool medium, bool slow_goto, bool ccw, bool south, bool goto_coarse){
    // Initialize motion mode params field
    uint8_t motionModeParams[2];
    motionModeParams[0] = 0;
    motionModeParams[1] = 0;

    // Set correct values to each bit of the field:
    motionModeParams[0] |= tracking ? 0x1 : 0;
    motionModeParams[0] |= tracking ? (fast ? 0x2 : 0) : (fast ? 0 : 0x2);
    motionModeParams[0] |= medium ? 0x4 : 0;
    motionModeParams[0] |= slow_goto ? 0x8 : 0;
    motionModeParams[1] |= south ? 0x1 : 0;
    motionModeParams[1] |= ccw ? 0x2 : 0;
    motionModeParams[1] |= goto_coarse ? 0x4 : 0;

    eqPort.print(":G");
    eqPort.print(axis);
    eqPort.print(motionModeParams[0]); // Send value converted to ASCII (byte 1)
    eqPort.print(motionModeParams[1]); // Send value converted to ASCII (byte 2)
    eqPort.print('\r');

    bool result = waitForResponse();
    cleanAfter();
    return result;
}

// ***************************
// PROPERTIES
// ***************************

/**
 * Returns firmware version of the motor controller inside the mount
 */
long EQMount_real::getFirmwareVersion(){
    return firmwareVersion;
}

