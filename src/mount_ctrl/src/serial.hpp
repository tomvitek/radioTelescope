/**
 * @file serial.hpp
 * @author Tomáš Vítek (tomas.vitek.tvt@gmail.com)
 * @brief Header for the serial class, used for communication throught UART
 * @version 0.1
 * @date 2020-03-01
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __EQMOUNTCTRL__SERIAL__
#define __EQMOUNTCTRL__SERIAL__

#include <stdint.h>

class Serial{
public:
    /**
     * @brief  Opens the serial port and applies specified settings to it.
     * @note   
     * @param  baudrate: Baud rate of the serial port. Supported baud rates are 9600, 57 600, 115 200, 230 400, 460 800
     * @param  name: Device name of the serial port
     * @retval None
     */
    void begin(int baudrate, const char* name = "/dev/serial0");
    /**
     * @brief  Enable or disable parity bit. Disabled by default.
     * @note   
     * @param  parityBit: Parity bit; true for enable, false for disable
     * @retval None
     */
    void setParityBit(bool parityBit);
    /**
     * @brief  Enable or disable two stop bits. One stop bit by default.
     * @note   
     * @param  stopBit: True for two stop bits, false for one.
     * @retval None
     */
    void setTwoStopBits(bool stopBit);
    /**
     * @brief  Set number of bits per byte. 8 by default
     * @note   
     * @param  numBits: Number of bits per byte
     * @retval None
     */
    void setNumberOfBits(uint8_t numBits);
    /**
     * @brief  Enable or disable flow control. Disabled by default.
     * @note   
     * @param  flowControl: flow control, true for enable, false for disable
     * @retval None
     */
    void setFlowControl(bool flowControl);
    /**
     * @brief  Set timeout for reading
     * @note   
     * @param  deciseconds: Timeout in deciseconds.
     * @retval None
     */
    void setTimeout(uint8_t deciseconds);
    void closePort();
    void print(const char* data);
    void print(const char ch);
    void print(uint8_t num);
    void printHex(uint8_t num);
    void writeData(const uint8_t* data, uint32_t len);
    uint32_t readData(char* buffer, uint32_t bufferSize);
    uint32_t readData(uint8_t* buffer, uint32_t bufferSize);

protected:
    /**
     * @brief  File descriptor of the serial port
     */
    int sp;
    bool parity = false;
    bool stopBit = false;
    uint8_t numBits = 8;
    bool flowControl = false;
    uint8_t timeout = 5;
};

#endif