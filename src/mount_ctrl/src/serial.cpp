/**
 * @file serial.cpp
 * @author Tomáš Vítek (tomas.vitek.tvt@gmail.com)
 * @brief Implementation of the class "Serial" for UART communication.
 * @version 0.1
 * @date 2020-03-01
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "serial.hpp"

#include <cstdio>
#include <cstring>
#include <iostream>

#include <fcntl.h> // File (and therefore serial) controls
#include <errno.h>
#include <termios.h> // Configuration of a serial port
#include <unistd.h> // function like read(), write(), ...

void Serial::begin(int baudrate, const char* name){
    // Open serial port
    sp = open(name, O_RDWR);

    // Check for errors
    if(sp < 0){
        std::cerr << "Serial port open error (" << errno << "): " << strerror(errno) << std::endl;
        throw errno;
    }

    // Configure serial port
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if(tcgetattr(sp, &tty) != 0){
        std::cerr << "Serial port error - couldn't get termios info (" << errno << "): " << strerror(errno) << std::endl;
        throw errno;
    }

    // Parity bit
    if(parity) tty.c_cflag |= PARENB;
    else tty.c_cflag &= ~PARENB;

    // Stop bits
    if(stopBit) tty.c_cflag |= CSTOPB;
    else tty.c_cflag &= ~CSTOPB;

    // Number of bits per byte
    tty.c_cflag &= ~(CS5 | CS6 | CS7 | CS8);
    switch(numBits){
        case 5:
            tty.c_cflag |= CS5;
            break;
        case 6:
            tty.c_cflag |= CS6;
            break;
        case 7:
            tty.c_cflag |= CS7;
            break;
        case 8:
            tty.c_cflag |= CS8;
            break;
        default:
            tty.c_cflag |= CS8;
            break;
    }

    // Set flow control
    if(flowControl) tty.c_cflag |= CRTSCTS;
    else tty.c_cflag &= ~CRTSCTS;

    // Disble modem-specific signal lines
    tty.c_cflag |= CLOCAL;
    
    // Enable reading
    tty.c_cflag |= CREAD;

    // Basicaly disable all local flags, because they do wierd things noone wants.
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = timeout;
    tty.c_cc[VMIN] = 0;

    switch(baudrate){
        case 9600:
            cfsetispeed(&tty, B9600);
            break;
        
        case 57600:
            cfsetispeed(&tty, B57600);
            break;

        case 115200:
            cfsetispeed(&tty, B115200);
            break;

        case 230400:
            cfsetispeed(&tty, B230400);
            break;

        case 460800:
            cfsetispeed(&tty, B460800);
            break;

        default:
            std::cerr << "Serial port error - specified non-supported baud rate" << std::endl;
            throw -1;
            break;
    }

    // Apply settings
    if(tcsetattr(sp, TCSANOW, &tty) != 0){
        std::cerr << "Serial port error - couldn't apply serial port settings (" << errno << "): " << strerror(errno) << std::endl;
        throw errno;
    }
}

void Serial::setParityBit(bool paritySet){
    parity = paritySet;
}

void Serial::setTwoStopBits(bool stopBits){
    this->stopBit = stopBits;
}

void Serial::setNumberOfBits(uint8_t numBits){
    this->numBits = numBits;
}

void Serial::setFlowControl(bool flowControl){
    this->flowControl = flowControl;
}

void Serial::setTimeout(uint8_t timeout){
    this->timeout = timeout;
}

void Serial::closePort(){
    if(close(sp) != 0){
        std::cerr << "Serial port error - close failed (" << errno << "): " << strerror(errno) << std::endl;
        throw errno;
    }
}

void Serial::print(const char* str){
    size_t data_len = strlen(str);
    
    if(write(sp, str, data_len) < 0){
        std::cerr << "Serila port error - write failed (" << errno << "): " << strerror(errno) << std::endl;
    }
}

void Serial::print(const char ch){
    if(write(sp, &ch, 1) < 0){
        std::cerr << "Serila port error - write failed (" << errno << "): " << strerror(errno) << std::endl;
    }
}

void Serial::print(uint8_t num){
    char str[4];
    sprintf(str, "%hhd", num);
    print(str);
}

void Serial::printHex(uint8_t num){
    char str[3];
    sprintf(str, "%X", num);
    print(str);
}

void Serial::writeData(const uint8_t* data, uint32_t len){
    if(write(sp, data, len) < 0){
        std::cerr << "Serila port error - write failed (" << errno << "): " << strerror(errno) << std::endl;
    }
}

uint32_t Serial::readData(uint8_t* buffer, uint32_t bufferSize){
    return read(sp, buffer, bufferSize);
}

uint32_t Serial::readData(char* buffer, uint32_t bufferSize){
    return read(sp, buffer, bufferSize);
}