#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include "eqMount_real.hpp"
#include "eqMount_virtual.hpp"
#include "mount_ctrl/status.h"
#include "mount_ctrl/cmd.h"
#include "mount_ctrl/verbose.h"
#include "sigMeter.hpp"
#include "sigMeter_virtual.hpp"
#include <memory>
#include <queue>
#include <sstream>
#include <exception>

#define CMD_DELIMITER ' '
#define CMD_GOTO_HR_POS 1
#define CMD_GOTO_DEC_POS 2

#define VERB_GOTO_START 1
#define VERB_GOTO_DONE 2
#define VERB_GOTO_ERR_NO_PARAMS 3
#define VERB_GOTO_ERR_INVALID_PARAMS 4

#define VERB_STOP_OK 1

#define VERB_HOMING_START 1
#define VERB_HOMING_DONE 2

#define ROS_STATUS_TOPIC "mountStatus"
#define ROS_VERBOSE_TOPIC "mountVerbose"
#define ROS_COMMAND_TOPIC "mountCmd"

// ROS publisher and subscribers
ros::Publisher statusPublisher;
ros::Publisher verbosePublisher;
ros::Subscriber cmdSubscriber;
std::queue<std::string> cmdQueue;

/**
 * @brief Status of the telescope
 * 
 */
enum Status{
    DISCONNECTED = 0,
    CONNECTED = 1,
    GOTO = 2,
    MOVING = 3,
    TRACKING = 4,
    SCANNING = 5,
    HOMING = 6,
    STOP = 7,
};

// Last known status of the telescope
Status lastStatus;

/**
 * @brief Publish current status and other properties of the telescope
 * 
 * @param hr Hour angle
 * @param dec Declination
 * @param singnalStrength Signal strength currently measured by the telescope
 * @param status Status
 */
void publishStatus(double hr, double dec, int16_t singalStrength, int8_t status){
    mount_ctrl::status stat;
    stat.status = status;
    stat.scanProgress = 0;
    stat.hr = hr;
    stat.dec = dec;
    statusPublisher.publish(stat);
}

void receiveCommand(mount_ctrl::cmd cmd){
    cmdQueue.push(cmd.text); // Push cmd's text into the command queue to be processed by main thread
}

void publishVerbose(int16_t code, int16_t subcode, std::string msg, bool error){
    mount_ctrl::verbose verbose;
    verbose.code = code;
    verbose.subCode = subcode;
    verbose.msg = msg;
    verbose.isError = error;
    verbosePublisher.publish(verbose);
}



int main(int argc, char** argv){
    // Init ros
    ros::init(argc, argv, "mount_ctrl");
    ros::NodeHandle nodeHandle;
    statusPublisher = nodeHandle.advertise<mount_ctrl::status>(ROS_STATUS_TOPIC, 100);
    verbosePublisher = nodeHandle.advertise<mount_ctrl::verbose>(ROS_VERBOSE_TOPIC, 100);
    cmdSubscriber = nodeHandle.subscribe<mount_ctrl::cmd>(ROS_COMMAND_TOPIC, 10, receiveCommand);
    ros::Rate looper(2); // Frequency of looping
    
    ROS_INFO("mount_ctrl initialized");

    // Init EQMount
    std::unique_ptr<EQMount> mount = std::make_unique<EQMount_virtual>();
    mount->begin();
    lastStatus = Status::CONNECTED;

    // Init sigMeter
    std::unique_ptr<SigMeter> sigMeter = std::make_unique<SigMeter_virtual>(mount.get());
    sigMeter->begin();

    while(ros::ok()){
        // Measure current signal strength
        int16_t sigStrength = sigMeter->measure();
        
        Position currentPos = mount->getPosition();
        if(cmdQueue.size() > 0){
            // Get next command from cmdQueue
            std::string cmd = cmdQueue.front();
            cmdQueue.pop();
            // Split command into arguments
            std::stringstream cmdStream(cmd);
            std::vector<std::string> cmdParams;
            std::string token;
            while(std::getline(cmdStream, token, CMD_DELIMITER)){
                cmdParams.push_back(token);
            }
            if(cmdParams.at(0) == "goto"){
                // Check wherether parameters are OK
                double hr, dec;
                ROS_INFO("RECEIVED GOTO CMD");
                try
                {
                    hr = std::stod(cmdParams.at(CMD_GOTO_HR_POS));
                    dec = std::stod(cmdParams.at(CMD_GOTO_DEC_POS));
                    if(hr > 360 || hr < 0 || dec > 90 || dec < -90)
                        throw std::invalid_argument("Selected coordiantes are out of range");
                }
                catch(const std::invalid_argument& e)
                {
                    ROS_ERROR("Received invalid GOTO command - invalid arguments");
                    publishVerbose(Status::GOTO, VERB_GOTO_ERR_INVALID_PARAMS, "Goto command failed - invalid parameters", true);
                    continue;
                }
                catch(const std::out_of_range& e){
                    ROS_ERROR("Received invalid GOTO command - not enough arguments");
                    publishVerbose(Status::GOTO, VERB_GOTO_ERR_NO_PARAMS, "Goto command failed - not enough parameters", true);
                    continue;
                }

                lastStatus = Status::GOTO;
                // Get position
                Position gotoPos(hr, dec);
                // Command mount to slew to target position
                mount->target_goto(gotoPos);
                publishVerbose(Status::GOTO, VERB_GOTO_START, "Goto started", false);
            }
            else if(cmdParams.at(0) == "stop"){
                ROS_INFO("RECEIVED STOP CMD");
                lastStatus = Status::STOP;
                mount->stop();
                publishVerbose(Status::STOP, VERB_STOP_OK, "Mount stopped", false);
            }
            else if(cmdParams.at(0) == "home"){
                ROS_INFO("RECEIVED HOME CMD");
                lastStatus = Status::HOMING;
                Position homePos;
                homePos.dec = 90;
                homePos.hr = 0;
                mount->target_goto(homePos);
                publishVerbose(Status::HOMING, VERB_HOMING_START, "Homing started", false);
            }
        }
        
        // Check if status hasn't changed
        if(lastStatus & (Status::GOTO | Status::HOMING)){
            if(mount->getGotoFinished()){
                lastStatus = Status::STOP;
                if(lastStatus & Status::GOTO)
                    publishVerbose(Status::GOTO, VERB_GOTO_DONE, "GOTO finished", false);
                else
                    publishVerbose(Status::HOMING, VERB_HOMING_DONE, "Homing finished", false);
            }
        }

        // Generate status output
        mount_ctrl::status stat;
        stat.status = lastStatus;
        stat.hr = currentPos.hr;
        stat.dec = currentPos.dec;
        stat.signal = sigStrength;
        if(lastStatus == Status::SCANNING){
            std::cerr << "Scanning was triggered, but is not implemented yet" << std::endl;
            throw -1; // Not implemented
        }
        else
            stat.scanProgress = 0;
        statusPublisher.publish(stat);
        ros::spinOnce();

        looper.sleep();
    }

}
