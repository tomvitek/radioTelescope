#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include "eqMount_real.hpp"
#include "mount_ctrl/position.h"
#include <memory>

int main(int argc, char** argv){
    // Init ros
    ros::init(argc, argv, "mount_ctrl");
    ros::NodeHandle nodeHandle;
    ros::Publisher posPublisher = nodeHandle.advertise<mount_ctrl::position>("mountPos", 100);
    ros::Rate looper(10); // Frequency of looping

    // Init EQMount

    std::unique_ptr<EQMount> mount = std::make_unique<EQMount_real>();
    mount->begin();
    Position testSlewPos;
    testSlewPos.hr = 0.0;
    testSlewPos.dec = -80.0;
    mount->target_goto(testSlewPos);
    ROS_INFO("mount_ctrl initialized");
    Position homePos;
    homePos.dec = 90.0;
    homePos.hr = 0.0;

    while(ros::ok()){
        mount_ctrl::position pos;
        Position rawPos = mount->getPosition();
        pos.dec = rawPos.dec;
        pos.ha = rawPos.hr;

        if(mount->getGotoFinished() && !(rawPos.hr == 0.0 && rawPos.dec == 90.0))
            mount->target_goto(homePos);

        posPublisher.publish(pos);

        ros::spinOnce();

        looper.sleep();
    }

}
