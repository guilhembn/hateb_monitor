//
// Created by gbuisan on 11/17/17.
//

#include <csignal>
#include <cstdlib>
#include "../include/HATEBMonitor.h"

HATEBMonitor* hatebMonitor;


void sigintHandler(int sig){
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "node will now shutdown");
    // the default sigint handler, it calls shutdown() on node
    delete hatebMonitor;
    ros::shutdown();
    exit(sig);
}

int main(int argc, char** argv){
    ros::init(argc, argv, NODE_NAME);
    signal(SIGINT, sigintHandler);
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    hatebMonitor = new HATEBMonitor(n, pn);
    ros::spin();
    return 0;
}

