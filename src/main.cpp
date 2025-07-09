#include <ros/ros.h>
#include "navigationStack.cpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <signal.h>


navigation_stack::NavigationStack* ns;

void signalHandler(int signal) {
    delete ns;
    ros::shutdown();
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "navigation_stack");
    ros::NodeHandle nh;

    navigation_stack::NavigationStack tmp(nh);
    ns = &tmp;
    
    signal(SIGINT, signalHandler);
    ns->run();


    return 0;
}

