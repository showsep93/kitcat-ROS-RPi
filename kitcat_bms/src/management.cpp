/**
 *
   \file
   \brief      Node for managing the Kit-Cat's wireless charging station
   \author     Josep Clotet Ginovart <josep_c_g@hotmail.com>
*/


#include <ros/ros.h>
#include <signal.h>
#include <kitcat_bms/esc.h> // Custom ROS message type for the ESC
#include <sensor_msgs/BatteryState.h>


void mySigintHandler(int sig) {
    // Always turn off the ESC before shutting down
    // TODO: turn off the ESC by GPIO

    // Do also what the default sigint handler does
    ros::shutdown();
}

int main (int argc, char** argv) {
    kitcat_bms::esc msg_esc;

    ros::init (argc, argv, "charging_station");
    ros::NodeHandle nh;

    // Override the default ros sigint handler.
    signal(SIGINT, mySigintHandler);


	ros::spin();
    return 0;
}