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
#include "charging_station.h" // Custom Class to manage the Kit-Cat's charging station


// === CREATE AN INSTANCE OF THE CLASS CHARGING STATION ===
ChargingStation chargingStation;
// ========================================================


// ROS SIGINT HANDLER
void mySigintHandler(int sig) {
    // Always turn off the ESC before shutting down
    chargingStation.setEsc(false);

    // Do also what the default SIGINT handler does
    std::cout << "Shutting down Charging Station components! Bye!" << std::endl;
    ros::shutdown();
}


// CALLBACK FUNCTION
void changeEscState (const kitcat_bms::esc::ConstPtr &msg) {
    bool escState = msg->state;
    std::cout << "New ESC state: " << escState << std::endl;
    // std::cout << "Hello World!" << std::endl;
    chargingStation.setEsc(escState);
}


int main (int argc, char** argv) {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes remappings directly, but for most command-line programs, passing argc and argv is the easiest way to do it.
     * The third argument to init() is the name of the node.
     * 
     * You must call one of the versions of ros::init() before using any other part of the ROS system.
     */
    ros::init (argc, argv, "charging_station");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last NodeHandle destructed will close down the node.
     */
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler); // Override the default ROS SIGINT handler

    // === SUBSCRIBER ===
    /**
     * The subscribe() call is how you tell ROS that you want to receive messages on a given topic.  This invokes a call to the ROS master node, which keeps a registry of who is publishing and who is subscribing. Messages are passed to a callback function, here called chatterCallback.
     * 
     * subscribe() returns a Subscriber object that you must hold on to until you want to unsubscribe.  When all copies of the Subscriber object go out of scope, this callback will automatically be unsubscribed from this topic.
     * 
     * The second parameter to the subscribe() function is the size of the message queue. If messages are arriving faster than they are being processed, this is the number of messages that will be buffered up before beginning to throw away the oldest ones.
     */
    ros::Subscriber sub = nh.subscribe("/esc", 1, changeEscState);

    // === PUBLISHER ===
    /**
     * The advertise() function is how you tell ROS that you want to publish on a given topic name. This invokes a call to the ROS master node, which keeps a registry of who is publishing and who is subscribing. After this advertise() call is made, the master node will notify anyone who is trying to subscribe to this topic name, and they will in turn negotiate a peer-to-peer connection with this node.
     * 
     * advertise() returns a Publisher object which allows you to publish messages on that topic through a call to publish(). Once all copies of the returned Publisher object are destroyed, the topic will be automatically unadvertised.
     * 
     * The second parameter to advertise() is the size of the message queue used for publishing messages. If messages are published more quickly than we can send them, the number here specifies how many messages to buffer up before throwing some away.
     */
    ros::Publisher pub = nh.advertise<sensor_msgs::BatteryState>("/batteries", 1);
    sensor_msgs::BatteryState kitCatBattteries;
    /*  uint8 POWER_SUPPLY_STATUS_CHARGING=1
        uint8 POWER_SUPPLY_STATUS_NOT_CHARGING=3
        uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5
    */
    kitCatBatteries.capacity = 1800.0; kitCatBatteries.power_supply_technology = 5; kitCatBattteries.power_supply_status = 3;

    // === LOOP ===
    // ros::spin() would not return until the node has been shutdown, either through a call to ros::shutdown() or a Ctrl-C. As this would only allow the subscribers to work, the while ros::ok() plus ros::spinOnce() implementation has been chosen in order to also do some work, publish some messages, etc.
    ros::Rate rate(1); // 1 Hz
    while (ros::ok()) {

        // Read and write GPIO for the battery state
        kitCatBattteries.power_supply_status = chargingStation.areBatteriesCharging();
        pub.publish(kitCatBattteries);

        // ESC GPIO is updated by the callback function when a new ESC state is received from the subscription!

        ros::spinOnce(); // It will call all the callbacks waiting to be called at that point in time.
        rate.sleep();
    }

    

    return 0;
}