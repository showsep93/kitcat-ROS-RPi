// charging_sation.cpp
#include "charging_station.h"
#include <iostream>
#include <wiringPi.h>
#include <ros/ros.h>

#define ESC_PIN 23
#define CHARGING_PIN 22

ChargingStation::ChargingStation() {
    /**
     * Initialize the wiringPi library using wiringPiSetupGpio.
     * This function will let us use the BCM pin numbering layout, which is the numbering system you will find in the official Raspberry Pi GPIO documentation.
     */
    std::cout << "Hello World!" << std::endl;
    wiringPiSetupGpio();

    // Initialize the ESC STATE pin as OUTPUT and set it to LOW (OFF)
    pinMode(ESC_PIN, OUTPUT);
    ROS_INFO("ESC STATE pin has been set as OUTPUT");
    digitalWrite(ESC_PIN, LOW);
    ROS_INFO("ESC STATE set to OFF");
    this->ESC = false;

    // Initialize the CHARGING DETECTION pin as INPUT and read its value
    pinMode(CHARGING_PIN, INPUT);
    ROS_INFO("CHARGING DETECTION pin has been set as INPUT");
    this->batteriesCharging = digitalRead(CHARGING_PIN);
}


bool ChargingStation::isEscEnabled() {
    if (this->ESC) {
        ROS_INFO("ESC STATE is ON");
    } else {
        ROS_INFO("ESC STATE is OFF");
    }

    return this->ESC;
}

void ChargingStation::setEsc(bool state) {
    // Toggle state
    if (state != this->ESC) {
        digitalWrite(ESC_PIN, state);
        this->ESC = state;
    }
    
    if (this->ESC) {
        ROS_INFO("ESC STATE set to ON");
    } else {
        ROS_INFO("ESC STATE set to OFF");
    }
}

uint8_t ChargingStation::areBatteriesCharging() {
    this->batteriesCharging = digitalRead(CHARGING_PIN);

    if (this->batteriesCharging) {
        ROS_INFO("Kit-Cat is properly parked in the charging station!");
        return POWER_SUPPLY_STATUS_CHARGING; //CHARGING
    } else {
        ROS_INFO("Kit-Cat is NOT properly parked, it must be driving around...");
        return POWER_SUPPLY_STATUS_NOT_CHARGING; //NOT CHARGING
    }
}
