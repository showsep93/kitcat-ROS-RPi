#!/usr/bin/python

import math, rospy

from hc_sr04_ultrasonic import HC_SR04

from sensor_msgs.msg import Range # native message type from ROS
"""
sensor_msgs/Range

uint8 ULTRASOUND=0
uint8 INFRARED=1
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint8 radiation_type
float32 field_of_view
float32 min_range
float32 max_range
float32 range
"""

# Sensor GPIO numbers
GPIO_TRIGGER    = 4
GPIO_ECHO       = 17


# Scans the Kit-Cat's ultrasonic distance sensor in ROS
class ScanUltrasonic():
    # CONSTRUCTOR
    def __init__(self,
                 gpio_trigger,
                 gpio_echo,
                 range_min,          # In meters
                 range_max,          # In meters
                 effectual_angle_deg # +/- effectual angle from the longitudinal angle position at which the sensor is positioned
                ):

        self.delta_angle_deg = 2*effectual_angle_deg # Best car's field of vision when the sensor is positioned along the longitudinal axis

        rospy.loginfo("Initializing ultrasonic distance sensor")

        self.sensor = HC_SR04(gpio_trigger, gpio_echo, range_min=range_min*100, range_max=range_max*100)
        rospy.loginfo("Frontal HC_SR04 is set")

        # PUBLICATION
        # Create the publisher to topic "/kitcat/ultrasonic/" of message type Range
        self.ros_pub_ultrasonic = rospy.Publisher("/kitcat/ultrasonic/", Range, queue_size=5)
        rospy.loginfo("Publisher correctly initialized to topic \"/kitcat/ultrasonic\"")     
        
        # MESSAGE TYPE 
        self._sensor_msg = Range()
        self._sensor_msg.radiation_type = 0
        self._sensor_msg.field_of_view  = 0
        self._sensor_msg.min_range      = range_min
        self._sensor_msg.max_range      = range_max


    # PUBLISHER
    # Method that updates the message with the ultrasonic sensor values and publishes them
    def scan(self):
        distance_cm = self.sensor.get_distance()
        self._sensor_msg.range = distance_cm * 0.01

        self.ros_pub_ultrasonic.publish(self._sensor_msg)

        rospy.loginfo("Distance to collision [cm]: center = %4.2f" %distance_cm)
    

    # MAIN
    def run(self):
        rate = rospy.Rate(10)
        
        rospy.loginfo("Running ultrasonic sensor...")
        while not rospy.is_shutdown():
            self.scan()
            rate.sleep()
        
        self.sensor.cleanGPIOs()
        rospy.loginfo("Stopped ultrasonic sensor")



if __name__ == "__main__":
    rospy.init_node('ultrasonic_sensor')
    rospy.loginfo("Setting up the \"ultrasonic_sensor\" node...")

    ultrasonic_sensor = ScanUltrasonic(gpio_trigger=GPIO_TRIGGER, gpio_echo=GPIO_ECHO, range_min=0.2, range_max=3.5, effectual_angle_deg=15)
    rospy.loginfo("Initialization for \"ultrasonic_sensor\" complete!")
    ultrasonic_sensor.run()        

