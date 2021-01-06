#!/usr/bin/python

import rospy, time

from i2cpwm_board.msg import Servo, ServoArray
from geometry_msgs.msg import Twist # native message type from ROS

from servo_convert import *


# Low-level control for Kit-Cat servo actuators in ROS. It listens to "/kitcat/ultrasonic/" for collision avoidance.
class KitCatLowLevelCtrl():
    # CONSTRUCTOR
    def __init__(self):
        rospy.loginfo("Setting up \"kitcat_llc\" node...")
        rospy.init_node('kitcat_llc')

        # Create an actuator dictionary
        self.actuators = {} 
        self.actuators['throttle']  = ServoConvert(id=1, center_value=270, range=40)
        self.actuators['steering']  = ServoConvert(id=2, center_value=270, range=140, direction=1) # Positive sign is left
        self.actuators['accessory']  = ServoConvert(id=3, center_value=270, range=1)
        rospy.loginfo("Servo actuators correctly initialized")

        # MESSAGE TYPE
        self._servo_msg = ServoArray() 
        for i in self.actuators:
            self._servo_msg.servos.append(Servo()) # Initialize for each servo channel

        # PUBLICATION
        # Create the publisher to topic "/servos_absolute" of message type ServoArray
        self.ros_pub_servo_array    = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1) # Queue size to 1 so that it publishes every single time
        rospy.loginfo("Publisher to \"/servos_absolute\" correctly initialized")

        # SUBSCRIPTION
        # Create the subscriber to topic "/cmd_vel" with the callback function "set_actuators_from_cmdvel"
        self.ros_sub_twist          = rospy.Subscriber("/cmd_vel", Twist, self.update_from_command)
        rospy.loginfo("Subscriber to \"/cmd_vel\" correctly initialized")

        # Create the subscriber to topic "/control/cmd_vel" with the callback function "update_from_avoidance"
        self.ros_sub_twist          = rospy.Subscriber("/control/cmd_vel", Twist, self.update_from_avoidance)
        rospy.loginfo("Subscriber to \"/control/cmd_vel\" corrrectly initialized") 

        # Set a timeout
        self._last_time_cmd_rcv     = time.time()
        self._last_time_avoid_rcv   = time.time()
        self._timeout_s             = 3

        self.throttle, self.steer = 0.0, 0.0

        self.throttle_cmd, self.steer_cmd = 0.0, 0.0
        self.throttle_avoid = 1.0

        rospy.loginfo("Initialization for \"kitcat_llc\" complete!")


    # PUBLISHER
    # Method that updates the  attribute self._servo_msg with the servo PWM values of dictionary self.actuators and publishes them
    def send_servo_msg(self):
        for name, servo_obj in self.actuators.items():
            currentId = servo_obj.id
            # Note that ServoArray starts at 0 but i2cpwm_board does it at 1
            self._servo_msg.servos[currentId-1].servo = currentId
            self._servo_msg.servos[currentId-1].value = servo_obj.value_out
            rospy.loginfo("Sending to %s command %d"%(name, servo_obj.value_out))

        self.ros_pub_servo_array.publish(self._servo_msg)


    # CALLBACKS
    def update_from_command(self, message):
        # Saves the time the message has been received for the timeout
        self._last_time_cmd_rcv = time.time()
        self.throttle_cmd       = message.linear.x  # Throttle along x axis
        self.steer_cmd          = message.angular.z # Steering or "yaw" 
        
    def update_from_avoidance(self, message):
        # Saves the time the message has been received for the timeout
        self._last_time_avoid_rcv = time.time()
        self.throttle_avoid       = message.linear.x # Braking coefficient


    # Method that combines commands from the user and the ultrasonic sensor to obtains the self.throttle and self.steer attributes; finally it calls the function that will perform the conversion to PWM.
    def compose_movement(self):
        # Only update throttle attribute if the car is receiving the order to move forward, otherwise stick to the user command.
        if self.throttle_cmd > 0:
            self.throttle = saturate(self.throttle_cmd * self.throttle_avoid, 0, 1)
        else:
            self.throttle = saturate(self.throttle_cmd, -1, 0)

        # Update steer attribute
        self.steer = self.steer_cmd
        
        # Call the unitary range to PWM converter
        self.set_actuators_from_cmdvel(self.throttle, self.steer)


    # From throttle and steering and assuming a range [-1, 1], it updates the dictionary self.actuators and calls the publisher
    def set_actuators_from_cmdvel(self, throttle, steering):
        # Converts the Twist message received into servo PWM values
        self.actuators['throttle'].getPWM(throttle)
        self.actuators['steering'].getPWM(steering)
        rospy.loginfo("Received cmd throttle = %3.1f" %throttle)
        rospy.loginfo("Received cmd steering = %3.1f" %steering)

        # Calls the publisher
        self.send_servo_msg()


    # Actuators are set back to idle
    def set_actuators_idle(self):
        self.throttle_cmd   = 0.0
        self.steer_cmd      = 0.0
        rospy.loginfo("Setting actuators to idle")

    # Avoidance values are reset
    def reset_avoidance(self):
        self.throttle_avoid = 1.0


    # Checks timeout
    def is_controller_connected(self):
        interval = time.time() - self._last_time_cmd_rcv
        # print "Interval since last received command: "+str(interval)+ " seconds"
        if interval < self._timeout_s:
            return True
        else:
            return False

    # Checks timeout
    def is_avoidance_connected(self):
        interval = time.time() - self._last_time_avoid_rcv
        # print "Interval since last received avoidance: "+str(interval)+ " seconds"
        if interval < self._timeout_s:
            return True
        else:
            return False


    # MAIN
    def run(self):
        rate = rospy.Rate(5) # Hz

        # When a subscriber is involved, node must go into an infinite loop until it receives a shutdown signal so that the published messages arrive. For ROS Python, rospy.spin() can be replaced by a while loop and regular processing will be done at a certain specified rate.
        while not rospy.is_shutdown():
            self.compose_movement()

            if not self.is_controller_connected():
                self.set_actuators_idle() # In case timeout expires
            if not self.is_avoidance_connected():
                self.reset_avoidance()

            rate.sleep()


# Auxiliary function to get a value back in range
def saturate(value, min, max):
    if value <= min:
        return(min)
    elif value >= max:
        return(max)
    else:
        return(value)


'''
Before executing code, Python interpreter reads source file and define few special variables/global variables. 
If the python interpreter is running that module (the source file) as the main program, it sets the special __name__ variable to have a value "__main__".
If this file is being imported from another module, __name__ will be set to the module's name. Module's name is available as value to __name__ global variable. 
'''
# It ensures that it will execute only if the file is invoked directly but it will not when imported:
if __name__ == "__main__":
    kitcat_llc = KitCatLowLevelCtrl()
    kitcat_llc.run()