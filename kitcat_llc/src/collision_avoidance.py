#!/usr/bin/python

import math, time, rospy

from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

DIST_ACTUATION      = 0.6 # Meters
DIST_BRAKE          = DIST_ACTUATION / 3
MIN_SENSOR_RANGE    = 0.2 + 0.001 # For float precision

# Subscribes to topic "/kitkat/ultrasonic" and calculates a correction to Kit-Cat's movement in order to avoid a collision. Corrections are published to "/control/cmd_vel":
class CollisionAvoidance():
    def __init__(self):
        self.detected_range = 3 # Meters, default value 
        
        # SUBSCRIPTION
        self.sub_range = rospy.Subscriber("/kitcat/ultrasonic", Range, self.update_range)
        rospy.loginfo("Subscriber set")
        
        # PUBLICATION
        self.pub_twist = rospy.Publisher("/control/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")
        
        # MESSAGE TYPE
        self._message = Twist()
        
        self._time_steer        = 0
        self._steer_sign_prev   = 0


    # CALLBACK
    def update_range(self, message):        
        self.detected_range = message.range


    # Method that calculates the correction to apply to the vehicle
    def obtain_control_action(self):      
        correction = 1.0     

        # Apply total/progressive braking
        if self.detected_range < DIST_BRAKE or self.detected_range <= MIN_SENSOR_RANGE:
            correction = 0.0
        elif self.detected_range < DIST_ACTUATION:
            correction = self.detected_range / DIST_ACTUATION
            correction = saturate(correction, 0.0, 1.0)

        if correction != 1.0:
            rospy.loginfo("Correction in linear speed: %.2f" %correction)

        return correction


    # MAIN
    def run(self):
        rate = rospy.Rate(5) # Runs faster than others

        while not rospy.is_shutdown():
            # Obtain the correction
            correction = self.obtain_control_action()

            # Update the message
            self._message.linear.x  = correction

            # Publish
            self.pub_twist.publish(self._message)

            rate.sleep()        


# Auxiliary function to get a value back in range
def saturate(value, min, max):
    if value <= min:
        return(min)
    elif value >= max:
        return(max)
    else:
        return(value)


if __name__ == "__main__":
    rospy.init_node('avoid_collision')
    
    avoid_collision = CollisionAvoidance()
    avoid_collision.run()            