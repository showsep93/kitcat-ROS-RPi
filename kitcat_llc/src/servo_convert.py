#!/usr/bin/python


# Servo handling for ROS i2cpwm_board
class ServoConvert():
    # Constructor function
    def __init__(self, id, center_value, range, direction=1):
        # Public instance attributes
        self.id         = id
        self.value      = 0.0
        self.value_out  = center_value

        # Protected instance attributes
        self._center        = center_value
        self._range         = range
        self._half_range    = 0.5*range
        self._dir           = direction
        self._step          = 1.0/self._half_range # smallest step in range [-1, 1]


    # Method to obtain the actual servo PWM given a value in range [-1 ,1]
    def getPWM(self, value_in):
        self.value      = value_in
        self.value_out  = int(self._dir * value_in * self._half_range + self._center)

        print "Servo Channel "+str(self.id-1), "PWM: "+str(self.value_out)

        return(self.value_out)

