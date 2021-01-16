#!/usr/bin/python

import RPi.GPIO as GPIO
import time

# ubuntu@kitcat:~/WiringPi$ gpio readall
#  +-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+
#  | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
#  +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
#  |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
#  |   2 |   8 |   SDA.1 | ALT0 | 1 |  3 || 4  |   |      | 5v      |     |     |
#  |   3 |   9 |   SCL.1 | ALT0 | 1 |  5 || 6  |   |      | 0v      |     |     |
#  |   4 |   7 | GPIO. 7 |   IN | 1 |  7 || 8  | 1 | ALT0 | TxD     | 15  | 14  |
#  |     |     |      0v |      |   |  9 || 10 | 1 | ALT0 | RxD     | 16  | 15  |
#  |  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
#  |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
#  |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
#  |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
#  |  10 |  12 |    MOSI |   IN | 0 | 19 || 20 |   |      | 0v      |     |     |
#  |   9 |  13 |    MISO |   IN | 0 | 21 || 22 | 0 | OUT  | GPIO. 6 | 6   | 25  |
#  |  11 |  14 |    SCLK |   IN | 0 | 23 || 24 | 1 | IN   | CE0     | 10  | 8   |
#  |     |     |      0v |      |   | 25 || 26 | 1 | IN   | CE1     | 11  | 7   |
#  |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
#  |   5 |  21 | GPIO.21 |  OUT | 0 | 29 || 30 |   |      | 0v      |     |     |
#  |   6 |  22 | GPIO.22 |   IN | 1 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
#  |  13 |  23 | GPIO.23 |   IN | 1 | 33 || 34 |   |      | 0v      |     |     |
#  |  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
#  |  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
#  |     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
#  +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
#  | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
#  +-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+

# Class for the HC-SR04 ultrasonic distance sensor
class HC_SR04():
    # CONSTRUCTOR
    def __init__(self, gpio_trigger, gpio_echo, range_min=2, range_max=450):        
        self._gpio_trigger  = gpio_trigger
        self._gpio_echo     = gpio_echo
        self._range_min     = range_min
        self._range_max     = range_max
        self._is_reading    = False
        
        # Time taken by the pulse is actually for going and return travels of the ultrasonic signals. Time is taken as Time/2 and therefore Distance = SoundSpeed/2 * Time
        self._sound_speed   = 17150.0 # SoundSpeed/2 in cm/s

        # Timeout parameters
        self._last_time_reading = 0
        self._timeout       = range_max/self._sound_speed * 2

        # Mode referring to the pins by the "Broadcom SOC channel" number (GPIOXX)
        GPIO.setmode(GPIO.BCM)

        # Setting GPIO ports defined to be echo/trigger as input/ouput
        GPIO.setup(gpio_echo, GPIO.IN)
        GPIO.setup(gpio_trigger, GPIO.OUT)

        # Let the sensor settle
        GPIO.output(gpio_trigger, GPIO.LOW)
        time.sleep(1)
        

    # Method that obtains the range following the procedure from the sensor's datasheet
    def get_distance(self):
        # Start a reading
        self._is_reading = True

        # The sensor requires a short 10uS pulse to trigger the module, which will cause the sensor to start the ranging program (8 ultrasound bursts at 40 kHz) in order to obtain an echo response.
        GPIO.output(self._gpio_trigger, GPIO.HIGH); time.sleep(0.00001)
        GPIO.output(self._gpio_trigger, GPIO.LOW)
        
        pulse_start_time = time0 = time.time()
        pulse_end_time = time.time()

        # The sensor sets ECHO to high for the amount of time it takes for the pulse to go and come back, so the code measures the amount of time that the ECHO pin stays high. 
        while GPIO.input(self._gpio_echo)==0:
            pulse_start_time = time.time()
            # Manual timeout for when sensor's ECHO pin is not set to HIGH
            if time.time() - time0 > self._timeout:
                print("Ultrasonic sensor ECHO LOW TIMEOUT")
                self._is_reading = False
                return(-1)

        while GPIO.input(self._gpio_echo)==1:
            pulse_end_time = time.time()
            # Manual timeout for when sensor's ECHO pin is never set back to LOW
            if time.time() - pulse_start_time > self._timeout:
                print("Ultrasonic sensor ECHO HIGH TIMEOUT")
                self._is_reading = False
                return(-1)
        
        # Reading is finished
        self._last_time_reading = time.time()
        self._is_reading = False

        # Obtain distance in centimeters
        pulse_duration = pulse_end_time - pulse_start_time
        distance = pulse_duration * self._sound_speed
        
        # Adjust distance according to sensor range
        if distance > self._range_max:
            distance = self._range_max

        if distance < self._range_min:
            distance = self._range_min

        return(distance)


    # Auxiliar function to finish ROS node properly
    def cleanGPIOs(self):
    	GPIO.cleanup()


    # Defined as property so that the protected attribute can be accessed without getters/setters
    @property
    def is_reading(self):
        return(self._is_reading)



if __name__ == "__main__":
    # ULTRASONIC SENSOR BCM/GPIO PINS
    PIN_TRIGGER = 5
    PIN_ECHO = 6

    ultrasonic_sensor = HC_SR04(PIN_TRIGGER, PIN_ECHO)
    
    while True:
        d = ultrasonic_sensor.get_distance()
        if d>0:
            print "Distance = %4.1f cm"%d

