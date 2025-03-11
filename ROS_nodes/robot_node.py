import rospy
from std_msgs.msg import Int32MultiArray, String
import RPi.GPIO as GPIO
import time

# Motor GPIO Pins
MOTOR_LEFT = (17, 18)
MOTOR_RIGHT = (22, 23)

# HC-SR04 Sensors
TRIG = [5, 6, 13]
ECHO = [19, 26, 21]

GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LEFT + MOTOR_RIGHT, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

class Robot:
    def __init__(self):
        rospy.init_node('robot_node', anonymous=True)
        self.sonar_pub = rospy.Publisher('/ultrasonic_data', Int32MultiArray, queue_size=10)
        rospy.Subscriber('/robot_commands', String, self.command_callback)
        self.motor_control = { 'FORWARD': self.forward, 'STOP': self.stop, 'LEFT': self.turn_left, 'RIGHT': self.turn_right }
    
    def measure_distance(self):
        distances = []
        for i in range(3):
            GPIO.output(TRIG[i], True)
            time.sleep(0.00001)
            GPIO.output(TRIG[i], False)
            while GPIO.input(ECHO[i]) == 0:
                start_time = time.time()
            while GPIO.input(ECHO[i]) == 1:
                end_time = time.time()
            distance = (end_time - start_time) * 17150
            distances.append(int(distance))
        return distances
    
    def publish_sonar_data(self):
        data = Int32MultiArray()
        data.data = self.measure_distance()
        self.sonar_pub.publish(data)
    
    def command_callback(self, msg):
        command = msg.data
        if command in self.motor_control:
            self.motor_control[command]()
    
    def forward(self):
        GPIO.output(MOTOR_LEFT, (1, 0))
        GPIO.output(MOTOR_RIGHT, (1, 0))
    
    def stop(self):
        GPIO.output(MOTOR_LEFT + MOTOR_RIGHT, (0, 0))
    
    def turn_left(self):
        GPIO.output(MOTOR_LEFT, (0, 1))
        GPIO.output(MOTOR_RIGHT, (1, 0))
        time.sleep(0.5)
        self.stop()
    
    def turn_right(self):
        GPIO.output(MOTOR_LEFT, (1, 0))
        GPIO.output(MOTOR_RIGHT, (0, 1))
        time.sleep(0.5)
        self.stop()
        
if __name__ == '__main__':
    robot = Robot()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        robot.publish_sonar_data()
        rate.sleep()