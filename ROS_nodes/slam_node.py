import rospy
import numpy as np
from sensor_msgs.msg import Range
from std_msgs.msg import Int32MultiArray

class GridBasedSLAM:
    def __init__(self):
        rospy.init_node('slam_node', anonymous=True)
        self.grid_size = (50, 50)
        self.occupancy_grid = np.zeros(self.grid_size, dtype=int)
        
        rospy.Subscriber('/ultrasonic_data', Int32MultiArray, self.sensor_callback)
        self.grid_pub = rospy.Publisher('/slam_map', Int32MultiArray, queue_size=10)
    
    def sensor_callback(self, data):
        distances = data.data
        # Process sensor readings and update map
        self.update_map(distances)
        self.publish_map()
    
    def update_map(self, distances):
        for i, distance in enumerate(distances):
            if distance < 50:
                self.occupancy_grid[i, distance] = 1  # Mark obstacles
    
    def publish_map(self):
        grid_msg = Int32MultiArray()
        grid_msg.data = self.occupancy_grid.flatten().tolist()
        self.grid_pub.publish(grid_msg)

if __name__ == '__main__':
    slam = GridBasedSLAM()
    rospy.spin()