import rospy
from std_msgs.msg import Int32MultiArray, String

class MainNode:
    def __init__(self):
        rospy.init_node('main_node', anonymous=True)
        self.cmd_pub = rospy.Publisher('/robot_commands', String, queue_size=10)
        rospy.Subscriber('/navigation_path', Int32MultiArray, self.path_callback)
        
        self.start = (0, 0)
        self.end = self.get_user_input()
        
        rospy.Publisher('/pathfinding_goal', Int32MultiArray, queue_size=10).publish(Int32MultiArray(data=[self.end[0], self.end[1]]))
    
    def get_user_input(self):
        x = int(input("Enter destination X coordinate: "))
        y = int(input("Enter destination Y coordinate: "))
        return (x, y)
    
    def path_callback(self, data):
        path = [(data.data[i], data.data[i+1]) for i in range(0, len(data.data), 2)]
        for point in path:
            if point[0] > self.start[0]:
                self.cmd_pub.publish("RIGHT")
            elif point[0] < self.start[0]:
                self.cmd_pub.publish("LEFT")
            else:
                self.cmd_pub.publish("FORWARD")
            rospy.sleep(1)
        self.cmd_pub.publish("STOP")

if __name__ == '__main__':
    MainNode()
    rospy.spin()