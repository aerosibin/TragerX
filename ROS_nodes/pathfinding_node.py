import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray, String

class AStar:
    def __init__(self):
        rospy.init_node('pathfinding_node', anonymous=True)
        rospy.Subscriber('/slam_map', Int32MultiArray, self.map_callback)
        self.path_pub = rospy.Publisher('/navigation_path', Int32MultiArray, queue_size=10)
        self.grid = np.zeros((50, 50), dtype=int)
    
    def map_callback(self, data):
        self.grid = np.array(data.data).reshape(50, 50)
        self.find_path((0, 0), (49, 49))
    
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def find_path(self, start, goal):
        open_list = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        while open_list:
            _, current = open_list.pop(0)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                self.publish_path(path)
                return
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < 50 and 0 <= neighbor[1] < 50 and self.grid[neighbor] == 0:
                    tentative_g = g_score[current] + 1
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                        open_list.append((f_score[neighbor], neighbor))
    
    def publish_path(self, path):
        path_msg = Int32MultiArray()
        path_msg.data = [coord for point in path for coord in point]
        self.path_pub.publish(path_msg)

if __name__ == '__main__':
    AStar()
    rospy.spin()