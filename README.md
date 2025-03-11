# 🚀 TragerX - Autonomous Navigation Trolley



TragerX is an **ROS-based autonomous navigation system** designed for Raspberry Pi with Ubuntu Server. Using **SLAM, A* Pathfinding, and HC-SR04 Ultrasonic Sensors**, the robot explores unknown environments and navigates efficiently from **Point A to Point B** while avoiding obstacles. 

## 🔥 Key Features

### 🌍 **Simultaneous Localization and Mapping (SLAM)**
- Uses **HC-SR04 Ultrasonic Sensors** to dynamically build an **occupancy grid map**.
- Continuously updates the map based on detected obstacles.

### 🏎️ **Motor Control & Path Execution**
- **2 DC motors** (Rear-Wheel Drive) for precise movement.
- **ROS commands for Forward, Left, and Right turns** based on planned paths.
- Dynamic speed adjustments based on surroundings.

### 🛤️ **A* Pathfinding for Smart Navigation**
- Implements **A* Algorithm** for finding the shortest path in real-time.
- Avoids obstacles using a dynamic **cost-based traversal system**.

### 🎯 **Real-time User-defined Destination Input**
- Allows users to **input destination coordinates** before execution.
- Robot autonomously calculates the best path and navigates accordingly.

### 🔗 **ROS Node Architecture**
- **SLAM Node**: Builds and updates the occupancy grid.
- **Robot Node**: Controls DC motors and handles movement.
- **Pathfinding Node**: Runs A* Algorithm for route planning.
- **Main Node**: Orchestrates navigation and receives user input.

## 📌 Usage - Airport Trolley Navigation
TragerX can be applied in **airport trolleys** to assist travelers by autonomously navigating to predefined locations such as:
- 🛄 **Check-in Counters** - Move the trolley to the airline counter seamlessly.
- 🎟 **Boarding Gates** - Assist passengers in reaching their boarding gates efficiently.
- 🚪 **Exit & Baggage Claim** - Navigate from the gate to baggage claim and airport exit.

### 🚶 How It Works:
1. The user **enters a destination** (e.g., Check-in Counter 12).
2. The trolley **scans the surroundings** and builds a map using SLAM.
3. The **A* pathfinding algorithm calculates the optimal route**.
4. The trolley **autonomously navigates** while avoiding obstacles.
5. **Dynamic rerouting occurs** if new obstacles are detected.

## 👨‍💻 Contributors
- **Sree Sai Raghav**
- **Shakthevell**
- **U Pranov Shanker**
- **Sibin Paulraj** 

We welcome contributions! Feel free to fork, create issues, and submit pull requests. 🚀

## 📜 License
This project is licensed under the **MIT License**. See the [LICENSE](LICENSE) file for more details.

### 🎥 Demo Video
[![Watch the Video](https://drive.google.com/file/d/1kYC7qZnJugHmmbDcFfw0_Zb0wkv_mJqY/view?usp=sharing)

## 🌟 Support
If you find this project useful, consider **starring the repo ⭐** and sharing it with others!


