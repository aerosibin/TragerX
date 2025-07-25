# 🚀 TragerX - Autonomous Navigation Trolley

TragerX is an **ROS-based autonomous navigation system** designed for Raspberry Pi with Ubuntu Server. Using **SLAM**, **A\* Pathfinding**, and a **360° LIDAR sensor (RPLIDAR)**, the robot explores unknown environments and navigates efficiently from **Point A to Point B** while avoiding obstacles with high precision.

---

## 🔥 Key Features

### 🌍 Simultaneous Localization and Mapping (SLAM)
- Uses a **360° LIDAR sensor** to continuously scan and build an accurate **occupancy grid map**.
- Dynamically updates the map with real-time obstacle data in all directions.

### 🏎️ Motor Control & Path Execution
- **2 DC motors** (Rear-Wheel Drive) for precise and responsive movement.
- Controlled through **ROS commands** for forward motion and turning.
- Adapts movement based on environmental context from LIDAR data.

### 🛤️ A* Pathfinding for Smart Navigation
- Implements the **A\*** algorithm to determine the shortest viable route to the destination.
- Integrates LIDAR-based environment perception for **real-time obstacle avoidance**.

### 🎯 Real-time User-defined Destination Input
- Allows users to input destination coordinates or waypoints.
- Trolley autonomously plans, recalculates, and navigates the route.

### 🔗 ROS Node Architecture
- **SLAM Node**: Builds and updates the occupancy grid using LIDAR scan data.
- **Robot Node**: Drives the motors based on control logic and path planning.
- **Pathfinding Node**: Computes optimal routes using A* and map data.
- **Main Node**: Integrates all modules, accepts user input, and executes navigation.

---

## 📌 Usage - Airport Trolley Navigation

TragerX is ideal for environments like **airports**, where autonomous trolleys can improve passenger convenience by navigating to key locations such as:

- 🛄 **Check-in Counters**
- 🎟 **Boarding Gates**
- 🚪 **Exit & Baggage Claim Areas**

### 🚶 How It Works:
1. The user **enters a destination** (e.g., Check-in Counter 12).
2. The trolley performs **360° environmental scanning** using the LIDAR and builds a live map.
3. The **A\*** algorithm calculates the most efficient and safe route.
4. The trolley **navigates autonomously**, smoothly avoiding dynamic obstacles.
5. **Live rerouting** occurs if the trolley detects changes or new obstructions in its path.

---

## 👨‍💻 Contributors

- **Sree Sai Raghav**  
- **Shakthevell M**  
- **U Pranov Shanker**  
- **Sibin Paulraj**

We welcome contributions! Fork the repository, open issues, or submit pull requests to enhance the project. 🚀

---

## 📜 License

This project is licensed under the **MIT License**. See the [LICENSE](LICENSE) file for more details.

---

## 🎥 Demo Video

[![Watch the Demo Video](https://drive.google.com/uc?id=1zD2VkdZcTOZjkl9Ch24ykw5aXjkH-O67)](https://drive.google.com/file/d/1zD2VkdZcTOZjkl9Ch24ykw5aXjkH-O67/view?usp=sharing)

---

## 🌟 Support

If you find this project helpful or inspiring, consider **starring the repo ⭐** and sharing it with others in the robotics community!
