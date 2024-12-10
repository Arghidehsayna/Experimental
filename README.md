# Experimental


**Introduction**  
This package is for Assignment 1 of the Experimental Robotics Lab, simulating a robot navigating using ArUco markers in a virtual environment.

---

**Package Overview**  
- **src**: Scripts for marker detection and navigation (`MarkerDetection.py`, `MarkerNavigation.py`, `Mymoviet.cpp`, `My_moveit2.cpp`)  
- **launch**: Launch file for Gazebo and RViz (`robot4_gazebo.launch`)  
- **motors_config**: Robot motor configuration files  
- **worlds**: Gazebo world file (`aruco.world`)  
- **urdf**: Robot model file (`Myrobot4.xacro`)  
- **CMakeLists.txt**: Build configuration  
- **Myrobot4.gazebo**: Gazebo configuration  

---

**Getting Started**  
1. **Build the package**:  
   ```bash
   catkin_make
   ```

2. **Source the workspace**:  
   ```bash
   source devel/setup.bash
   ```

3. **Launch Gazebo and RViz**:  
   ```bash
   roslaunch robot_urdf robot4_gazebo.launch
   ```

4. **Start RViz visualization**:  
   ```bash
   rosrun rviz rviz -d /path/to/robot_urdf/motors_config/rvizconfig.rviz
   ```

---

**Running Nodes Separately**  
- **Launch Gazebo**:  
   ```bash
   roslaunch robot_urdf robot4_gazebo.launch
   ```

- **Launch RViz**:  
   ```bash
   rosrun rviz rviz -d /path/to/robot_urdf/motors_config/rvizconfig.rviz
   ```

- **Run Marker Navigation**:  
   ```bash
   rosrun robot_urdf MarkerNavigation.py
   ```

