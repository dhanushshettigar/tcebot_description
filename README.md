# tcebot_description

This package holds the URDF description of the TCEBot robot.

![TCE Robot](https://raw.githubusercontent.com/dhanushshettigar/tcebot_description/refs/heads/main/media/TCEBOT.png)

---

## File Structure

📦 **tcebot_description**  
├── 📂 **urdf/** — URDF files containing the description of the robot  
│   ├── 📄 *robot.urdf.xacro*  
│  
├── 📂 **meshes/** — STL files representing the 3D model of the robot  
│   ├── 📄 *robot_part.stl*  
│  
├── 📂 **launch/** — Launch files for Robot State Publisher and RViz visualization  
│   ├── 📄 *display.launch.py*  
│  
├── 📂 **rviz/** — RViz2 configuration files  
│   ├── 📄 *robot_config.rviz*  


---

## Launch Files

The `launch/` directory contains scripts to facilitate the visualization and simulation of the robot.

- **Robot State Publisher**  
  Publishes the state of the robot model to TF, allowing visualization tools like RViz to represent it properly.

- **RViz Visualization**  
  Provides a pre-configured RViz setup to visualize the robot with its links, joints, and sensors.

- **Gazebo Simulation (if applicable)**  
  Launch files can be extended to include Gazebo simulation for physics-based interaction.

---

## Launching the Robot in RViz

To visualize your robot in **RViz**, use the following command:

```bash
ros2 launch tcebot_description display.launch.py
```

Ensure that you have the necessary dependencies installed and sourced before running the command.

---

## Dependencies

Ensure the following packages are installed before launching the robot:

- **ROS 2** (Humble, Foxy, or your preferred distribution)  
- **robot_state_publisher** – Publishes the robot’s state to TF for visualization  
- **joint_state_publisher_gui** – Provides a GUI to modify joint states  
- **rviz2** – Visualization tool for displaying the robot model  
