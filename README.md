# Franka Panda Color Sorting Robot

A **ROS 2-based color sorting system** using the **Franka Emika Panda robotic arm**, combining **OpenCV for vision**, **MoveIt 2 for motion planning**, and **Gazebo for simulation**.
This project allows the Panda robot to detect colored objects (Red, Green, or Blue) and perform **pick-and-place** actions automatically.

Run the system:

   ```bash
   ros2 launch panda_bringup pick_and_place.launch.py
   ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
   ```

---

## 📂 Package Summary

| Package                 | Description                           |
| ----------------------- | ------------------------------------- |
| **panda_description**   | URDF, meshes, and robot model         |
| **panda_controller**    | Controller configs and test nodes     |
| **panda_moveit**        | MoveIt 2 motion planning setup        |
| **panda_vision**        | OpenCV-based color detection          |
| **panda_bringup**       | Launch files to start the full system |
| **pymoveit2**           | PyMoveIt2 pick-and-place control node |

---

## 🧠 Key Technologies

| Component        | Purpose                                 |
| ---------------- | --------------------------------------- |
| **ROS 2 Humble** | Robot middleware and node communication |
| **Gazebo**       | Robot simulation                        |
| **RViz**         | Visualization of robot and motion plans |
| **MoveIt 2**     | Motion planning and control             |
| **PyMoveIt2**    | Python interface for MoveIt 2           |
| **OpenCV**       | Real-time color detection               |


---
