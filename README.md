# ROS 2 Humble Warehouse AGV Robot Simulation in Gazebo
### SLAM Mapping, YOLO Object Detection and Nav2 Autonomous Navigation

A 4-wheel AGV (automated guided vehicle / warehouse robot) that builds a 2D map with LiDAR SLAM, recognises objects with a custom-trained YOLO model, and drives itself to goals with Nav2 while avoiding obstacles. Built with ROS 2 Humble, Gazebo Harmonic, slam_toolbox, Nav2, AMCL, PyTorch and Ultralytics YOLO.

---

## 🎬 Watch the Demo Video

[![AGV Warehouse Demo](https://img.youtube.com/vi/bXsn-yhT05Y/maxresdefault.jpg)](https://www.youtube.com/watch?v=bXsn-yhT05Y)

---

## 📸 Project Screenshots

<!-- Side-by-side images using an HTML table -->
<table>
  <tr>
    <td width="33%" align="center">
      <b>1. Teleoperation</b><br>
      <img src="media/Teleoperation.png" alt="Teleoperation" width="100%">
    </td>
    <td width="33%" align="center">
      <b>2. SLAM Mapping</b><br>
      <!-- CHANGE THE SRC BELOW TO YOUR ACTUAL SLAM IMAGE PATH -->
      <img src="media/Teleoperation.png" alt="Mapping" width="100%">
    </td>
    <td width="33%" align="center">
      <b>3. YOLO Object Detection</b><br>
      <img src="media/Screenshot 2026-10-29 142146.png" alt="YOLO Object Detection" width="100%">
    </td>
  </tr>
</table>

---

## ✨ Features
* **Gazebo warehouse simulation:** Custom 20 x 20 m world with shelves, boxes and cylinders, plus a URDF/xacro 4-wheel skid-steer AGV with LiDAR, depth camera and IMU.
* **Teleoperation:** Keyboard control for manual driving.
* **SLAM Mapping:** Using `slam_toolbox` and map saving with `nav2_map_server`.
* **Object Detection:** Custom YOLO model (Box, cylinder, shelf) running live on the simulated camera.
* **Autonomous Navigation:** Nav2 (AMCL localization, costmaps, DWB controller) with goals set from RViz, terminal, or Python.
* **ROS 2 - Gazebo Bridge:** (`ros_gz_bridge`) for clock, odometry, LiDAR, camera, IMU, TF, and joint states.

**Tested on:** Ubuntu 22.04 (including WSL2), ROS 2 Humble, Gazebo Harmonic, NVIDIA GTX 1650, Python 3.10.

---

## 📂 Project Structure
```text
agv_chassis_description/   ROS 2 package: URDF/xacro, Gazebo world, bridge, SLAM + Nav2 + YOLO launch files,
                           scripts/yolo_detector.py (ROS 2 node) and models/best.pt (custom YOLO model)
agv_nav/                   goto_goal.py (send goals), calibrate_rotation.py (odometry check)
tools/                     frame_saver.py (record training images), environment check scripts
maps/                      saved SLAM map (warehouse_map.yaml + .pgm)
media/                      project reports and demo media

```
## 1. Installation
