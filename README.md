# ROS-Robot-Operating-System-
ROS2 OpenCV vision package for real-time color and QR code detection using a webcam. The system publishes detected color labels and decoded QR data as ROS topics for robot integration. Built with rclpy, OpenCV, and HSV segmentation. Useful for autonomous navigation, object sorting, smart robots, and perception learning projects and labs. demo test



## 📌 Project Overview

This project integrates **OpenCV computer vision with ROS2 (Robot Operating System)** to perform real-time webcam-based perception. It detects colors and QR codes and publishes the extracted information as ROS topics. The system enables robots and automation platforms to receive vision-based data for navigation, sorting, and decision-making tasks.

---

## 🎯 Project Objectives

The main objective of this project is to bridge computer vision and robotic middleware communication. It allows visual perception results to be transferred between nodes using ROS topics.

**Goals include:**

* Real-time camera-based perception
* HSV color detection implementation
* QR code scanning and decoding
* ROS2 publisher node integration
* Vision-to-robot data communication

---

## 📂 Modules Included

### 🎨 Color Detection Publisher Node

* Detects RED, GREEN, and BLUE objects using HSV segmentation
* Publishes detected color names to the `detected_color` ROS topic
* Uses contour filtering to reduce noise
* Runs in real time using ROS timers

---

### 📱 QR Code Publisher Node

* Scans QR codes from live webcam feed
* Extracts encoded data automatically
* Publishes decoded QR text to the `qr_data` ROS topic
* Displays camera output for monitoring

---

## ⚙️ Technologies Used

* Python 3
* ROS2 (rclpy)
* OpenCV
* NumPy
* Webcam input

---

## 📦 Installation

Install required dependencies:

```bash
pip install opencv-python numpy
```

Make sure ROS2 is installed and sourced properly.

---

## ▶️ How To Run

### Start ROS2 workspace

```bash
source /opt/ros/humble/setup.bash
```

### Run Color Detection Node

```bash
python color_pub.py
```

### Run QR Detection Node

```bash
python qr_pub.py
```

---

## 📡 Published Topics

**Color Node:**

* Topic: `detected_color`
* Message Type: `std_msgs/String`

**QR Node:**

* Topic: `qr_data`
* Message Type: `std_msgs/String`

---


✅ LinkedIn project description

Just say 👍
