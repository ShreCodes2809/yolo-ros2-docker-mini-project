# YOLO-ROS2 Docker Mini-Project

This project demonstrates an end-to-end ROS2 pipeline for real-time object detection using the YOLO model inside a Dockerized ROS2 Humble environment. It was developed and tested entirely on macOS using Docker (Linux/AMD64 emulation) and includes:

* YOLO ROS2 node running inside Docker
* Custom ROS2 publisher that streams 10 test images
* Custom ROS2 subscriber that receives YOLO detections
* Full ROS2 topic bridging: Image → YOLO → DetectionArray
* A reproducible setup for robotics beginners and students working on CNN + ROS2 tasks

---

## Project Overview

The goals of this mini-project are:

1. Instantiate a CNN (YOLO) as a ROS2 lifecycle node
2. Publish 10 images in real-time into YOLO’s input topic
3. Subscribe to YOLO detection outputs
4. Validate the complete perception pipeline inside Docker

This confirms understanding of ROS2 publishers/subscribers, node lifecycles, message flow, and ML model integration.

---

## Architecture

```
+-------------------+        publishes Image msgs        +-------------------------+
|  Custom Publisher |  --------------------------------> |     YOLO ROS2 Node      |
| (10 test images)  |   (/camera/rgb/image_raw)          |  yolov8.launch.py        |
+-------------------+                                     |  runs CNN inference     |
                                                         |  publishes DetectionArray
                                                         +-----------+-------------+
                                                                     |
                                                                     v
                                                      +--------------+--------------+
                                                      |    Custom Detection Node     |
                                                      |    (/yolo/detections)       |
                                                      +-----------------------------+
```

---

## ✅ Features

* Fully containerized ROS2 + YOLO setup
* Real-time image publishing at 5 Hz
* Detection subscriber printing:

  * class ID
  * class name
  * score
  * 2D bounding box (center + size)
* Works on macOS, even with Docker/apt networking issues
* Clean reproducible workspace structure

---

## 📂 Repository Structure

```
yolo-ros2-docker-mini-project/
│
├── Dockerfile
├── ros2_ws/
│   ├── src/                # YOLO ROS2 packages
│   ├── install/
│   ├── build/
│   └── images/             # 10 test images for publisher
│
├── scripts/
│   └── publisher_10_images.py
│
├── README.md
└── .gitignore
```

---

# Docker & ROS2 Setup (Chronological Steps Added Above)

Below are the original build instructions. The detailed chronological steps have been inserted earlier in this README.

# 1. Build the Docker Imagecal Steps)

1. **Test container build environment (fix Hash Sum mismatch on macOS):**

```
docker run --rm -it \
  --platform=linux/amd64 \
  ros:humble \
  bash -lc '\
    sed -i "s|http://archive.ubuntu.com/ubuntu/|https://archive.ubuntu.com/ubuntu/|g" /etc/apt/sources.list && \
    echo '\''Acquire::http::Pipeline-Depth "0";'\'' > /etc/apt/apt.conf.d/99fix-hash-sum-mismatch && \
    echo '\''Acquire::http::No-Cache "true";'\'' >> /etc/apt/apt.conf.d/99fix-hash-sum-mismatch && \
    echo '\''Acquire::BrokenProxy "true";'\'' >> /etc/apt/apt.conf.d/99fix-hash-sum-mismatch && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    apt-get update && \
    apt-get install -y ros-humble-cv-bridge \
  '
```

2. **Update the Dockerfile with macOS-safe apt settings and rosdep logic.**
   (Your existing Dockerfile already includes these.)

3. **Build the Docker image:**

```
docker build --platform=linux/amd64 -t yolo_ros .
```

4. **Run the container (Terminal T1):**

```
docker run --rm -it \
  --platform=linux/amd64 \
  --name yolo_ros \
  yolo_ros
```

5. **Source ROS and workspace inside T1:**

```
source /opt/ros/humble/setup.bash || true
source /root/ros2_ws/install/setup.bash || true
```

---

## Running the Publisher Setup (Chronological)

1. **Launch YOLO in T1:**

```
ros2 launch yolo_bringup yolov8.launch.py
```

2. **Open Terminal T2 into the same container:**

```
docker exec -it yolo_ros bash
```

3. **Source environment:**

```
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

4. **Check available topics:**

```
ros2 topic list
```

5. **Install tools if needed:**

```
apt-get install -y ros-humble-cv-bridge python3-opencv
apt-get update && apt-get install -y nano
```

6. **Create publisher script:**

```
mkdir -p /root/ros2_ws/scripts
cd /root/ros2_ws/scripts
nano publisher_10_images.py
```

(Paste publisher code here.)

7. **Make it executable:**

```
chmod +x publisher_10_images.py
```

8. **Run publisher:**

```
python3 /root/ros2_ws/scripts/publisher_10_images.py
```

---

## Running the Subscriber + Validation Steps (Chronological)

1. **Copy 10 images from host → container (Terminal T3):**

```
docker cp ~/yolo_ros_images yolo_ros:/root/ros2_ws/images
```

2. **Open T3 bash:**

```
docker exec -it yolo_ros bash
```

3. **Source environment:**

```
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

4. **Verify YOLO node info:**

```
ros2 node info /yolo/yolo_node
```

(Expected subscriber → `/camera/rgb/image_raw`.)

5. **Ensure publisher (T2) is publishing:**

```
Published: img1.jpg
Published: img2.jpg
...
```

6. **Check the input image topic:**

```
ros2 topic info /camera/rgb/image_raw
```

7. **Verify images are actually being published:**

```
ros2 topic echo /camera/rgb/image_raw
```

8. **Check YOLO detection topic:**

````
ros2 top

On macOS (M1/M2/M3/M4):

```bash
docker build --platform=linux/amd64 -t yolo_ros2_project .
````

---

# 2. Run the Container

```bash
docker run --rm -it \
  --platform=linux/amd64 \
  --name yolo_ros \
  yolo_ros2_project
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

---

# 3. Run YOLO

```bash
ros2 launch yolo_bringup yolov8.launch.py
```

YOLO subscribes to:

```
/camera/rgb/image_raw
```

Leave this running.

---

# 4. Run the Image Publisher

In a second terminal:

```bash
docker exec -it yolo_ros bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
python3 /root/scripts/publisher_10_images.py
```

You should see:

```
Published: img1.jpg
Published: img2.jpg
...
```

---

# 5. Run the Detection Subscriber

Open a third terminal:

```bash
docker exec -it yolo_ros bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
python3 /root/scripts/detection_subscriber.py
```

Expected output:

```
Received 3 detections
  [0] class_id=0 class_name='person' score=0.87
  [1] class_id=56 class_name='chair' score=0.91
```

---

# 🔧 Important ROS2 Topics

| Purpose             | Topic                   | Message Type                   |
| ------------------- | ----------------------- | ------------------------------ |
| Image input to YOLO | `/camera/rgb/image_raw` | `sensor_msgs/msg/Image`        |
| YOLO detections     | `/yolo/detections`      | `yolo_msgs/msg/DetectionArray` |
| YOLO debug image    | `/yolo/dbg_image`       | `sensor_msgs/msg/Image`        |

---

# 🛠️ Troubleshooting

### No detections?

Run:

```bash
ros2 topic echo /camera/rgb/image_raw
```

If empty → publisher isn’t working.

Check YOLO node state:

```bash
ros2 node info /yolo/yolo_node
```

### apt Hash Sum mismatch (macOS Docker)?

This project applies:

* HTTPS apt sources
* Disabled pipelining
* Disabled caching
* Full apt list purge

Included in the Dockerfile.

---

# 📌 Future Improvements

* Use webcam or real sensor instead of static images
* Bag file playback
* Create unified launch file for YOLO + publisher + subscriber
* Add Jetson/NVIDIA GPU support

---

# 📜 License

Apache 2.0 License
