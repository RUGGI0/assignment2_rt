
# RT Assignment 2: Mobile Robot Safety and Teleoperation
### (ROS2 – Gazebo Simulation)

This project implements a **teleoperation and safety supervision system** for a mobile robot simulated in **Gazebo** using ROS2.

The goal of the assignment is to combine **manual control** and **autonomous safety logic**, ensuring that the robot can be driven by the user while automatically reacting to nearby obstacles detected by onboard sensors.

The system is composed of two main custom nodes:

1. **safety_node** – Supervisory logic responsible for obstacle detection, safety evaluation, and velocity limiting.
2. **teleop_node** – A command-line user interface that allows the user to manually control the robot by sending velocity commands.

The nodes communicate through **ROS2 topics, services, and custom interfaces**, enabling a clear separation between **user control** and **real-time safety supervision**.

---

# 1. System Architecture

Below is a simplified ASCII architecture diagram showing the main nodes, topics, and data flow:

                      +-----------------------------+
                      |        teleop_node          |
                      |   (User Command Input)      |
                      +-----------------------------+
                                  |
                       publishes /cmd_vel (Twist)
                                  |
                                  v
                      +-----------------------------+
                      |        safety_node          |
                      |  (Safety & Supervision)    |
                      +-----------------------------+
                         |                 |
      subscribes /scan    |                 | publishes /cmd_vel_safe
      (LaserScan)         |                 |
                         v                 v
               +------------------+   +------------------+
               |   Gazebo Robot   |   |   Gazebo Robot   |
               |  (Simulation)    |   |  (Actuators)    |
               +------------------+   +------------------+

                      publishes /obstacle_info
                      (custom message)

---

# 2. Node Descriptions

## 2.1 safety_node

The **supervisory control node**, responsible for:

- Subscribing to `/scan` (`sensor_msgs/msg/LaserScan`)
- Monitoring the distance to nearby obstacles
- Computing the minimum obstacle distance in front of the robot
- Publishing safety information through a custom message:
  - `/obstacle_info` — `assignment2_rt_interfaces/msg/ObstacleInfo`
- Limiting or blocking velocity commands when a safety threshold is violated
- Providing ROS2 services to:
  - Set the obstacle distance threshold
  - Query average linear and angular velocities
- Acting as the only node allowed to forward velocity commands to the robot

---

## 2.2 teleop_node

A **terminal-based teleoperation interface**, offering:

- Manual input of linear and angular velocities
- Publishing velocity commands to `/cmd_vel`
- Interaction with `safety_node` through ROS2 services
- User feedback via colored terminal messages
- Simple and deterministic behaviour suitable for real-time control scenarios

---

# 3. ROS Interfaces

## 3.1 Topics

### Publishers
- `/cmd_vel` — `geometry_msgs/msg/Twist`
- `/cmd_vel_safe` — `geometry_msgs/msg/Twist`
- `/obstacle_info` — `assignment2_rt_interfaces/msg/ObstacleInfo`

### Subscribers
- `/scan` — `sensor_msgs/msg/LaserScan`
- `/cmd_vel` — `geometry_msgs/msg/Twist`

---

## 3.2 Services

Defined in the `assignment2_rt_interfaces` package:

- **SetThreshold.srv**  
  Used to dynamically change the obstacle distance threshold.

- **GetVelAvg.srv**  
  Used to retrieve average linear and angular velocities computed by the safety node.

---

## 3.3 Custom Messages

- **ObstacleInfo.msg**
  - `float32 distance`
  - `string direction`
  - `float32 threshold`

---

# 4. Behaviour Overview

The system operates in three conceptual phases:

## 1) Normal operation
- No obstacles within the safety threshold
- User velocity commands are forwarded unchanged
- Robot moves freely in the environment

## 2) Safety monitoring
- Obstacles detected but still outside the critical threshold
- Safety node continuously updates obstacle distance
- Informational messages are published on `/obstacle_info`

## 3) Safety intervention
- Obstacle distance falls below the configured threshold
- Velocity commands are limited or blocked
- Robot motion is reduced or stopped to prevent collisions

---

# 5. Running the Project Manually

## 5.1 Build the workspace

```bash
rm -rf build install log
colcon build
source install/setup.bash
## 5.2 Start Gazebo Simulation

Open **Terminal 1** and launch the Gazebo simulation containing the mobile robot and sensors.

```
source install/setup.bash
# Example (actual launch file may vary)
# ros2 launch bme_gazebo_sensors gazebo.launch.py
```

The simulation publishes sensor data (e.g. `/scan`) used by the safety node.

---

## 5.3 Start safety_node

Open **Terminal 2**:

```
source install/setup.bash
ros2 run assignment2_rt safety_node
```

The safety node:
- subscribes to `/scan`
- evaluates obstacle distances
- publishes `/obstacle_info`
- limits or blocks unsafe velocity commands

---

## 5.4 Start teleop_node

Open **Terminal 3**:

```
source install/setup.bash
ros2 run assignment2_rt teleop_node
```

The teleoperation node:
- accepts user input from terminal
- publishes velocity commands to `/cmd_vel`
- interacts with the safety node via services

---

# 6. Running the Project with Scripts (recommended)

To simplify execution, two automation scripts are provided:

- `launcher.sh`
- `run_assignment.sh`

The scripts automatically:

- create a `tmux` 2×2 session
- source the ROS2 environment
- start `safety_node` and `teleop_node`
- provide a persistent menu-based launcher

Run the launcher with:

```
cd ~/ros2_workshop/src/assignment2_rt
chmod +x launcher.sh run_assignment.sh
./launcher.sh
```

---

# 7. Terminal Layout

When launched via script, the tmux session is arranged as follows:

```
+---------------------------+---------------------------+
| Pane 0                    | Pane 1                    |
| Environment / Gazebo      | Topic monitoring & debug  |
+---------------------------+---------------------------+
| Pane 2                    | Pane 3                    |
| safety_node               | teleop_node               |
+---------------------------+---------------------------+
```

This layout separates:
- simulation
- observation
- safety logic
- user control

---

# 8. Terminal Requirements

The launcher relies on the presence of:

```
x-terminal-emulator
```

This ensures compatibility with GNOME Terminal, XTerm, Terminator, and similar emulators.

Check availability:

```
which x-terminal-emulator
```

Configure if needed:

```
sudo update-alternatives --config x-terminal-emulator
```

---

# 9. Package Structure

The project is composed of the following ROS2 packages:

- assignment2_rt  
  Contains `teleop_node` and `safety_node`.

- assignment2_rt_interfaces  
  Defines custom messages and services used for inter-node communication.

- bme_gazebo_sensors  
  Provides Gazebo sensor plugins and simulation assets.
