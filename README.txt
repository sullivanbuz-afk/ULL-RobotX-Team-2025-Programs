# RobotX ROS2 Control Stack

This repository contains the core ROS 2 Python nodes that form the complete control pipeline for your boat.  
The system connects an RC receiver → selects a working mode → generates servo angles → drives hardware servos → displays LED status → triggers throttle power actuators.

---

## Node Overview

### **1. `rc_receiver.py`**
**Role:** Interface between the RC receiver microcontroller and ROS 2.

- Opens a serial connection to `/dev/ttyACM0` at 115200 baud.
- Expects comma-separated integers (one line per frame).
- Parses into a list of channel values.
- Publishes them as `Int32MultiArray` on `rc_channels`.

**ROS Topics:**
- **Published**
  - `rc_channels` (`std_msgs/Int32MultiArray`)

---

### **2. `working_mode_publisher.py`**
**Role:** Determine the boat’s *working mode* from RC channel values.

Logic uses CH1–CH4 to choose among:

- `EMERGENCY STOP`
- `STANDBY`
- `REMOTE CONTROL`
- `AUTONOMOUS`

**ROS Topics:**
- **Subscribed**
  - `rc_channels`
- **Published**
  - `working_mode` (`std_msgs/String`)

---

### **3. `remote_control.py`**
**Role:** Convert RC PWM throttle signals into servo angles.

Features:

- Maps PWM → servo angle (0–180°)
- Applies:
  - deadband
  - exponential smoothing
  - slew rate limiting
- Publishes left/right steering angles as `remote_angle`
- Drives a relay GPIO for EMERGENCY STOP

**ROS Topics:**
- **Subscribed**
  - `rc_channels`
  - `working_mode`
- **Published**
  - `remote_angle` (`std_msgs/Int32MultiArray`)

---

### **4. `angle_combiner.py`**
**Role:** Merge both control sources (remote + autonomous) into one array.

- Subscribes to `remote_angle`
- Subscribes to `auto_angle`
- Publishes `[remote_left, remote_right, auto_left, auto_right]` as `servo_angles`

**ROS Topics:**
- **Subscribed**
  - `remote_angle`
  - `auto_angle`
- **Published**
  - `servo_angles` (`std_msgs/Int32MultiArray`)

---

### **5. `servo_writer.py`**
**Role:** Write final servo commands to PCA9685 hardware.

Behavior depends on working mode:

- `REMOTE CONTROL`, `STANDBY`, `EMERGENCY STOP` → use remote angles  
- `AUTONOMOUS` → use auto angles  
- Unknown mode → neutral (80°)

Also publishes `servo_angles_echo` for latency measurements.

**ROS Topics:**
- **Subscribed**
  - `working_mode`
  - `servo_angles`
- **Published**
  - `servo_angles_echo` (`std_msgs/Int32MultiArray`)

---

### **6. `led_control.py`**
**Role:** Show system mode using LED relays.

LED patterns:

| Mode             | Green | Yellow | Red |
|------------------|-------|--------|-----|
| REMOTE CONTROL   | ON    | ON     | OFF |
| STANDBY          | OFF   | ON     | OFF |
| AUTONOMOUS       | ON    | OFF    | OFF |
| EMERGENCY STOP   | OFF   | OFF    | ON  |

**ROS Topics:**
- **Subscribed**
  - `working_mode`

---

### **7. `throttle_power.py`**
**Role:** Trigger physical throttle power buttons using dedicated servos.

- Watches CH5 on the RC input
- If CH5 < 1450 → press left power button  
- If CH5 > 1550 → press right power button  
- Otherwise → keep servos in neutral

**ROS Topics:**
- **Subscribed**
  - `rc_channels`

---

## High-Level Signal Flow

```text
[RC Transmitter]
        ↓
[Receiver + Microcontroller] --(serial CSV)--> rc_receiver.py
        ↓
   rc_channels (Int32MultiArray)
        ↓                       ↘
working_mode_publisher.py        throttle_power.py
        ↓
 working_mode (String)
    ↓           ↘
remote_control.py   led_control.py
        ↓
  remote_angle (Int32MultiArray)
        ↘
    angle_combiner.py ← auto_angle (from autonomous node)
        ↓
 servo_angles (Int32MultiArray)
        ↓
    servo_writer.py → PCA9685 → physical servos (channels 0 & 15)
