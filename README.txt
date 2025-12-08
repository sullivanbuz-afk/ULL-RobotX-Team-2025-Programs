# RobotX ROS2 Vision & Policy Stack (WAM-V)

This repository contains the core ROS 2 Python nodes that implement the **autonomous vision → control pipeline** for a WAM-V boat in RobotX tasks.  
The system uses **two Reolink RTSP cameras + YOLOv8 on a Jetson Orin Nano** to detect buoys, estimate distance/bearing, plan behavior (gates & wildlife), and output thrust commands that are converted into servo angles for the thrusters.

Target tasks:

- **Entrance & Exit Gates** – detect red/green buoys, form gates, and drive through the center.  
- **Wildlife Encounter** – detect a black buoy and circle it at a target radius.

Built on:

- **Ubuntu 22.04.5 (Jammy Jellyfish)**
- **ROS 2 Humble**
- **NVIDIA Jetson Orin Nano**

---

## Node Overview

### **1. `left_pub.py`** – Left Camera YOLOv8 RTSP Publisher

**Role:** Run YOLOv8 on the **left Reolink RTSP camera**, estimate distance & bearing to buoys, and publish compact numeric detections.

- Opens RTSP stream (GStreamer first, then TCP fallback) from the left camera.
- Loads YOLOv8 weights (custom buoy detector).
- For each detection, estimates distance using a pinhole model:
  - Class-specific focal lengths (`FY_RED`, `FY_GREEN`, `FY_BLACK`)
  - Known buoy heights (`H_RED`, `H_GREEN`, `H_BLACK`)
- Computes **bearing** from image x-coordinate and camera HFOV.
- Publishes **5-tuples** per detection:
  ```text
  [cls_idx, conf, bearing_deg, dist_m, spare]



### **2. right_pub.py – Right Camera YOLOv8 RTSP Publisher

Role: Same as left_pub.py but for the right Reolink camera.

Opens RTSP stream from the right camera.

Runs YOLOv8 on each frame.

Estimates distance & bearing for each buoy.

Publishes the same 5-tuple format for the policy node.

ROS Topics:
Published

yolov8/Right/detections (std_msgs/String)
cam_right/minimal (std_msgs/Float32MultiArray)
yolov8/Right/fps (std_msgs/Float32)





### **3. feature_extractor.py -- Minimal Feature Converter
Role: Parse the string YOLO logs into compact numeric features.
This node is useful if you are using the older string-only topics instead of the 5-tuple outputs from left_pub / right_pub.

Subscribes to left/right YOLO string topics.

Uses regex to parse class, confidence, distance, and zone (LEFT/MIDDLE/RIGHT).

Converts them into quads:
[class_idx, confidence, zone_num, distance_m]
where zone_num ∈ {-1, 0, +1}.

Publishes quads only when detections are present.

ROS Topics:

Subscribed
yolov8/Left/detections (std_msgs/String)
yolov8/Right/detections (std_msgs/String)

Published
cam_left/minimal (std_msgs/Float32MultiArray)
cam_right/minimal (std_msgs/Float32MultiArray)

Note: The policy node can handle both 5-tuple (bearing-based) and 4-tuple (zone-based) formats, so you can either


### **4. policy.py – Gate Navigation & Wildlife Policy

Role: Fuse left/right camera detections and decide how to drive the boat.
Outputs normalized thrust commands [uL, uR] used later to generate servo angles.

Inputs:
cam_left/minimal (std_msgs/Float32MultiArray)
cam_right/minimal (std_msgs/Float32MultiArray)

Accepted formats per detection:

Preferred 5-tuple:
[cls_idx, conf, bearing_deg, dist_m, spare]

Fallback 4-tuple:
[cls_idx, conf, zone_num, dist_m] → zone mapped back to approximate bearing.

Output:
thrust_cmd (std_msgs/Float32MultiArray)
[uL, uR] in [-1.0, +1.0]

Main behaviors:

Panic Stop
	If any buoy is detected closer than panic_distance_m, command is [0.0, 0.0].

Entrance/Exit Gate Navigation

		Forms candidate red/green pairs across both cameras.

	Uses distance & bearing to compute:

		Gate width in meters.

		Gate center bearing.

	Accepts only gates with:
	
		Width within [min_gate_width_m, max_gate_width_m].

		Similar ranges for red & green (gate_max_range_diff_m).

		Picks the nearest valid gate and:

		Steers toward the gate center.
		
		Sets forward speed from distance (slow when close, cruise when far).

Wildlife Encounter – Black Buoy Circle Mode

		If black buoy is present and no valid gate:

	Tries to keep buoy at:

		Target radius (circle_target_radius_m) and

		Target bearing offset (circle_bearing_setpoint_deg).
	
		Uses simple proportional control on radius and bearing to circle around the buoy.

Red/Green Avoidance (No Gate)

	If no gate but red/green buoys are visible:

		Finds nearest red/green buoy.

		Steers away from it while moving forward (basic obstacle avoidance).

Hold & Cruise

	When detections disappear:

For no_det_hold_s seconds, hold last maneuver instead of snapping immediately back 								to straight.

After that, revert to straight cruise (forward_base on both sides)


### **5. thrust_mixer.py – Thrust → Servo Angle Mapper

Role: Convert the normalized thrust commands from policy.py into servo angles for both thrusters and publish them for the Pi/servo side.

Input:
thrust_cmd (std_msgs/Float32MultiArray)
[uL, uR] with each in [-1, +1]

Output:
auto_angle (std_msgs/Int32MultiArray)
[left_deg, right_deg] in degrees for hardware servos.

Features:
	Parameters define neutral and full-throw angles:
	deg_min, deg_mid, deg_max
	(e.g. [0°, 80°, 170°])

Computes target angle as:
angle = deg_mid + span * u
clamped to [deg_min, deg_max]

Option to disallow reverse (allow_reverse) by clipping negative u.

Applies:
	Exponential moving average smoothing (output_alpha)

	Slew rate limiting (slew_deg_per_s) for mechanical safety.

If thrust_cmd is stale longer than stale_timeout_s, smoothly returns both angles to deg_mid.

ROS Topics:

Subscribed
thrust_cmd (std_msgs/Float32MultiArray)

Published
auto_angle (std_msgs/Int32MultiArray)



High-Level Signal Flow:

[Reolink Left Camera]                      [Reolink Right Camera]
        ↓                                             ↓
    left_pub.py                                   right_pub.py
 (YOLOv8 + distance/bearing)                (YOLOv8 + distance/bearing)
        ↓                                             ↓
  cam_left/minimal (Float32MultiArray)   cam_right/minimal (Float32MultiArray)
        ↘_____________________________________↙
                     policy.py
     (gate detection, wildlife circling, avoidance,
          speed/steering, CSV logging for ML)
                      ↓
          thrust_cmd (Float32MultiArray [uL, uR])
                      ↓
                 thrust_mixer.py
      (map normalized thrust to servo angles)
                      ↓
          auto_angle (Int32MultiArray [L_deg, R_deg])
                      ↓
          [Pi/servo writer] → Thruster servos / ESCs



