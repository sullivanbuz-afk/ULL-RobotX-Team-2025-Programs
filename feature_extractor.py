#!/home/jetson-orin/yolo-venv/bin/python
# Feature extractor: camera String logs -> minimal numeric quads
# Publishes ONLY when detections are present (no empty publishes, no skip logs).
# Output Float32MultiArray: [class_idx, confidence, zone_num, distance_m, ...]
# class_idx: black=0, green=1, red=2 ; zone_num: LEFT=-1, MIDDLE/CENTER=0, RIGHT=+1

from typing import List
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String as StringMsg
from std_msgs.msg import Float32MultiArray

# ----- Config (class order & zone encoding) -----
CLASS_ORDER = ['black', 'green', 'red']
CLASS_INDEX = {c: i for i, c in enumerate(CLASS_ORDER)}
ZONE_MAP = {
    'LEFT':  -1.0, 'L': -1.0,
    'MIDDLE': 0.0, 'CENTER': 0.0, 'C': 0.0,
    'RIGHT': +1.0, 'R': +1.0,
}

# Pattern A: your compact colon-delimited format
# "cam=L:green_buoy:0.68:235px:3.46m:zone=MIDDLE:bearing=+1.6"
PAT_COLON = re.compile(
    r'cam\s*=\s*[LR]\s*:'                 # cam=L or cam=R
    r'\s*(black|green|red)_buoy\s*:'      # class
    r'\s*([0-9]*\.?[0-9]+)\s*:'           # confidence
    r'\s*[0-9]+px\s*:'                    # height px (ignored)
    r'\s*([0-9]*\.?[0-9]+)\s*m?\s*:'      # distance meters (3.46m or 3.46)
    r'\s*zone\s*=\s*([A-Za-z]+)',         # zone=MIDDLE
    re.IGNORECASE
)

# Pattern B: pipe-delimited variant (kept for compatibility)
# "[R] red_buoy | h=217px | conf=0.66 | est=3.58 m | zone=MIDDLE | b=-6.1°"
PAT_PIPE = re.compile(
    r'(black|green|red)_buoy'
    r'.*?conf\s*=\s*([0-9]*\.?[0-9]+)'
    r'.*?(?:est|dist)\s*=\s*([0-9]*\.?[0-9]+)\s*m'
    r'.*?zone\s*=\s*([A-Za-z]+)',
    re.IGNORECASE | re.DOTALL
)

def parse_from_string(payload: str, min_conf: float = 0.0) -> List[float]:
    """Return quads [class_idx, conf, zone_num, dist_m] for all matches in the payload."""
    out: List[float] = []
    text = payload or ''

    # Try colon-delimited first
    matches = PAT_COLON.findall(text)
    if matches:
        for cls, conf, dist, zone in matches:
            conf_f = float(conf)
            if conf_f < min_conf:
                continue
            cls_idx  = CLASS_INDEX[cls.lower()]
            dist_f   = float(dist)
            zone_num = float(ZONE_MAP.get(zone.upper(), 0.0))
            out += [cls_idx, conf_f, zone_num, dist_f]
        return out

    # Fallback: pipe-delimited
    matches = PAT_PIPE.findall(text)
    if matches:
        for cls, conf, dist, zone in matches:
            conf_f = float(conf)
            if conf_f < min_conf:
                continue
            cls_idx  = CLASS_INDEX[cls.lower()]
            dist_f   = float(dist)
            zone_num = float(ZONE_MAP.get(zone.upper(), 0.0))
            out += [cls_idx, conf_f, zone_num, dist_f]
    return out

class FeatureExtractor(Node):
    """
    Subscribes to raw camera topics (String) and publishes minimal numeric arrays
    ONLY when at least one detection is present. No debug prints unless enabled.
    """
    def __init__(self):
        super().__init__('feature_extractor')

        # ---- Parameters ----
        self.declare_parameter('left_in',   '/yolov8/Left/detections')
        self.declare_parameter('right_in',  '/yolov8/Right/detections')
        self.declare_parameter('left_out',  'cam_left/minimal')
        self.declare_parameter('right_out', 'cam_right/minimal')
        self.declare_parameter('min_conf',  0.0)
        self.declare_parameter('log_info_on_publish', False)  # if True, log one line per publish

        self.left_in   = self.get_parameter('left_in').value
        self.right_in  = self.get_parameter('right_in').value
        self.left_out  = self.get_parameter('left_out').value
        self.right_out = self.get_parameter('right_out').value
        self.min_conf  = float(self.get_parameter('min_conf').value)
        self.log_info  = bool(self.get_parameter('log_info_on_publish').value)

        # ---- Publishers ----
        self.pub_L = self.create_publisher(Float32MultiArray, self.left_out, 10)
        self.pub_R = self.create_publisher(Float32MultiArray, self.right_out, 10)

        # ---- Subscribers ----
        self.sub_L = self.create_subscription(StringMsg, self.left_in,  self.cb_left, qos_profile_sensor_data)
        self.sub_R = self.create_subscription(StringMsg, self.right_in, self.cb_right, qos_profile_sensor_data)

        self.get_logger().info(
            f"[feature_extractor] input=({self.left_in}, {self.right_in}) -> output=({self.left_out}, {self.right_out}) "
            f"(min_conf={self.min_conf}, quiet={'yes' if not self.log_info else 'no'})"
        )

    def emit(self, floats: List[float], pub, side_label: str):
        msg = Float32MultiArray()
        msg.data = [float(x) for x in floats]
        pub.publish(msg)
        if self.log_info:
            # One concise line per publish, includes count and first quad preview
            preview = floats[:4] if len(floats) >= 4 else []
            self.get_logger().info(f"[{side_label}] published {len(floats)//4} det(s) {preview}")

    def cb_left(self, msg: StringMsg):
        floats = parse_from_string(msg.data, self.min_conf)
        if floats:
            self.emit(floats, self.pub_L, 'L')

    def cb_right(self, msg: StringMsg):
        floats = parse_from_string(msg.data, self.min_conf)
        if floats:
            self.emit(floats, self.pub_R, 'R')

def main():
    rclpy.init()
    rclpy.spin(FeatureExtractor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
