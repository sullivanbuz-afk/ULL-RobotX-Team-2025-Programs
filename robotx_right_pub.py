#!/home/jetson-orin/yolo-venv/bin/python
# ROS 2 YOLOv8 RTSP publisher (RIGHT camera) with per-class distance + zones/bearing
# Adds numeric 5-tuples to /cam_right/minimal for the policy:
#   [cls_idx, conf, bearing_deg, dist_m, 0.0] (repeated per detection)

import os, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float32MultiArray
import cv2
import torch
from ultralytics import YOLO
import numpy as np

# ---------- Defaults (overridable via ROS params or envs) ----------
ID_TO_NAME   = {0: "black_buoy", 1: "green_buoy", 2: "red_buoy"}
ID_TO_COLOR  = {0: (0,0,0),       1: (0,255,0),    2: (0,0,255)}

DEFAULT_FY = {
    2: float(os.getenv("FY_RED",   "630.7")),
    1: float(os.getenv("FY_GREEN", "660.3")),
    0: float(os.getenv("FY_BLACK", "687"))
}
H_REAL = {
    2: float(os.getenv("H_RED",   "1.2319")),
    1: float(os.getenv("H_GREEN", "1.2319")),
    0: float(os.getenv("H_BLACK", "0.5715"))
}

def fy_for_class(cid:int)->float:
    fy = DEFAULT_FY.get(cid, 0.0)
    if fy > 0: return fy
    vals = [v for v in DEFAULT_FY.values() if v > 0]
    return sum(vals)/len(vals) if vals else 650.0

def estimate_distance(cid:int, h_px:float)->float:
    if h_px <= 1: return float("nan")
    return (H_REAL.get(cid,1.0) * fy_for_class(cid)) / float(h_px)

def zone_and_bearing(cx_px:int, width:int, hfov_deg:float, num_zones:int):
    width  = max(1, int(width))
    zone_w = width / float(num_zones)
    zone_i = int(np.clip(cx_px // zone_w + 1, 1, num_zones))
    zone_label = {1:"LEFT", 2:"MIDDLE", 3:"RIGHT"}.get(zone_i, str(zone_i)) if num_zones==3 else str(zone_i)
    norm = (cx_px + 0.5)/width - 0.5
    bearing = norm * hfov_deg
    return zone_label, bearing

def draw_dets(img, result, hfov_deg, num_zones):
    out = img.copy()
    if result is None or result.boxes is None or result.boxes.xyxy is None:
        return out
    xyxy  = result.boxes.xyxy.cpu().numpy()
    clss  = result.boxes.cls.int().cpu().numpy() if result.boxes.cls is not None else []
    confs = result.boxes.conf.cpu().numpy() if result.boxes.conf is not None else []
    h_img, w_img = out.shape[:2]
    for i in range(len(xyxy)):
        x1, y1, x2, y2 = [int(v) for v in xyxy[i]]
        h   = max(1, y2 - y1)
        cx  = (x1 + x2)//2
        cid = int(clss[i]) if i < len(clss) else -1
        conf= float(confs[i]) if i < len(confs) else 0.0
        name  = ID_TO_NAME.get(cid, f"id{cid}")
        color = ID_TO_COLOR.get(cid, (255,255,255))
        z_est = estimate_distance(cid, h)
        zone, bearing = zone_and_bearing(cx, w_img, hfov_deg, num_zones)
        cv2.rectangle(out, (x1,y1), (x2,y2), color, 2)
        label = f"{name} | h={h}px | conf={conf:.2f} | est={z_est:.2f} m | zone={zone} | b={bearing:+.1f}°"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        y_text = max(y1 - 6, th + 6)
        cv2.rectangle(out, (x1, y_text - th - 6), (x1 + tw + 6, y_text + 2), (255,255,255), -1)
        cv2.putText(out, label, (x1 + 3, y_text - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2, cv2.LINE_AA)
    return out

class RtspYolov8Right(Node):
    def _open_stream(self, tag:str, url:str):
        """Open one RTSP stream, GStreamer first then TCP fallback."""
        self.get_logger().info(f"[{tag}] Opening stream: {url}")
        gst = (
            f"rtspsrc location={url} latency=50 drop-on-late=true ! "
            f"rtph264depay ! h264parse ! "
            f"nvv4l2decoder ! "
            f"videoconvert ! video/x-raw,format=BGR ! "
            f"appsink sync=false max-buffers=1 drop=true"
        )
        cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self.get_logger().warn(f"[{tag}] GStreamer failed; falling back to default VideoCapture (TCP)")
            cap = cv2.VideoCapture(url)
            try: cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception: pass
        if not cap.isOpened():
            raise RuntimeError(f"[{tag}] Could not open RTSP stream")
        return cap

    def __init__(self):
        super().__init__('rtsp_yolov8_right')

        # -------- Parameters (declare + read) --------
        self.declare_parameter('rtsp_url', os.getenv("RTSP_URL", "rtsp://admin:BayouBot@192.168.1.142:554/h264Preview_01_sub"))
        self.declare_parameter('cam_tag',  os.getenv("CAM_TAG", "R"))
        self.declare_parameter('weights',  os.getenv("WEIGHTS", os.path.expanduser("~/Documents/buoys/runs/train6/weights/best.pt")))
        self.declare_parameter('imgsz',    int(os.getenv("IMGSZ", "320")))
        self.declare_parameter('conf',     float(os.getenv("CONF", "0.40")))
        self.declare_parameter('device',   os.getenv("DEVICE", "cpu").strip())
        self.declare_parameter('max_infer_fps', float(os.getenv("MAX_INFER_FPS", "2.0")))
        self.declare_parameter('headless', int(os.getenv("HEADLESS", "1")))
        self.declare_parameter('hfov_deg', float(os.getenv("HFOV_DEG", "87")))
        self.declare_parameter('num_zones',int(os.getenv("NUM_ZONES", "3")))

        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.cam_tag  = self.get_parameter('cam_tag').value
        self.weights  = self.get_parameter('weights').value
        self.imgsz    = int(self.get_parameter('imgsz').value)
        self.conf     = float(self.get_parameter('conf').value)
        device_env    = self.get_parameter('device').value
        self.max_infer_fps = float(self.get_parameter('max_infer_fps').value)
        self.headless = int(self.get_parameter('headless').value)
        self.hfov_deg = float(self.get_parameter('hfov_deg').value)
        self.num_zones= int(self.get_parameter('num_zones').value)

        # Publishers:
        # - Human-readable String (kept for debugging)
        # - Numeric 5-tuples for policy
        self.pub      = self.create_publisher(String,            'yolov8/Right/detections', 10)
        self.min_pub  = self.create_publisher(Float32MultiArray, 'cam_right/minimal',       10)
        self.fps_pub  = self.create_publisher(Float32,           'yolov8/Right/fps',        10)

        device = "cpu" if device_env.lower() == "cpu" else (0 if torch.cuda.is_available() else "cpu")
        self.device = device

        self.get_logger().info(f"[{self.cam_tag}] Torch {torch.__version__} | CUDA? {torch.cuda.is_available()} | device={device}")
        self.get_logger().info(f"[{self.cam_tag}] WEIGHTS={self.weights} IMGSZ={self.imgsz} CONF={self.conf} MAX_INFER_FPS={self.max_infer_fps}")
        self.get_logger().info(f"[{self.cam_tag}] FY: red={fy_for_class(2):.1f}, green={fy_for_class(1):.1f}, black={fy_for_class(0):.1f}")
        self.get_logger().info(f"[{self.cam_tag}] Zoning: NUM_ZONES={self.num_zones} | zone_width={self.hfov_deg/self.num_zones:.2f}° | HFOV={self.hfov_deg}°")

        if not os.path.exists(self.weights):
            raise FileNotFoundError(f"Could not find weights at: {self.weights}")

        self.model = YOLO(self.weights)
        try: self.model.fuse()
        except Exception: pass

        # open this camera only
        self.cap = self._open_stream(f"cam={self.cam_tag}", self.rtsp_url)

        if not self.headless:
            cv2.namedWindow(f"YOLOv8 RTSP - {self.cam_tag}", cv2.WINDOW_NORMAL)

        self.last_infer_t = 0.0
        self.min_infer_period = 1.0 / max(0.1, self.max_infer_fps)
        self.last_drawn = None

        self.timer = self.create_timer(0.0, self.loop)  # as fast as possible

    def _process_one(self, frame):
        rs = self.model.predict(source=frame, imgsz=self.imgsz, conf=self.conf, device=self.device, verbose=False)
        result = rs[0] if rs else None

        det_lines = []
        payload = []  # numeric 5-tuples for policy: [cls_idx, conf, bearing_deg, dist_m, 0.0]*N

        if result is not None and result.boxes is not None and result.boxes.xyxy is not None:
            xyxy  = result.boxes.xyxy.cpu().numpy()
            clss  = result.boxes.cls.int().cpu().numpy() if result.boxes.cls is not None else []
            confs = result.boxes.conf.cpu().numpy() if result.boxes.conf is not None else []
            h_img, w_img = frame.shape[:2]

            for i in range(len(xyxy)):
                x1, y1, x2, y2 = [int(v) for v in xyxy[i]]
                h   = max(1, y2 - y1)
                cx  = (x1 + x2)//2
                cid = int(clss[i]) if i < len(clss) else -1
                conf= float(confs[i]) if i < len(confs) else 0.0

                name  = ID_TO_NAME.get(cid, f"id{cid}")
                z_est = estimate_distance(cid, h)
                zone, bearing = zone_and_bearing(cx, w_img, self.hfov_deg, self.num_zones)

                # Console/debug String
                self.get_logger().info(
                    f"[{self.cam_tag}] {name:<9} | h={h:>3}px | conf={conf:0.2f} | est={z_est:0.2f} m | zone={zone} | b={bearing:+.1f}°"
                )
                det_lines.append(f"cam={self.cam_tag}:{name}:{conf:0.2f}:{h}px:{z_est:0.2f}m:zone={zone}:bearing={bearing:+.1f}")

                # Numeric 5-tuple for policy
                # Note: cid must be 0/1/2 matching CLASS_ORDER in the policy
                if cid in (0,1,2) and np.isfinite(z_est):
                    payload += [float(cid), float(conf), float(bearing), float(z_est), 0.0]

        # Publish minimal (numeric) if we have any detections
        if payload:
            m = Float32MultiArray(); m.data = payload
            self.min_pub.publish(m)

        drawn = draw_dets(frame, result, self.hfov_deg, self.num_zones) if result is not None else frame
        return det_lines, drawn

    def loop(self):
        now = time.perf_counter()
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn(f"[{self.cam_tag}] Failed to grab frame. Retrying in 2s...")
            try:
                self.cap.release()
            except Exception:
                pass
            time.sleep(2.0)
            try:
                self.cap = self._open_stream(f"cam={self.cam_tag}", self.rtsp_url)
            except Exception as e:
                self.get_logger().error(f"[{self.cam_tag}] Reopen failed: {e}")
            return

        dets = []
        if (now - self.last_infer_t) >= self.min_infer_period:
            self.last_infer_t = now
            dets, self.last_drawn = self._process_one(frame)

        frame_show = self.last_drawn if self.last_drawn is not None else frame

        # publish human-readable + fps
        self.pub.publish(String(data="; ".join(dets)))
        fps = 1.0 / max(1e-6, (time.perf_counter() - now))
        self.fps_pub.publish(Float32(data=float(fps)))

        if not self.headless:
            cv2.imshow(f"YOLOv8 RTSP - {self.cam_tag}", frame_show)
            if cv2.waitKey(1) & 0xFF == 27:
                rclpy.shutdown()

def main():
    rclpy.init()
    node = RtspYolov8Right()
    try:
        rclpy.spin(node)
    finally:
        try:
            if getattr(node, "cap", None): node.cap.release()
        except Exception:
            pass
        if not node.headless:
            try: cv2.destroyAllWindows()
            except Exception: pass
        node.destroy_node()

if __name__ == "__main__":
    main()