#!/usr/bin/env python3
"""Run the trained warehouse YOLO model on the simulated camera.

Subscribes : /camera/image_raw                     (sensor_msgs/Image)
Publishes  : /detections/image  annotated image    (sensor_msgs/Image, bgr8)
             /yolo/detections   detections as JSON (std_msgs/String)

Run (ROS sourced, yolo_env active, simulation running):
  python3 yolo_detector.py --ros-args -p use_sim_time:=true
"""
import json
import os
import time

import numpy as np


def image_to_bgr(msg):
    """sensor_msgs/Image (rgb8, bgr8, rgba8, bgra8, mono8) -> BGR numpy array."""
    h, w, enc = msg.height, msg.width, msg.encoding
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        a = buf.reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)
        return a[:, :, ::-1].copy() if enc == 'rgb8' else a.copy()
    if enc in ('rgba8', 'bgra8'):
        a = buf.reshape(h, msg.step)[:, :w * 4].reshape(h, w, 4)[:, :, :3]
        return a[:, :, ::-1].copy() if enc == 'rgba8' else a.copy()
    if enc == 'mono8':
        g = buf.reshape(h, msg.step)[:, :w]
        return np.stack([g, g, g], axis=2)
    raise ValueError(f'Unsupported image encoding: {enc}')


def detections_to_json(stamp_s, frame_id, width, height, xyxy, conf, cls, names):
    dets = []
    for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls):
        dets.append({'class': names[int(k)], 'class_id': int(k), 'conf': round(float(c), 3),
                     'box': [round(float(x1), 1), round(float(y1), 1),
                             round(float(x2), 1), round(float(y2), 1)]})
    return json.dumps({'stamp': round(float(stamp_s), 3), 'frame': frame_id,
                       'image_w': int(width), 'image_h': int(height), 'detections': dets})


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    import torch
    from ultralytics import YOLO

    class YoloDetector(Node):
        def __init__(self):
            super().__init__('yolo_detector')
            self.declare_parameter('model_path', os.path.expanduser(
                '~/carparking_ws/agv_yolo_model/export/best.pt'))
            self.declare_parameter('image_topic', '/camera/image_raw')
            self.declare_parameter('conf', 0.40)
            self.declare_parameter('iou', 0.50)
            self.declare_parameter('imgsz', 640)
            self.declare_parameter('gpu_id', 0)          # GPU index; -1 = run on the CPU
            self.declare_parameter('min_interval', 0.0)  # seconds between inferences (wall)
            self.declare_parameter('disable_cudnn', False)  # workaround if cuDNN fails to initialise
            g = self.get_parameter
            path = os.path.expanduser(g('model_path').value)
            if not os.path.isfile(path):
                raise FileNotFoundError(f'Model not found: {path}')
            gid = int(g('gpu_id').value)
            self.device = 'cpu' if gid < 0 else gid
            self.conf, self.iou = g('conf').value, g('iou').value
            self.imgsz = int(g('imgsz').value)
            self.min_interval = g('min_interval').value

            if g('disable_cudnn').value:
                torch.backends.cudnn.enabled = False
                self.get_logger().warn('cuDNN disabled (slower, but works around init errors)')

            self.model = YOLO(path)
            self.names = self.model.names
            self.get_logger().info(f'Loaded {path}')
            self.get_logger().info(f'Classes: {dict(self.names)}')

            # warm-up now, so any GPU problem shows up at start-up, not on the first frame
            try:
                self.model.predict(np.zeros((480, 640, 3), np.uint8), conf=self.conf,
                                   imgsz=self.imgsz, device=self.device, verbose=False)
                self.get_logger().info(f'Warm-up OK on device {self.device}')
            except RuntimeError as e:
                self.get_logger().error(f'Warm-up failed: {e}')
                self.get_logger().error('Try: -p disable_cudnn:=true  or  -p gpu_id:=-1 (CPU)')
                raise

            self.pub_img = self.create_publisher(Image, '/detections/image', 1)
            self.pub_json = self.create_publisher(String, '/yolo/detections', 10)
            self.create_subscription(Image, g('image_topic').value, self.on_image,
                                     qos_profile_sensor_data)
            self.last_run = 0.0
            self.n, self.t_inf, self.t_report = 0, 0.0, time.time()
            self.Image = Image

        def on_image(self, msg):
            now = time.time()
            if now - self.last_run < self.min_interval:
                return
            self.last_run = now
            try:
                frame = image_to_bgr(msg)
            except ValueError as e:
                self.get_logger().error(str(e))
                return

            t0 = time.time()
            r = self.model.predict(frame, conf=self.conf, iou=self.iou, imgsz=self.imgsz,
                                   device=self.device, verbose=False)[0]
            self.t_inf += time.time() - t0
            self.n += 1

            b = r.boxes
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.pub_json.publish(String(data=detections_to_json(
                stamp, msg.header.frame_id, msg.width, msg.height,
                b.xyxy.cpu().numpy(), b.conf.cpu().numpy(), b.cls.cpu().numpy(), self.names)))

            ann = np.ascontiguousarray(r.plot())          # BGR with boxes + labels
            out = self.Image()
            out.header = msg.header
            out.height, out.width = int(ann.shape[0]), int(ann.shape[1])
            out.encoding, out.is_bigendian, out.step = 'bgr8', 0, int(ann.shape[1] * 3)
            out.data = ann.tobytes()
            self.pub_img.publish(out)

            if time.time() - self.t_report > 5.0:
                self.get_logger().info(
                    f'{self.n} frames, avg inference {1000 * self.t_inf / max(self.n, 1):.0f} ms, '
                    f'{len(b)} detections in last frame')
                self.n, self.t_inf, self.t_report = 0, 0.0, time.time()

    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
