#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
from typing import Dict, List

import numpy as np
import torch
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D, Detection2DArray,
    ObjectHypothesisWithPose, BoundingBox2D,
    Pose2D
)
from cv_bridge import CvBridge

# Ultralytics YOLO (v8+)
try:
    from ultralytics import YOLO
except ImportError as e:
    raise RuntimeError("Ultralytics nicht installiert: pip install ultralytics") from e


def _default_model_path() -> str:
    """Share-Verzeichnis bevorzugen, sonst Source-Fallback."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('ricoh_theta_ros')
        cand = os.path.join(share, 'resource', 'best.pt')
        if os.path.exists(cand):
            return cand
    except Exception:
        pass
    return os.path.expanduser('~/ros2_ws/src/ricoh_theta_ros/resource/best.pt')


class YoloDetector(Node):
    """
    Abonniert sechs Roh-Streams (/ricoh_theta/<view>) und publiziert pro View:
      /detections/<view>            (vision_msgs/Detection2DArray, immer)
      /detections/<view>/annotated  (sensor_msgs/Image, NUR bei >=1 Detektion)
    """

    def __init__(self):
        super().__init__("yolo_detector")

        # Parameter
        self.declare_parameter("model_path", _default_model_path())
        self.declare_parameter("input_topics", [
            "/ricoh_theta/front",
            "/ricoh_theta/right",
            "/ricoh_theta/back",
            "/ricoh_theta/left",
            "/ricoh_theta/up",
            "/ricoh_theta/down",
        ])
        self.declare_parameter("output_namespace", "/detections")
        self.declare_parameter("publish_annotated", True)  # nur bei >=1 Box
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "auto")     # "cpu", "0", "cuda:0", "auto"
        self.declare_parameter("device_index", -1)   # erlaubt int-Override
        self.declare_parameter("half", True)         # FP16 auf CUDA

        # Parameter lesen
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        topics: List[str] = list(self.get_parameter("input_topics").value)
        self.output_ns: str = self.get_parameter("output_namespace").get_parameter_value().string_value
        self.publish_annotated: bool = bool(self.get_parameter("publish_annotated").value)
        self.conf: float = float(self.get_parameter("conf").value)
        self.iou: float = float(self.get_parameter("iou").value)
        self.imgsz: int = int(self.get_parameter("imgsz").value)
        device_param = self.get_parameter("device").get_parameter_value().string_value
        device_index = int(self.get_parameter("device_index").value)
        self.use_half: bool = bool(self.get_parameter("half").value)

        # Gerät ermitteln
        self.device = self._resolve_device(device_param, device_index)

        self.get_logger().info(f"Lade YOLO Modell: {model_path} auf {self.device}")
        self.model = YOLO(model_path)
        self.counter = 0

        # Modell explizit verschieben (optional, aber robust)
        try:
            if self.device != "cpu":
                self.model.to(self.device)  # "0" oder "cuda:0"
        except Exception as e:
            self.get_logger().warn(f"Konnte Modell nicht explizit verschieben ({e}); Ultralytics verwaltet das ggf. intern.")

        try:
            self.model.fuse()
        except Exception:
            pass

        # QoS für Kameradaten
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.bridge = CvBridge()

        # Subscriber/Publisher pro Topic
        self.subscribers: Dict[str, rclpy.subscription.Subscription] = {}
        self.pub_dets: Dict[str, rclpy.publisher.Publisher] = {}
        self.pub_imgs: Dict[str, rclpy.publisher.Publisher] = {}

        for t in topics:
            view = t.split("/")[-1] or t
            self.subscribers[view] = self.create_subscription(
                Image, t, lambda msg, v=view: self.on_image_named(msg, v), qos
            )
            det_topic = f"{self.output_ns}/{view}"
            ann_topic = f"{self.output_ns}/{view}/annotated"
            self.pub_dets[view] = self.create_publisher(Detection2DArray, det_topic, 10)
            if self.publish_annotated:
                self.pub_imgs[view] = self.create_publisher(Image, ann_topic, 10)

            # self.get_logger().info(f"Abonniere: {t} -> View='{view}'")
            # self.get_logger().info(f"Publiziere: {det_topic}" + (" und " + ann_topic if self.publish_annotated else ""))

        # Klassenbezeichnungen
        self.class_names = {}
        try:
            if hasattr(self.model, "names"):
                self.class_names = self.model.names
        except Exception:
            pass

    def _resolve_device(self, dev_str: str, dev_idx: int) -> str:
        if dev_idx >= 0:
            return str(dev_idx)
        dv = (dev_str or "").strip()
        if dv == "" or dv.lower() == "auto":
            return "0" if torch.cuda.is_available() else "cpu"
        return dv

    def on_image_named(self, msg: Image, view_name: str):
        start_time = time.time()
        t0 = time.time()

        # ROS Image -> OpenCV (BGR)
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge Fehler ({view_name}): {e}")
            return

        # Inferenz
        try:
            half_flag = self.use_half and (self.device != "cpu")
            results = self.model.predict(
                source=frame,
                conf=self.conf,
                iou=self.iou,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False,
                half=half_flag
            )
        except Exception as e:
            self.get_logger().error(f"YOLO predict Fehler ({view_name}): {e}")
            return

        det_array = Detection2DArray()
        det_array.header = msg.header

        annotated = None  # wir erzeugen nur bei >=1 Box ein Overlay

        r = None
        try:
            r = results[0]
        except Exception:
            r = None

        num_boxes = 0
        if r is not None and getattr(r, "boxes", None) is not None and len(r.boxes) > 0:
            xyxy = r.boxes.xyxy.detach().cpu().numpy()
            confs = r.boxes.conf.detach().cpu().numpy()
            clss = r.boxes.cls.detach().cpu().numpy().astype(int)
            num_boxes = len(xyxy)

            # Nur jetzt ein Annotated-Frame erzeugen
            if self.publish_annotated:
                annotated = frame.copy()
            for (x1, y1, x2, y2), score, cls in zip(xyxy, confs, clss):
                det = Detection2D()
                det.header = msg.header

                cx = float((x1 + x2) / 2.0)
                cy = float((y1 + y2) / 2.0)
                w  = float(max(0.0, x2 - x1))
                h  = float(max(0.0, y2 - y1))

                # BoundingBox2D robust bauen
                bbox = BoundingBox2D()
                CenterCls = type(bbox.center)   # exakt die erwartete Klasse
                center = CenterCls()
                # Standard-Feldnamen
                if hasattr(center, 'position'):
                    center.position.x = cx
                    center.position.y = cy
                    center.theta = 0.0
                else:
                    # Fallbacks (exotische Generatoren)
                    for name, val in (('x', cx), ('y', cy), ('theta', 0.0),
                                      ('x_', cx), ('y_', cy), ('theta_', 0.0)):
                        if hasattr(center, name):
                            setattr(center, name, val)
                bbox.center = center
                bbox.size_x = w
                bbox.size_y = h
                det.bbox = bbox
                print("center_x: ", center.position.x, flush=True)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(cls)
                hyp.hypothesis.score = float(score)
                det.results.append(hyp)

                det_array.detections.append(det)

                # --- Neue Log-Ausgabe ---
                label = self.class_names.get(cls, str(cls))
                self.get_logger().info(
                    f"[{view_name}] Detected: {label} ({score:.2f}) "
                    f"@ (x1={int(x1)}, y1={int(y1)}, x2={int(x2)}, y2={int(y2)})"
                )

                # Overlay
                if annotated is not None:
                    label = self.class_names.get(cls, str(cls))
                    x1i, y1i, x2i, y2i = int(x1), int(y1), int(x2), int(y2)
                    cv2.rectangle(annotated, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)
                    cv2.putText(annotated, f"{label} {score:.2f}",
                                (x1i, max(0, y1i - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.imshow("annotated", annotated)
                    cv2.waitKey(1)

        # Publish detections (immer)
        pub_d = self.pub_dets.get(view_name)
        if pub_d is not None:
            self.pub_dets[view_name].publish(det_array)

        # Publish annotated NUR wenn >=1 Box
        if self.publish_annotated and annotated is not None and view_name in self.pub_imgs and num_boxes > 0:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out_msg.header = msg.header
            self.pub_imgs[view_name].publish(out_msg)
            
            # time_elapsed = time.time() - start_time
            # self.get_logger().info(f"Dauer YOLO: {time_elapsed}")
            # self.get_logger().info(f"End Timestamp Bild {self.counter}: {time.time()}")
            # self.counter += 1

        # Log
        dt = time.time() - t0
        if dt > 0:
            fps = 1.0 / dt
            self.get_logger().debug(f"[{view_name}] boxes={num_boxes}  {fps:.1f} FPS ({dt*1000:.1f} ms)")
        


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Programm wurde mit Strg+C beendet.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"Shutdown bereits durchgeführt oder fehlgeschlagen: {e}")


if __name__ == "__main__":
    main()
