import requests
import numpy as np
import time
import cv2
import py360convert
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from requests.auth import HTTPDigestAuth

# NEU
import os
from datetime import datetime
from pathlib import Path


class RicohPublisher(Node):
    def __init__(self):
        super().__init__('ricoh_theta_publisher')

        # Kameraeinstellungen
        # self.CAMERA_IP = "192.168.137.170"
        self.CAMERA_IP = "10.42.0.17"
        self.USERNAME = "THETAYR30101068"
        self.PASSWORD = "30101068"
        self.auth = HTTPDigestAuth(self.USERNAME, self.PASSWORD)

        # NEU: Zielordner zum Speichern der Bilder
        # Wird bei Bedarf automatisch erstellt.
        self.save_base_dir = Path("./ricoh_theta_views")  # anpassen falls gewünscht
        self.save_base_dir.mkdir(parents=True, exist_ok=True)

        # QoS passend für Kameradaten
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Sechs getrennte Topics anlegen
        self.view_pubs = {
            "front": self.create_publisher(Image, "/ricoh_theta/front", qos),
            "right": self.create_publisher(Image, "/ricoh_theta/right", qos),
            "back":  self.create_publisher(Image, "/ricoh_theta/back", qos),
            "left":  self.create_publisher(Image, "/ricoh_theta/left", qos),
            "up":    self.create_publisher(Image, "/ricoh_theta/up", qos),
            "down":  self.create_publisher(Image, "/ricoh_theta/down", qos),
        }

        # Bilder speichern = True
        self.save_enabled = False
        # Welche Richtungen sollen gespeichert werden
        self.save_views = {"front", "right", "back", "left", "up", "down"}

        self.bridge = CvBridge()

        # Alle N Sekunden Bild aufnehmen und veröffentlichen
        self.timer = self.create_timer(5.0, self.capture_and_publish)

        self.counter = 0

    def capture_and_publish(self):
        img = self.capture_image()
        if img is None:
            self.get_logger().error("Kein Bild erhalten.")
            return

        # 1) 360° -> 6 perspektivische Ansichten
        views = self.generate_views(img)

        # konsistenter Zeitstempel (lokal) für Dateinamen und ROS-Header
        now = datetime.now()
        # Beispiel: 2025-08-22_14-03-12.123456
        ts_str = now.strftime("%Y-%m-%d_%H-%M-%S.%f")

        # Unterordner pro Tag (optional, für Übersicht)
        day_dir = self.save_base_dir / now.strftime("%Y-%m-%d")
        day_dir.mkdir(parents=True, exist_ok=True)

        # NEU: gemeinsamer ROS-Zeitstempel
        stamp = self.get_clock().now().to_msg()

        # 2) publishen und (für 4 Richtungen) speichern
        for name, view_img in views.items():
            # Publish
            ros_img = self.bridge.cv2_to_imgmsg(view_img, encoding="bgr8")
            ros_img.header.stamp = stamp
            ros_img.header.frame_id = name
            self.view_pubs[name].publish(ros_img)

            # Lokal speichern (nur front/right/back/left)
            if name in self.save_views and self.save_enabled:
                # Dateiname: <timestamp>_<view>.jpg
                filename = f"{ts_str}_{name}.jpg"
                file_path = day_dir / filename

                # cv2.imwrite gibt True/False zurück
                ok = cv2.imwrite(str(file_path), view_img)
                if not ok:
                    self.get_logger().error(f"Speichern fehlgeschlagen: {file_path}")
                else:
                    # Optional: Log auf Debug-Level
                    self.get_logger().debug(f"Gespeichert: {file_path}")

    def capture_image(self):
        start_time = time.time()
        # Foto aufnehmen
        take_resp = requests.post(
            f"http://{self.CAMERA_IP}/osc/commands/execute",
            json={"name": "camera.takePicture"},
            auth=self.auth
        )
        if take_resp.status_code != 200:
            self.get_logger().error("Fehler beim Auslösen der Kamera.")
            return None

        # ID holen
        try:
            command_id = take_resp.json().get("id")
        except Exception:
            self.get_logger().error("Fehler: Keine ID in der Antwort.")
            return None
        if not command_id:
            self.get_logger().error("Keine gültige Command-ID erhalten.")
            return None

        # Warten bis fertig
        latest_file = None
        for _ in range(20):
            status_resp = requests.post(
                f"http://{self.CAMERA_IP}/osc/commands/status",
                json={"id": command_id},
                auth=self.auth
            )
            if status_resp.status_code == 200:
                data = status_resp.json()
                if data.get("state") == "done":
                    latest_file = data.get("results", {}).get("fileUrl")
                    break
            time.sleep(0.5)

        if not latest_file:
            self.get_logger().error("Aufnahme nicht abgeschlossen.")
            return None

        # Bild herunterladen
        img_resp = requests.get(latest_file, auth=self.auth)
        if img_resp.status_code != 200:
            self.get_logger().error("Fehler beim Herunterladen.")
            return None
        
        time_elapsed = time.time() - start_time
        self.get_logger().info(f"Komplett Bildaufnahme: {time_elapsed}")

        img_array = np.asarray(bytearray(img_resp.content), dtype=np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        return img

    def generate_views(self, img):
        fov = 90
        out_size = (800, 800)
        directions = {
            "front": (0, 0),
            "right": (90, 0),
            "back": (180, 0),
            "left": (-90, 0),
            "up": (0, 90),
            "down": (0, -90),
        }
        views = {}
        for name, (yaw, pitch) in directions.items():
            persp_img = py360convert.e2p(
                img,
                fov_deg=fov,
                u_deg=yaw,
                v_deg=pitch,
                out_hw=out_size
            )
            views[name] = persp_img
        return views


def main(args=None):
    rclpy.init(args=args)
    node = RicohPublisher()
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
