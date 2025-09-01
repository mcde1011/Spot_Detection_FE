import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from vision_msgs.msg import Pose2D
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Point

# BB von Feuerlöscher und Schild haben feste Größe in Occ Map

# Nehme an ID von Feuerlöscher ist 1 und von Schild ist 2 

class TransformToMapNode(Node):
    def __init__(self):
        super().__init__('BB_subscriber')
    
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.marker_array = MarkerArray()
        self.marker = Marker()

        self.marker_pub = self.create_publisher(MarkerArray, 'utils_rviz_visualization', 10)

        self.sub_front = self.create_subscription(
            Detection2DArray,
            '/detections/front',
            self.front_cb,
            10
        )
        # self.subscription  # prevent unused variable warning

        self.sub_front = self.create_subscription(
            Detection2DArray,
            '/detections/back',
            self.back_cb,
            10
        )

        self.sub_front = self.create_subscription(
            Detection2DArray,
            '/detections/left',
            self.left_cb,
            10
        )

        self.sub_front = self.create_subscription(
            Detection2DArray,
            '/detections/right',
            self.right_cb,
            10
        )
        self.sub_front = self.create_subscription(
            Detection2DArray,
            '/detections/up',
            self.up_cb,
            10
        )

        self.sub_front = self.create_subscription(
            Detection2DArray,
            '/detections/down',
            self.down_cb,
            10
        )

    def front_cb(self, msg):
        label = "front"
        detections_arr = msg
        success = self.drawObjInMap(label,detections_arr)
        if not success:
            self.get_logger().error('ERROR WHILE DRAWING OBJECTS FROM FRONT CAMERA')

    def back_cb(self, msg):
        label = "back"
        detections_arr = msg
        success = self.drawObjInMap(label,detections_arr)
        # if not success:
            # self.get_logger().error('ERROR WHILE DRAWING OBJECTS FROM BACK CAMERA')

    def left_cb(self, msg):
        label = "left"
        detections_arr = msg
        success = self.drawObjInMap(label,detections_arr)
        # if not success:
            # self.get_logger().error('ERROR WHILE DRAWING OBJECTS FROM LEFT CAMERA')
    
    def right_cb(self, msg):
        label = "right"
        detections_arr = msg
        success = self.drawObjInMap(label,detections_arr)
        # if not success:
            # self.get_logger().error('ERROR WHILE DRAWING OBJECTS FROM RIGHT CAMERA')

    def up_cb(self, msg):
        label = "up"
        detections_arr = msg
        success = self.drawObjInMap(label,detections_arr)
        # if not success:
            # self.get_logger().error('ERROR WHILE DRAWING OBJECTS FROM UP CAMERA')
        
    def down_cb(self, msg):
        label = "down"
        detections_arr = msg
        success = self.drawObjInMap(label, detections_arr)
        # if not success:
            # self.get_logger().error('ERROR WHILE DRAWING OBJECTS FROM DOWN CAMERA')


    def drawObjInMap(self, label, detections_arr):
        self.marker_array.markers.clear()
        point_camera_frame = PointStamped()
        i = 0
        for detection in detections_arr.detections:
            # print("ID: ", detection.results[0].hypothesis.class_id, flush=True)
            # print("center: ", detection.bbox.center, " X: ", detection.bbox.size_x, " Y: ", detection.bbox.size_y)
            bbox = detection.bbox
            # voerst nur für Feuerlöscher
            dist_approximation = calcDistance(bbox)
            # print("dist_approximation:", dist_approximation, " label: ", label)
            if label != "up" and label != "down":
                # calculate angle to object by using camera pixels (FoV = 90°, 800x800 Pixel)
                angle_in_image = -(0.1125 * (bbox.center.position.x - 400) * np.pi) / 180
                point_camera_frame = getPose(label, dist_approximation, angle_in_image)
            else:
                h = 2.275
                # delta from image middlepoint
                dx = bbox.center.position.x - 400
                dy = bbox.center.position.y - 400
                
                angle_in_image = np.arctan(dx/dy)
                vertical_angle = (0.1125 * np.sqrt(dx ** 2 + dy ** 2) * np.pi) /180
                dist_approximation = np.sin(vertical_angle * h)
                point_camera_frame = getPose(label, dist_approximation, angle_in_image)

            point_in_base_link = self.transformToBaselink(point_camera_frame)
            self.marker = createMarker(detection.results[i].hypothesis.class_id, point_in_base_link)
            self.marker.lifetime = rclpy.duration.Duration(seconds=9).to_msg()
            self.marker_array.markers.append(self.marker)
            i += 1

        self.marker_pub.publish(self.marker_array)
        
        if len(self.marker_array.markers) > 0:
            return True
        return False


    def transformToBaselink(self, camera_frame_point):

        # Transformiere in die Karte
        point_in_base_link = self.tf_buffer.transform(
            camera_frame_point, "hkaspot/base_link", timeout=rclpy.duration.Duration(seconds=1))
        return point_in_base_link


def calcDistance(bbox):
    # print("bbox size: ", bbox.size_y)
    dist_approximation = 0.1385566 + (2088723.861/(1+((bbox.size_y/0.0008264511) ** 1.174847)))
    return dist_approximation

def getPose(label, dist_approximation, angle_in_image):
    if label != "up" and label != "down":
        if label == "right":
            angle_in_image = angle_in_image - np.pi/2
        elif label == "back":
            angle_in_image = angle_in_image + np.pi
        elif label == "left":
            angle_in_image = angle_in_image + np.pi/2

    point_stamped = PointStamped()
    point_stamped.header.stamp = Time().to_msg()
    point_stamped.header.frame_id = "camera_link"
    point_stamped.point.x = np.cos(angle_in_image) * dist_approximation
    point_stamped.point.y = np.sin(angle_in_image) * dist_approximation
    point_stamped.point.z = 0.0

    if label == "up" and label == "down":
        buffer_x = point_stamped.point.x
        point_stamped.point.x = point_stamped.point.y
        point_stamped.point.y = buffer_x

    return point_stamped

def createMarker(id, point_in_map):
    marker = Marker()
    marker.header.frame_id = "camera_link"

    # Fire extinguisher
    if id == "1":
        # marker.ns = "circle"
        # marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025  # Linewidth
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        resolution = 20
        radius = 0.3
        for i in range(resolution+ 1):  # +1 für geschlossenen Kreis
            angle = 2 * np.pi * i / resolution
            p = Point()
            p.x = point_in_map.point.x + radius * np.cos(angle)
            p.y = point_in_map.point.y + radius * np.sin(angle)
            p.z = 0.0
            marker.points.append(p)
    
    # Fire extinguisher sign
    elif id == "2":
        marker.type = Marker.CUBE
        marker.pose.position = point_in_map.point
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

    return marker

def main(args=None):
    rclpy.init(args=args)
    node = TransformToMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
