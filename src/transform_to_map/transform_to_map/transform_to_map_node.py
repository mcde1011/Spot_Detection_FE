import rclpy
import numpy as np
from rclpy.node import Node
from vision_msgs.msg import Pose2D
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import Buffer, TransformListener
import geometry_msgs.msg

# BB von Feuerlöscher und Schild haben feste Größe in Occ Map

# Nehme an ID von Feuerlöscher ist 1 und von Schild ist 2 

class BoundingBoxSubscriber(Node):
    def __init__(self):

        self.marker_array = MarkerArray()
        self.marker = Marker()

        super().__init__('BB_subscriber')

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
        success = drawObjInMap(label,detections_arr)
        # if not success:
            # self.get_logger().error('ERROR WHILE DRAWING OBJECTS FROM FRONT CAMERA')

    def back_cb(self, msg):
        label = "back"
        detections_arr = msg
        success = drawObjInMap(label,detections_arr)
        # if not success:
            # self.get_logger().error('ERROR WHILE DRAWING OBJECTS FROM BACK CAMERA')

    def left_cb(self, msg):
        label = "left"
        detections_arr = msg
        success = drawObjInMap(label,detections_arr)
        # if not success:
            # self.get_logger().error('ERROR WHILE DRAWING OBJECTS FROM LEFT CAMERA')
    
    def right_cb(self, msg):
        label = "right"
        detections_arr = msg
        success = drawObjInMap(label,detections_arr)
        # if not success:
            # self.get_logger().error('ERROR WHILE DRAWING OBJECTS FROM RIGHT CAMERA')

    def up_cb(self, msg):
        label = "up"
        detections_arr = msg
        success = drawObjInMap(label,detections_arr)
        # if not success:
            # self.get_logger().error('ERROR WHILE DRAWING OBJECTS FROM UP CAMERA')
        
    def down_cb(self, msg):
        label = "down"
        detections_arr = msg
        success = drawObjInMap(label,detections_arr)
        # if not success:
            # self.get_logger().error('ERROR WHILE DRAWING OBJECTS FROM DOWN CAMERA')


def drawObjInMap(label, detections_arr):
    for detection in detections_arr.detections:
        bbox = detection.bbox
        # voerst nur für Feuerlöscher
        dist_approximation = calcDistance(bbox)
        # print("dist_approximation:", dist_approximation, " label: ", label)
        if label != "oben" and label != "unten":
            # calculate angle to object by using camera pixels (FoV = 90°, 800x800 Pixel)
            angle_in_image = (0.1125 * (bbox.center.position.x - 400) * np.pi) / 180
            point_camera_frame = getPose(label, dist_approximation, angle_in_image)
            print("(", point_camera_frame._point._x, "/", point_camera_frame._point._y, ")")
            # point_in_map = transformToBaselink(point_camera_frame)
            # marker = createMarker(point_in_map)


def calcDistance(bbox):
    # print("bbox size: ", bbox.size_y)
    dist_approximation = 0.1385566 + (2088723.861/(1+((bbox.size_y/0.0008264511) ** 1.174847)))
    return dist_approximation

def getPose(label, dist_approximation, angle_in_image):
    abs_angle = 0.0
    if label == "front":
        abs_angle = angle_in_image
    elif label == "right":
        abs_angle = angle_in_image - np.pi/4
    elif label == "back":
        abs_angle = angle_in_image - np.pi/2
    elif label == "left":
        abs_angle = angle_in_image + np.pi/4


    point_stamped = geometry_msgs.msg.PointStamped()
    point_stamped.header.frame_id = "camera_frame"
    point_stamped.point.x = np.sin(abs_angle) * dist_approximation
    point_stamped.point.y = np.cos(abs_angle) * dist_approximation
    point_stamped.point.z = 0.0

    return point_stamped

def transformToBaselink(camera_frame_point):
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, Node)

    # Transformiere in die Karte
    point_in_map = tf_buffer.transform(camera_frame_point, "map", timeout=rclpy.duration.Duration(seconds=1))
    return point_in_map

def createMarker(point_in_map, id):
    
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = Marker.SPHERE
    marker.pose.position = point_in_map.point
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

def main(args=None):
    rclpy.init(args=args)
    node = BoundingBoxSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
