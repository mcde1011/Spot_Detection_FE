import rclpy
import numpy as np
import os
import yaml
from rclpy.node import Node
from rclpy.time import Time
from vision_msgs.msg import Pose2D
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Point
from ament_index_python.packages import get_package_share_directory

class TransformToMapNode(Node):
    def __init__(self):
        super().__init__('BB_subscriber')
    
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.marker_array = MarkerArray()
        self.marker = Marker()
        self.semantic_data = {}
        
        # get path to semantic map and load it 
        self.declare_parameter("semantic_map_file", "")
        self.smap_filename = self.get_parameter("semantic_map_file").get_parameter_value().string_value
        if not self.smap_filename:
            self.get_logger().error("No semantic_map_file parameter provided")
        self.load_semantic_map()
        self.timer = self.create_timer(5.0, self.publish_semantic_map)

        # read params.yaml
        self.declare_parameter("camera_frame", "camera_link")
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.declare_parameter("base_frame", "hkaspot/base_link")
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.declare_parameter("position_tolerance", 0.4)
        self.position_tolerance = self.get_parameter("position_tolerance").get_parameter_value().double_value

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

    ####################################################################
    ###                     SEMANTIC MAP                             ###
    ####################################################################

    def load_semantic_map(self):
        """load YAML-file with graceful handling for empty files"""
        try:
            if not os.path.exists(self.smap_filename):
                self.get_logger().warn(f"YAML file does not exist: {self.smap_filename}")
                return
                
            with open(self.smap_filename, "r", encoding='utf-8') as f:
                loaded_data = yaml.safe_load(f)
                
            # check if file is empty 
            if loaded_data is None:
                self.get_logger().warn("YAML file is empty - using default empty configuration")
                self.semantic_data = {}
            elif isinstance(loaded_data, dict):
                self.semantic_data = loaded_data
                self.get_logger().info(f"Successfully loaded semantic map from: {self.smap_filename}")
            else:
                self.get_logger().warn(f"YAML file contains unexpected data type: {type(loaded_data)}")
                self.semantic_data = {}
                
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file: {e}")
            self.semantic_data = {}
        except Exception as e:
            self.get_logger().error(f"Error loading YAML file: {e}")
            self.semantic_data = {}
        
    # def get_objects(self):
    #     # semantic_data can't be none
    #     return self.semantic_data.get("objects", [])
    
    # def get_semantic_value(self, key, default=None):
    #     return self.semantic_data.get(key, default)

    def addObjToYaml(self, point_in_base_link, obj_class):
        """Add object to semantic map"""
        point_in_map = self.transformToMap(point_in_base_link)
        new_position = np.array([
            float(point_in_map.point.x),
            float(point_in_map.point.y),
            float(point_in_map.point.z)
        ])

        if 'objects' not in self.semantic_data:
            self.semantic_data['objects'] = []

        updated = False
        for obj in self.semantic_data['objects']:
            if obj.get('object_type') == obj_class:
                old_pos = np.array(obj.get('position', [0, 0, 0]))
                dist = np.linalg.norm(new_position - old_pos)
                if dist <= self.position_tolerance:
                    # Overwrite object
                    obj['position'] = new_position.tolist()
                    updated = True
                    break

        if not updated:
            # Create new entry
            object_id = self.generate_next_id(obj_class)
            new_object = {
                'object_type': obj_class,
                'id': object_id,
                'position': new_position.tolist()
            }
            self.semantic_data['objects'].append(new_object)
            self.get_logger().info(
                f"Added new {obj_class} with id {object_id}"
            )
        return True    

    def generate_next_id(self, obj_class):
        count = len([obj for obj in self.semantic_data.get('objects', []) if obj.get('object_type') == obj_class])
        next_number = count + 1
        
        # Format: object_type_XXX
        return f"{obj_class}_{next_number:03d}"

    def save_semantic_map(self):
        try:
            with open(self.smap_filename, 'w', encoding='utf-8') as f:
                yaml.dump(self.semantic_data, f, default_flow_style=False, allow_unicode=True)
            # self.get_logger().info(f"Saved semantic map to {self.smap_filename}")
            # return True
        except Exception as e:
            self.get_logger().error(f"Could not save semantic map: {e}")
            return False
        
    def publish_semantic_map(self):
        """Visualize all objects from YAML as markers in RViz"""
        self.marker_array.markers.clear()

        if 'objects' not in self.semantic_data:
            return

        for obj in self.semantic_data['objects']:
            obj_class = obj['object_type']
            pos = obj['position']
            id = obj['id']

            point_stamped = PointStamped()
            point_stamped.header.stamp = Time().to_msg()
            point_stamped.header.frame_id = "map"
            point_stamped.point.x = pos[0]
            point_stamped.point.y = pos[1]
            point_stamped.point.z = pos[2]

            marker = createMarker(obj_class, id, "map", point_stamped)
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            self.marker_array.markers.append(marker)

            text_marker = createTextMarker(id, "map", point_stamped)
            self.marker_array.markers.append(text_marker)

        if self.marker_array.markers:
            self.marker_pub.publish(self.marker_array)

    ####################################################################
    ###                       CALLBACKS                              ###
    ####################################################################

    def front_cb(self, msg):
        label = "front"
        detections_arr = msg
        success = self.drawObjInMap(label,detections_arr)
        # if not success:
        #     self.get_logger().error('ERROR WHILE DRAWING OBJECTS FROM FRONT CAMERA')

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

    ####################################################################
    ###                     DRAW OBJECT IN MAP                       ###
    ####################################################################

    def drawObjInMap(self, label, detections_arr):
        self.marker_array.markers.clear()
        point_camera_frame = PointStamped()
        i = 0
        for detection in detections_arr.detections:
            if not detection.results:
                self.get_logger().warn("Detection without results – skipping")
                continue

            bbox = detection.bbox
            obj_class = idToString(detection.results[i].hypothesis.class_id)
            obj_id = obj_class + str(i)
            # voerst nur für Feuerlöscher ##################################################################################
            dist_approximation = calcDistance(bbox)
            if label != "up" and label != "down":
                # calculate angle to object by using camera pixels (FoV = 90°, 800x800 Pixel)
                angle_in_image = -(0.1125 * (bbox.center.position.x - 400) * np.pi) / 180
                point_camera_frame = self.getPose(label, dist_approximation, angle_in_image)
            else:
                h = 2.275
                # delta from image middlepoint
                dx = bbox.center.position.x - 400
                dy = bbox.center.position.y - 400
                
                angle_in_image = np.arctan(dx/dy)
                vertical_angle = (0.1125 * np.sqrt(dx ** 2 + dy ** 2) * np.pi) /180
                dist_approximation = np.sin(vertical_angle * h)
                point_camera_frame = self.getPose(label, dist_approximation, angle_in_image)

            point_in_base_link = self.transformToBaseFrame(point_camera_frame)
            self.marker = createMarker(obj_class, obj_id, self.base_frame, point_in_base_link)
            self.marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
            self.addObjToYaml(point_in_base_link, obj_class)
            self.marker_array.markers.append(self.marker)
            i += 1

        self.marker_pub.publish(self.marker_array)
        self.save_semantic_map()
        if len(self.marker_array.markers) > 0:
            return True
        return False

    def transformToBaseFrame(self, camera_frame_point):
        point_in_base_link = self.tf_buffer.transform(
            camera_frame_point, self.base_frame, timeout=rclpy.duration.Duration(seconds=1))
        return point_in_base_link

    def transformToMap(self, base_link_point):
        point_in_map = self.tf_buffer.transform(
            base_link_point, "map", timeout=rclpy.duration.Duration(seconds=1))
        return point_in_map

    def getPose(self, label, dist_approximation, angle_in_image):
        if label != "up" and label != "down":
            if label == "right":
                angle_in_image = angle_in_image - np.pi/2
            elif label == "back":
                angle_in_image = angle_in_image + np.pi
            elif label == "left":
                angle_in_image = angle_in_image + np.pi/2

        point_stamped = PointStamped()
        point_stamped.header.stamp = Time().to_msg()
        point_stamped.header.frame_id = self.camera_frame
        point_stamped.point.x = np.cos(angle_in_image) * dist_approximation
        point_stamped.point.y = np.sin(angle_in_image) * dist_approximation
        point_stamped.point.z = 0.0

        if label == "up" and label == "down":
            buffer_x = point_stamped.point.x
            point_stamped.point.x = point_stamped.point.y
            point_stamped.point.y = buffer_x

        return point_stamped

def idToString(obj_id):
    if obj_id == "1":
        return "fire_extinguisher"
    elif obj_id == "2":
        return "fire_extinguisher_sign"
    else:
        return "unknown class"

def calcDistance(bbox):
    """Distance approximation with symmetrical sigmoidal"""
    dist_approximation = 0.1385566 + (2088723.861/(1+((bbox.size_y/0.0008264511) ** 1.174847)))
    return dist_approximation

def createMarker(obj_class, obj_id, frame_id, point_in_map):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = obj_id
    marker.id = hash(obj_id) % (2**31 - 1) if obj_id else 0

    if obj_class == "fire_extinguisher":
        marker.id = 1
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
        for i in range(resolution+ 1):  # +1 to close the circle
            angle = 2 * np.pi * i / resolution
            p = Point()
            p.x = point_in_map.point.x + radius * np.cos(angle)
            p.y = point_in_map.point.y + radius * np.sin(angle)
            p.z = 0.0
            marker.points.append(p)

    elif obj_class == "fire_extinguisher_sign":
        marker.id = 2
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

def createTextMarker(obj_id, frame_id, point_in_map):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "labels"
    marker.id = hash("text_" + str(obj_id)) % (2**31 - 1)

    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD

    marker.pose.position.x = point_in_map.point.x
    marker.pose.position.y = point_in_map.point.y
    marker.pose.position.z = point_in_map.point.z + 0.05

    marker.scale.z = 0.15  # Fontsize in Meters
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    marker.text = str(obj_id)

    return marker

def main(args=None):
    rclpy.init(args=args)
    node = TransformToMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
