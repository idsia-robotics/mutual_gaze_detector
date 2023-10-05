#!/usr/bin/env python3

import rclpy.node
from rclpy.time import Time
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException, Duration
from scipy.spatial.transform import Rotation

from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import Point, Pose, TransformStamped, Quaternion, PoseArray, Transform
from sensor_msgs.msg import Image, CameraInfo

from azure_kinect_ros_msgs.msg import MarkerArrayStamped
from users_landmarks_msgs.msg import MultipleUsersLandmarks, SingleUserLandmarks

import numpy as np
import cv2
import os
from scipy.spatial.transform import Rotation

from users_landmarks_utils.calibration_utils import Calibration
from users_landmarks_utils.kinect_bt_utils import body_joints_info, get_body_segments_joint_numbers
from .FaceLandmarksExtractor.FacesLandmarksExtractor import FacesLandmarksExtractor
from .FaceLandmarksExtractor.Face import Face

class FaceLandmarksDetectionNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("face_landmarks_node")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('debug_node', False),
                ('maximum_number_of_bodies', 3),
                ('draw_landmarks', False),
                ('face_landmarks_extraction_version', "simple"),
                ('rectified_image_input', False)
            ])
        
        # Parameters
        self.debug_node = self.get_parameter('debug_node').get_parameter_value().bool_value
        self.get_logger().info("debug_node: {}".format(self.debug_node))
        self.maximum_number_of_bodies = self.get_parameter('maximum_number_of_bodies').get_parameter_value().integer_value
        self.get_logger().info("maximum_number_of_bodies: {}".format(self.maximum_number_of_bodies))
        self.face_landmarks_extraction_version = self.get_parameter('face_landmarks_extraction_version').get_parameter_value().string_value
        self.get_logger().info("face_landmarks_extraction_version: {}".format(self.face_landmarks_extraction_version))
        self.draw_landmarks_debug = self.get_parameter('draw_landmarks').get_parameter_value().bool_value
        self.get_logger().info("draw_landmarks: {}".format(self.draw_landmarks_debug))
        self.rectified_image_input = self.get_parameter('rectified_image_input').get_parameter_value().bool_value
        self.get_logger().info("rectified_image_input: {}".format(self.rectified_image_input))

        qos = rclpy.qos.QoSProfile(
            depth=1,
        )
        
        # Rates for input topics
        self.imu_callback_rate = 10
        
        # Subscribers
        self.camera_info_sub = self.create_subscription(CameraInfo, "/depth_to_rgb/camera_info", self.camera_info_callback, 1)

        image_topic = None
        if self.rectified_image_input:
            image_topic = "/rgb/image_rect_raw"
        else:
            image_topic = "/rgb/image_raw"
        self.rgb_image_sub = Subscriber(self, Image, image_topic, qos_profile=qos)
        self.kinect_bt_sub = Subscriber(self, MarkerArrayStamped, "/body_tracking_data", qos_profile=qos)
        self.synchronizer = ApproximateTimeSynchronizer([self.rgb_image_sub, self.kinect_bt_sub],
                                                        queue_size=10, slop=0.1)
        self.synchronizer.registerCallback(self.synchronized_data_callback)

        # Publishers
        self.users_landmarks_pub = self.create_publisher(MultipleUsersLandmarks, "~/users_landmarks", 1)
        self.full_landmarks_debug_image_pub = self.create_publisher(Image, "~/full_landmarks_debug_image", 1)
        self.face_landmarks_closeup_debug_image_pub = self.create_publisher(Image, "~/face_landmarks_closeup_debug_image", 1)
        self.pose_array_debug_pub = self.create_publisher(PoseArray, "~/pose_array_debug", 1)
        
        # Transform listeners
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Arguments of node
        self.calibration = None
        self.cv_bridge = CvBridge()
        self.body_joints_info = body_joints_info
        self.body_joints_number = int(len(self.body_joints_info))
        
        face_landmarks_model_path = os.path.join(get_package_share_directory('users_landmarks_tracking'), 'config/face_landmarker.task')
        
        self.faces_landmarks_extractor = FacesLandmarksExtractor(maximum_number_of_bodies=self.maximum_number_of_bodies, 
                                                                 mediapipe_image_shape=(512, 512),
                                                                 static_image_mode=False,
                                                                 version=self.face_landmarks_extraction_version,
                                                                 model_path=face_landmarks_model_path,
                                                                 logger = self.get_logger())
        
        self.face = Face()
        
        self.get_logger().info("Node correctly initialized.")
    
    def camera_info_callback(self, msg):        
        self.get_logger().info(f"Got Calibration")
        self.calibration = Calibration(camera_info = msg, rectified_image_input = self.rectified_image_input)
        
        # Depth to RGB extrinsics 
        depth_to_rgb_tf = []
        try:
            depth_to_rgb_tf = self.tf_buffer.lookup_transform('rgb_camera_link', 'depth_camera_link', rclpy.time.Time(), timeout=Duration(seconds=1.0))
            self.get_logger().info(f'Got transform from depth_camera_link to rgb_camera_link.')
        except TransformException as ex:
            self.get_logger().info(f'Could not transform depth_camera_link to rgb_camera_link: {ex}')
            depth_to_rgb_tf = TransformStamped()
            return   
        
        # Logging
        if self.debug_node:
            self.get_logger().info(f"Calibration:\n{msg}")
            self.get_logger().info(f'Transform from depth_camera_link to rgb_camera_link:\n{depth_to_rgb_tf}')
        
        translation = np.array([depth_to_rgb_tf.transform.translation.x, 
                                depth_to_rgb_tf.transform.translation.y,
                                depth_to_rgb_tf.transform.translation.z])
        quat = [depth_to_rgb_tf.transform.rotation.x, 
                depth_to_rgb_tf.transform.rotation.y, 
                depth_to_rgb_tf.transform.rotation.z, 
                depth_to_rgb_tf.transform.rotation.w]
        rotation_matrix = Rotation.from_quat(quat).as_matrix()
        
        self.calibration.set_depth_to_rgb_extrinsics(translation, rotation_matrix)
        
        # Unsubscribe from camera_info topic
        if self.destroy_subscription(self.camera_info_sub):
            self.get_logger().info("Unsubscribed from camera_info topic.")
        else:
            self.get_logger().info("Failed to unsubscribe from camera_info topic.")
        
    def synchronized_data_callback(self, rgb_msg, body_tracking_msg):
        # Check if calibration received
        if self.calibration:
            image_raw = self.cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')
            
            # Check that some bodies are tracked
            if len(body_tracking_msg.markers) != 0:
                # Retrieve depth to rgb gravity aligned transform
                to_frame_rel = "gravity_aligned_rgb_camera_link"
                from_frame_rel = "depth_camera_link"

                try:
                    self.depth_to_rgb_gravity_aligned_tf = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                    return                
                
                self.number_of_bodies = len(body_tracking_msg.markers) // self.body_joints_number
                
                # Get bodies indeces
                bodies_index = [int(body_tracking_msg.markers[first_joint_marker_idx].id//100) for first_joint_marker_idx in range(0, len(body_tracking_msg.markers), self.body_joints_number)]
                                
                # Get 3D points of joints
                joints_3d_position = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for m in body_tracking_msg.markers])
                joints_3d_position *= 1000. # m to mm
                
                # Split joints 3D positions by body
                joints_3d_position = np.split(joints_3d_position, self.number_of_bodies, axis=0)
                
                # Calculate possible face ROI for each body
                faces_roi = self.calculate_faces_roi(bodies_joints_3d_position = joints_3d_position, calibration = self.calibration)
                
                # Create index to body joints touples
                bodies_index_points_3d_roi = list(zip(bodies_index, joints_3d_position, faces_roi))
                               
                if self.number_of_bodies>self.maximum_number_of_bodies:
                    # Get the closest bodies
                    bodies_index_points_3d_roi = sorted(bodies_index_points_3d_roi, key=lambda x: x[1][body_joints_info["HEAD"]["number"]])
                    bodies_index_points_3d_roi = bodies_index_points_3d_roi[:self.maximum_number_of_bodies]
                    
                # Order bodies by body index
                bodies_index_points_3d_roi = sorted(bodies_index_points_3d_roi, key=lambda x: x[0])
                               
                # MediaPipe works with RGB images                
                msg_timestamp_milliseconds = Time.from_msg(rgb_msg.header.stamp).nanoseconds // 1000000

                bodies_indeces_face_landmarks = self.faces_landmarks_extractor.process(input_image= cv2.cvtColor(image_raw, cv2.COLOR_BGR2RGB),
                                                                                       frame_timestamp_ms=msg_timestamp_milliseconds, 
                                                                                       bodies_index_points_3d_roi = bodies_index_points_3d_roi)

                if self.debug_node:
                    if self.draw_landmarks_debug:
                        # Body coordiantes always available here 
                        image_raw = self.draw_pose_landmarks(bodies_index_points_3d_roi=bodies_index_points_3d_roi, input_image=image_raw)
                        image_raw = self.draw_face_landmarks(bodies_indeces_face_landmarks=bodies_indeces_face_landmarks, input_image=image_raw)                     
                    
                    # Closeup face debug image of only the closest body
                    face_roi = bodies_index_points_3d_roi[0][2]
                    face_landmarks_close_up = cv2.resize(image_raw[face_roi[0][1]:face_roi[1][1], face_roi[0][0]:face_roi[1][0]], (512, 512), interpolation= cv2.INTER_LINEAR)
                    
                    self.face_landmarks_closeup_debug_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(face_landmarks_close_up[:,:,:-1], encoding='bgr8'))
                
                # Hack to fix bug in body tracking message code on the driver side 
                body_tracking_msg.header = body_tracking_msg.markers[0].header
                    
                # Publish user landmarks
                self.publish_users_landmarks(body_tracking_msg = body_tracking_msg, 
                                             transform = self.depth_to_rgb_gravity_aligned_tf.transform,
                                             bodies_indeces_face_landmarks = bodies_indeces_face_landmarks)
                    
            if self.debug_node:
                if rgb_msg.encoding == 'bgra8':
                    # Discard alpha channel for visualization
                    self.full_landmarks_debug_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(image_raw[:,:,:-1], encoding='bgr8'))
                if rgb_msg.encoding == 'bgr8':
                    self.full_landmarks_debug_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(image_raw, encoding='bgr8'))
                    
    def calculate_faces_roi(self, bodies_joints_3d_position: list, calibration: Calibration) -> list:
        # Average dimensions of a human head are available here: https://en.wikipedia.org/wiki/Human_head
        # Average human head width 99th percentile man: 16.5 cm
        # Average human head height 99th percentile man: 25.5 cm
        # MediaPipe needs 25% more space around the face on each side,
        # An image which projects to 50 cm i n the real world should be enough 
        
        roi_world_dimension = 500 # mm        
        faces_roi = []
                
        for body_joints_3d_position in bodies_joints_3d_position:
            # Center of the head will be the average of the 3D positions of the ear joints
            head_center = (body_joints_3d_position[body_joints_info["EAR_LEFT"]["number"]] +  \
                          body_joints_3d_position[body_joints_info["EAR_RIGHT"]["number"]])/2.0
            head_center_2d = calibration.world_to_rgb_image(head_center)
            
            head_top = head_center + np.array([0, roi_world_dimension/2.0, 0])
            head_top_2d = calibration.world_to_rgb_image(head_top)
            
            roi_half_dim = int((head_top_2d - head_center_2d)[1])
                
            roi_top_left =     (int(head_center_2d[0]-roi_half_dim), int(head_center_2d[1]-roi_half_dim))
            roi_bottom_right = (int(head_center_2d[0]+roi_half_dim), int(head_center_2d[1]+roi_half_dim))
            
            # Check if ROI is inside the image
            if roi_top_left[0] < 0: # Shift ROI to the right
                roi_top_left     = (0, roi_top_left[1])
                roi_bottom_right = (2*roi_half_dim, roi_bottom_right[1])
            
            if roi_top_left[1] < 0: # Shift ROI down
                roi_top_left     = (roi_top_left[0], 0)
                roi_bottom_right = (roi_bottom_right[0], 2*roi_half_dim)
                
            if roi_bottom_right[0] > calibration.width: # Shift ROI to the left
                roi_top_left     = (calibration.width - 2*roi_half_dim, roi_top_left[1])
                roi_bottom_right = (calibration.width, roi_bottom_right[1])
                
            if roi_bottom_right[1] > calibration.height: # Shift ROI up
                roi_top_left     = (roi_top_left[0], calibration.height - 2*roi_half_dim)
                roi_bottom_right = (roi_bottom_right[0], calibration.height)

            faces_roi.append((roi_top_left, roi_bottom_right))
            
            # if self.debug_node:
            #     self.get_logger().info(f"head_center: {head_center}")
            #     self.get_logger().info(f"roi_half_dim: {roi_half_dim}")
            #     self.get_logger().info(f"faces_roi[{len(faces_roi)-1}]: {faces_roi[-1]}")
            
        return faces_roi

    def draw_pose_landmarks(self, bodies_index_points_3d_roi: list, input_image: np.ndarray):
        landmarked_image = input_image
        
        for body_id, body_points_3d, roi in bodies_index_points_3d_roi:
            # Transform 3D points from 3D to RGB frame
            joints_2d_position = self.calibration.world_to_rgb_image(body_points_3d)
            
            # Draw body skeleton and joints in green color
            for body_segment in get_body_segments_joint_numbers():
                for i in range(len(body_segment)-1):
                    landmarked_image = cv2.line(landmarked_image, 
                                    joints_2d_position[body_segment[i]], 
                                    joints_2d_position[body_segment[i+1]], 
                                    [0, 255, 0], 2)
            for joint_2d_position in joints_2d_position:
                landmarked_image = cv2.circle(landmarked_image, joint_2d_position, 3, [0, 255, 0], 3)
            
            # Draw face ROI box in red color
            cv2.rectangle(landmarked_image, roi[0], roi[1], [0, 0, 255], 3)
        
        return landmarked_image
    
    def draw_face_landmarks(self, bodies_indeces_face_landmarks: list, input_image: np.ndarray):
        landmarked_image = input_image
        image_dimension = np.array([input_image.shape[1], input_image.shape[0]]) 

        for body_id, face_landmarks_normalized, detection_result in bodies_indeces_face_landmarks:
            if detection_result:
                landmarked_min = np.min(face_landmarks_normalized, axis=0)
                landmarked_max = np.max(face_landmarks_normalized, axis=0)
                landmarks_dimension = max(int(round((landmarked_max[0] - landmarked_min[0])/0.05)), 1)
                
                face_landmarks  = np.array([[lm[0] * image_dimension[0],
                                            lm[1] * image_dimension[1],
                                            lm[2] * image_dimension[0]] for lm in face_landmarks_normalized])
                self.face.draw_landmarks(landmarked_image, face_landmarks, landmarks_dimension, (0,180,255),-1, link=False)   
                
                self.face.draw_landmarks(landmarked_image, face_landmarks[self.face.left_eyelids_indices]    ,landmarks_dimension,(0,0,255),1, link=True)
                self.face.draw_landmarks(landmarked_image, face_landmarks[self.face.left_eye_contour_indices],landmarks_dimension,(0,255,0),1, link=True)
                pos = face_landmarks[self.face.left_eye_center_index]
                cv2.circle(landmarked_image,(int(pos[0]), int(pos[1])), landmarks_dimension, (0,255,0),-1)

                self.face.draw_landmarks(landmarked_image, face_landmarks[self.face.right_eyelids_indices]    ,landmarks_dimension,(0,0,255),1, link=True)
                self.face.draw_landmarks(landmarked_image, face_landmarks[self.face.right_eye_contour_indices],landmarks_dimension,(0,255,0),1, link=True) 
                pos = face_landmarks[self.face.right_eye_center_index] 
                cv2.circle(landmarked_image,(int(pos[0]), int(pos[1])), landmarks_dimension, (0,255,0),-1)

        return landmarked_image
    
    def publish_users_landmarks(self, body_tracking_msg: MarkerArrayStamped, transform: Transform, bodies_indeces_face_landmarks: list):
        users_landmarks = MultipleUsersLandmarks() 
        users_landmarks.header = body_tracking_msg.header
                
        for index in range(0, self.number_of_bodies):
            user_landmarks = SingleUserLandmarks()
            user_landmarks.header = body_tracking_msg.header
            user_landmarks.header.frame_id = "gravity_aligned_rgb_camera_link"
            user_landmarks.body_id = bodies_indeces_face_landmarks[index][0]

            user_landmarks.body_landmarks = [self.rotate_pose_with_transform(m.pose, transform) for m in body_tracking_msg.markers[index*self.body_joints_number:(index+1)*self.body_joints_number]]
            
            if self.debug_node:
                pose_msg = PoseArray()
                pose_msg.header = user_landmarks.header
                pose_msg.poses = user_landmarks.body_landmarks
                self.pose_array_debug_pub.publish(pose_msg)

            user_landmarks.face_landmarks = [Point(x=m[0], y=m[1], z=m[2]) for m in bodies_indeces_face_landmarks[index][1]]
            
            users_landmarks.users.append(user_landmarks)
            
        self.users_landmarks_pub.publish(users_landmarks)
    
    def get_homogeneous_transform_matrix(self, transform):
        homogeneous_matrix = np.eye(4)
        
        quaternion = np.array([transform.rotation.x, 
                               transform.rotation.y, 
                               transform.rotation.z,
                               transform.rotation.w], dtype=np.float64)
        homogeneous_matrix[0:3, 0:3] = Rotation.from_quat(quaternion).as_matrix()
        
        translation = np.array([transform.translation.x,
                                transform.translation.y,
                                transform.translation.z], dtype=np.float64) 
        homogeneous_matrix[:3, 3] = translation
                
        return homogeneous_matrix
    
    def get_pose_homogeneous_matrix(self, pose):
        homogeneous_matrix = np.eye(4)
        quaternion = np.array([pose.orientation.x, 
                               pose.orientation.y, 
                               pose.orientation.z,
                               pose.orientation.w], dtype=np.float64)
        homogeneous_matrix[0:3, 0:3] = Rotation.from_quat(quaternion).as_matrix()
        
        translation = np.array([pose.position.x,
                                pose.position.y,
                                pose.position.z], dtype=np.float64)
        homogeneous_matrix[:3, 3] = translation
        return homogeneous_matrix
        
    def rotate_pose_with_transform(self, pose, transform):
        transformed_pose = Pose()
        
        transform_matrix = self.get_homogeneous_transform_matrix(transform)
        pose_homogeneous_matrix = self.get_pose_homogeneous_matrix(pose)
        transformed_pose_homogeneous_matrix = np.dot(transform_matrix, pose_homogeneous_matrix)

        transformed_pose.position = Point(x = transformed_pose_homogeneous_matrix[0, 3],
                                          y = transformed_pose_homogeneous_matrix[1, 3],
                                          z = transformed_pose_homogeneous_matrix[2, 3])
                
        transformed_quaternion = Rotation.from_matrix(transformed_pose_homogeneous_matrix[:3, :3]).as_quat()
        transformed_pose.orientation = Quaternion(x = transformed_quaternion[1],
                                                  y = transformed_quaternion[2],
                                                  z = transformed_quaternion[3],
                                                  w = transformed_quaternion[0])
        return transformed_pose
        
def main(args=None):
    rclpy.init(args=args)
    node = FaceLandmarksDetectionNode()
    
    rclpy.spin(node)
    
    # On shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()