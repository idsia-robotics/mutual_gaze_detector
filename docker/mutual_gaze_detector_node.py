#!/usr/bin/env python3

import rclpy.node
from rclpy.time import Time
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException, Duration

from message_filters import Subscriber
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, TransformStamped, Quaternion, PoseArray, Transform
from sensor_msgs.msg import Image, CameraInfo
from users_landmarks_msgs.msg import MultipleUsersLandmarks, SingleUserLandmarks

import cv2
import joblib
import numpy as np
import os
import pandas as pd
import time 
from scipy.spatial.transform import Rotation

from users_landmarks_utils.kinect_bt_utils import body_joints_list
from users_landmarks_utils.landmarks_preprocessing_sklearn import SelectUsersFeaturesSklearn, PreprocessBodyJointsSklearn, PreprocessFaceLandmarksSklearn

class MutualGazeDetectorNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("mutual_gaze_detector_node")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('debug_node', False),
                ('threshold', 0.5)
            ])
        
        # Parameters
        self.debug_node = self.get_parameter('debug_node').get_parameter_value().bool_value
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        
        self.get_logger().info("threshold: {}".format(self.threshold))

        self.get_logger().info("debug_node: {}".format(self.debug_node))

        qos = rclpy.qos.QoSProfile(
            depth=1,
        )
        
        # Subscribers
        self.multiple_users_landmarks_sub = self.create_subscription(MultipleUsersLandmarks, 
                                                                     "/face_landmarks_node/users_landmarks", 
                                                                     self.users_landmarks_callback,
                                                                     qos_profile=qos)
        classifier_weights_path = os.path.join(get_package_share_directory("mutual_gaze_detector_ros"),
                                               "classifier_weights",
                                               "mutual_gaze_classifier_random_forest_weights.joblib")
                
        self.initialize_classifier(classifier_weights_path=classifier_weights_path)
        
        self.true_probability = np.zeros((50), dtype=np.float32)
        
        self.gui_window_name = "GUI"
        self.gui_window = cv2.namedWindow(self.gui_window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info("Node correctly initialized.")
    
    def initialize_classifier(self, classifier_weights_path):
        self.classifier_pipeline = joblib.load(classifier_weights_path)
        self.get_logger().info("Classifier correctly initialized.")
        return   
    
    def extract_users_features(self, users_msg):
        users_features_list = []
        for user_msg in users_msg.users:
            # First check that all the features are present
            if len(user_msg.body_landmarks) == 0 or len(user_msg.face_landmarks) == 0:
                continue
            user_dict = {}
            user_dict["timestamp"] = users_msg.header.stamp.sec + users_msg.header.stamp.nanosec/1000000000
            user_dict["frame_id"] = users_msg.header.frame_id
            user_dict["body_id"] = user_msg.body_id
            
            for body_joint_id, body_joint_pose in enumerate(user_msg.body_landmarks):
                # Position
                field_name = f'body_joint_{body_joints_list[body_joint_id]}_position_x'
                user_dict[field_name] = body_joint_pose.position.x
                field_name = f'body_joint_{body_joints_list[body_joint_id]}_position_y'
                user_dict[field_name] = body_joint_pose.position.y
                field_name = f'body_joint_{body_joints_list[body_joint_id]}_position_z'
                user_dict[field_name] = body_joint_pose.position.z
                # Orientation
                field_name = f'body_joint_{body_joints_list[body_joint_id]}_quaternion_x'
                user_dict[field_name] = body_joint_pose.orientation.x
                field_name = f'body_joint_{body_joints_list[body_joint_id]}_quaternion_y'
                user_dict[field_name] = body_joint_pose.orientation.y
                field_name = f'body_joint_{body_joints_list[body_joint_id]}_quaternion_z'
                user_dict[field_name] = body_joint_pose.orientation.z
                field_name = f'body_joint_{body_joints_list[body_joint_id]}_quaternion_w'
                user_dict[field_name] = body_joint_pose.orientation.w
            
            for face_landmark_idx, face_landmark in enumerate(user_msg.face_landmarks):
                field_name = f'face_landmark_{face_landmark_idx}_image_x'
                user_dict[field_name] = face_landmark.x
                field_name = f'face_landmark_{face_landmark_idx}_image_y'
                user_dict[field_name] = face_landmark.y
                field_name = f'face_landmark_{face_landmark_idx}_image_z'
                user_dict[field_name] = face_landmark.z
            
            users_features_list.append(user_dict)
            
        return pd.DataFrame.from_dict(users_features_list)
        
    def run_classifier(self, users_features_dataframe):
        classifier_results_probabilities = self.classifier_pipeline.predict_proba(users_features_dataframe)
        return classifier_results_probabilities
    
    def users_landmarks_callback(self, users_landmarks_msg):
        # Extract data from message
        users_features_dataframe = self.extract_users_features(users_landmarks_msg)
        # Check if dataframe is empty 
        if users_features_dataframe.shape[0] == 0:
            return
        # Run classifier
        are_looking_probabilities = self.run_classifier(users_features_dataframe)
        are_looking = False
        if are_looking_probabilities[0][1] > self.threshold:
            are_looking = True             
        
        self.true_probability = np.roll(self.true_probability, -1)
        self.true_probability[-1] = are_looking_probabilities[0][1]
        
        # Visualize results
        self.get_logger().info(f"User are looking: {are_looking} with probability: {are_looking_probabilities[0]}")
        
        text = f'{self.true_probability[-1]:.3f}'
        feedback_image = np.ones((200, 200, 3), dtype=np.uint8)
        if are_looking:
            feedback_image[:, :, 1] = 255
        else:
            feedback_image[:, :, 2] = 255
               
        cv2.putText(img = feedback_image, 
                    text = text,
                    org = (55, 25), 
                    fontFace = cv2.FONT_HERSHEY_SIMPLEX, 
                    fontScale = 1, 
                    thickness = 1, 
                    color = (0, 0, 0))
        
        plot_image = np.ones((200, 200, 3), dtype=np.uint8) * 255
        
        cv2.line(plot_image, (0, 100), (200, 100), (0, 0, 255), 1)
        
        x = np.linspace(0, 200, 50)
        points = np.column_stack((x.astype(int), 200 -(self.true_probability*200).astype(int)))
        cv2.polylines(plot_image, [points], isClosed=False, color=(255, 0, 0), thickness=1)
        
        cv2.imshow(self.gui_window_name, cv2.vconcat([plot_image, feedback_image]))
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    node = MutualGazeDetectorNode()
    
    rclpy.spin(node)
    
    # On shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()