import numpy as np
import cv2 

# https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/opencv_compatibility/main.cpp
class Calibration:
    def __init__(self, camera_info, rectified_image_input: bool = False, ):
        # Save camera info
        self.width = camera_info.width
        self.height = camera_info.height
        self.image_shape = (self.width, self.height)
        
        # Projection Matrix
        cx = camera_info.k[2]
        cy = camera_info.k[5]
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        self.camera_matrix = np.array([[fx, 0., cx],
                                       [0., fy, cy],
                                       [0., 0., 1.]])

        # Distorion Coefficients
        self.distortion_model = camera_info.distortion_model
        self.dist_coeffs = np.zeros((8,), dtype=float, order='C')
        if rectified_image_input:
            k1 = camera_info.d[0]
            k2 = camera_info.d[1]
            k3 = camera_info.d[4]
            k4 = camera_info.d[5]
            k5 = camera_info.d[6]
            k6 = camera_info.d[7]

            p1 = camera_info.d[2]
            p2 = camera_info.d[3]
        
            self.dist_coeffs = np.array([k1, k2, p1, p2, k3, k4, k5, k6])

        self.translation = np.zeros((3,), dtype=float, order='C')
        self.rot_matrix = np.eye(3, k=0, dtype=float, order='C')
        self.rot_angles, _ = cv2.Rodrigues(self.rot_matrix)
        
    def set_depth_to_rgb_extrinsics(self, translation, rot_matrix):
        # Check translation is passed in mm and not in meters
        if translation[0] < 1.0 and translation[1] < 1.0 and translation[2] < 1.0:
            translation *= 1000.0
             
        self.translation = translation        
        self.rot_matrix = rot_matrix
        self.rot_angles, _ = cv2.Rodrigues(self.rot_matrix)
        
    def world_to_rgb_image(self, points_3d):
        points_2d, _ = cv2.projectPoints(points_3d,
                                    self.rot_angles,
                                    self.translation,
                                    self.camera_matrix,
                                    self.dist_coeffs)
        
        points_2d = points_2d.astype(int).clip([0, 0], self.image_shape)
        
        return np.squeeze(points_2d)
    
    def rgb_image_to_world(self, depth_image, points_2d):
        # TODO: Create undistortMaps and precomputed image projection matrices
        # rectified_points_2d = cv2.undistortPoints(np.array([points_2d]), self.camera_matrix, self.dist_coeffs)
        
        points_3d = []
        return points_3d
