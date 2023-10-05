import cv2
import numpy as np
import mediapipe as mp
import os
import time
from typing import NamedTuple, Tuple

# from .Face import Face

class FacesLandmarksExtractor():
    """
    A class that extracts the facial landmarks of the detected people present in an image
    """

    def __init__(self, 
                 maximum_number_of_bodies=3, 
                 mediapipe_image_shape: tuple = (256, 256), 
                 static_image_mode=False,
                 version= 'simple',
                 model_path: str = None,
                 logger = None):
        """
        Creates multiple instances of the FaceAnalyzer object such that each instance is responsible for the analysis of a single face

        Args:
            maximum_number_of_bodies (int,optional) : The maximum number of faces to be treated by extractor. Defaults to 3.
            mediapipe_image_shape (tuple, optional): The shape of the image to be processed by MediaPipe. Defaults to (256, 256).
                Should not need bigger image as the input to the net is (256, 256) https://developers.google.com/mediapipe/solutions/vision/face_landmarker.
            static_image_mode (bool, optional): Whether to treat the input images as a batch of static and possibly unrelated images, or a video stream. Defaults to False.
            version (str, optional): The version of the model to be used, 'simple' and fast or 'complex' but slower. Defaults to 'simple'.
            model_path (str, optional): The path to the model to be used. Defaults to None.
            logger (Logger, optional): The logger to be used. Defaults to None.
        """
        
        self.maximum_number_of_bodies = maximum_number_of_bodies
        self.mediapipe_image_shape = mediapipe_image_shape
        self.static_image_mode = static_image_mode
        self.version = version
        self.logger = logger
        
        self.number_of_bodies_tracked = 0
        self.bodies_indeces_to_mp_instances_dict = {}
        
        self.face_mesh_solutions = []
        if self.version == 'simple':
            self.face_mesh_solutions = [mp.solutions.face_mesh.FaceMesh(refine_landmarks=True, max_num_faces=1, static_image_mode=static_image_mode) for i in range(maximum_number_of_bodies)]
            self.logger.info(f"Face landmark model is using 'simple' version")

        if self.version == 'complex' and os.path.exists(model_path):
            mp_base_options = mp.tasks.BaseOptions(model_asset_path=model_path)
            mp_options = mp.tasks.vision.FaceLandmarkerOptions(base_options=mp_base_options,
                                                           running_mode=mp.tasks.vision.RunningMode.VIDEO,
                                                           output_face_blendshapes=True,
                                                           output_facial_transformation_matrixes=False,
                                                           num_faces=1)
            self.face_mesh_solutions = [mp.tasks.vision.FaceLandmarker.create_from_options(mp_options) for i in range(maximum_number_of_bodies)]
        
        self.body_id_face_landmarks_nn_results = [[] for n in range(maximum_number_of_bodies)]
        self.body_id_face_landmarks_original = [[] for n in range(maximum_number_of_bodies)]
        
        # self.faces_tracked = [Face(image_shape=mediapipe_image_shape) for i in range(maximum_number_of_bodies)]
        
        self.input_image = None
        
        if self.logger is not None:
            self.logger.info(f"FacesLandmarksExtractor initialized correctly")

    @property
    def image_size(self)->tuple:
        """
        A property to image size

        Returns:
            tuple: The image size
        """
        return self.mediapipe_image_shape

    @image_size.setter
    def image_size(self,new_shape:tuple):
        self.mediapipe_image_shape=new_shape
        for face in self.faces:
            face.mediapipe_image_shape=new_shape

    @property
    def active_mediapipe_instances(self)->int:
        """
        A property to the number of active MediaPipe instances

        Returns:
            int: The number of active MediaPipe instances
        """
        return list(int(i) for i in self.bodies_indeces_to_mp_instances_dict.values())

    def process(self, 
                input_image: np.ndarray,
                frame_timestamp_ms: int, 
                bodies_index_points_3d_roi: list) -> NamedTuple:
        """
        Processes an image and extracts the faces associated with the bodies detected by the pose landmarks

        Args:
            input_image (np.ndarray): The image from which extract faces

        Returns:
            NamedTuple: The result of extracting the image, a list of different faces landmarks
        """
        # Input bodies indeces
        input_bodies_indeces = [body_index_points_3d_2d[0] for body_index_points_3d_2d in bodies_index_points_3d_roi]
        
        # Update map between tracked bodies IDs and MediaPipe FaceMesh solutions
        self.update_bodies_indeces_to_mp_instances_map(input_bodies_indeces)          
        
        # Process the different faces of different bodies
        input_image_shape = input_image.shape # (height, width, channels)
        
        # Process the image face-wise
        for body_id, body_points_3d, face_roi in bodies_index_points_3d_roi:
            # Get the MediaPipe FaceMesh solution instance index associated with the body ID
            mp_instance_index = self.bodies_indeces_to_mp_instances_dict[body_id]
            
            # if upsampling use INTER_LINEAR, if downsampling use INTER_AREA
            interpolation = cv2.INTER_LINEAR if (face_roi[1][0]-face_roi[0][0]) > self.mediapipe_image_shape[0] else cv2.INTER_AREA
            face_roi_image = cv2.resize(input_image[face_roi[0][1]:face_roi[1][1], face_roi[0][0]:face_roi[1][0]], 
                                        self.mediapipe_image_shape,
                                        interpolation= interpolation)
            
            if self.version == 'simple':
                self.body_id_face_landmarks_nn_results[mp_instance_index] = (body_id, 
                                                                             self.face_mesh_solutions[mp_instance_index].process(face_roi_image).multi_face_landmarks)
                
            if self.version == 'complex':
                rgb_frame = mp.Image(image_format=mp.ImageFormat.SRGB, data=face_roi_image)

                self.body_id_face_landmarks_nn_results[mp_instance_index] = (body_id, 
                                                                             self.face_mesh_solutions[mp_instance_index].detect_for_video(rgb_frame,frame_timestamp_ms).face_landmarks)
            
            if self.body_id_face_landmarks_nn_results[mp_instance_index][1] is None:
                # No face found, append empty list
                self.body_id_face_landmarks_original[mp_instance_index] = (body_id, np.array([]), False)
                continue

            # Landmarks are normalized to ROI dimensions but need to return landmarks in the original image dimensions
            self.body_id_face_landmarks_original[mp_instance_index] = (body_id, 
                                                                       self.trasform_landmarks_to_original_image(body_id_landmarks = self.body_id_face_landmarks_nn_results[mp_instance_index],
                                                                                                                 face_roi=face_roi,
                                                                                                                 global_image_shape=input_image_shape),
                                                                       True)
        
        return [self.body_id_face_landmarks_original[active_instance] for active_instance in self.active_mediapipe_instances]
                
    def update_bodies_indeces_to_mp_instances_map(self, input_bodies_indeces: list):
        # Keep the tracked bodies IDs from previous frame with same MediaPipe FaceMesh solution instance
        free_mediapipe_instances = [*range(0, self.maximum_number_of_bodies, 1)]
        
        map_dict = dict(self.bodies_indeces_to_mp_instances_dict)
        
        for body_index_key in self.bodies_indeces_to_mp_instances_dict:
            if body_index_key not in input_bodies_indeces: # Body ID is no longer tracked
                del map_dict[body_index_key]
            else:
                # Remove body ID as it is already in the map and tracked. 
                input_bodies_indeces.remove(body_index_key)
                # The associated MediaPipe FaceMesh solution instance therefore is not free.
                free_mediapipe_instances.remove(self.bodies_indeces_to_mp_instances_dict[body_index_key])   
                
        # Associate the remaining input bodies IDs with the free MediaPipe FaceMesh solution instances
        for input_body_index in input_bodies_indeces:
            map_dict[input_body_index] = free_mediapipe_instances.pop()
                   
        self.bodies_indeces_to_mp_instances_dict = map_dict
        
    def trasform_landmarks_to_original_image(self, 
                                             body_id_landmarks: tuple, 
                                             face_roi: tuple, 
                                             global_image_shape: tuple):
        # Remember in opencv point is (cols, rows)
        # Obtain ROI normalized position and normalized dimension in the original image
        roi_normalized_position = (float(face_roi[0][0])/global_image_shape[1], float(face_roi[0][1])/global_image_shape[0])
        roi_normalized_dimension = (float(face_roi[1][0]-face_roi[0][0])/global_image_shape[1], float(face_roi[1][1]-face_roi[0][1])/global_image_shape[0])
        
        normalized_face_landmarks_original_image = np.array([[lm.x * roi_normalized_dimension[0] + roi_normalized_position[0], 
                                                              lm.y * roi_normalized_dimension[1] + roi_normalized_position[1], 
                                                              lm.z * roi_normalized_dimension[0]] for lm in body_id_landmarks[1][0].landmark])
             
        return normalized_face_landmarks_original_image