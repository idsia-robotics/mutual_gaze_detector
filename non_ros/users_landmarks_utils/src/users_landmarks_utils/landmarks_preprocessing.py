# Functions for features preprocessing, features selection and normalization

import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation
from itertools import compress
from users_landmarks_utils.kinect_bt_utils import body_joints_list
from users_landmarks_utils.landmarks_naming import get_user_features_names, get_body_joint_features_names_list, get_face_landmarks_feature_names_list
from sklearn.base import BaseEstimator, TransformerMixin

##### UTILS ##### 
def intersection(lst1, lst2):
    lst3 = [value for value in lst1 if value in lst2]
    return lst3

def get_used_body_joint_names(columns_names):
    body_joint_names_used = []
    for body_joint in body_joints_list:
        if any([body_joint in col_name for col_name in columns_names]):
            body_joint_names_used.append(body_joint)
    return body_joint_names_used

##### FUNCTIONS IMPLEMENTATIONS #####
# Features selection
def select_features(features_dataframe,
                    body_joint_names = None, 
                    body_position_axis="xyz", 
                    body_orientation_type="quaternion",
                    face_landmark_idxs = None,
                    face_position_axis="xyz",
                    additional_features = []
                    ):
    selected_features_names = get_user_features_names(user_body_joint_names = body_joint_names,
                                                      user_body_position_axis = body_position_axis,
                                                      user_body_orientation_type = body_orientation_type,
                                                      user_face_landmark_idxs = face_landmark_idxs,
                                                      user_face_position_axis = face_position_axis)
    selected_features_names.extend(additional_features)
    
    selected_features_names_in_dataframe = intersection(selected_features_names, features_dataframe.columns)    
    
    return features_dataframe.loc[:, selected_features_names_in_dataframe]

# Feature preprocessing
def preprocess_face_landmarks(features_dataframe, center=True, normalization=None):
    # Check if dataset uses also z coordinate for face landmarks
    z_coordinate = any(col_name.endswith('image_z') for col_name in features_dataframe.columns)
    
    # Get columns names of features really present in the input dataset
    face_landmarks_all_x_names = get_face_landmarks_feature_names_list(landmark_idxs = None, position_axis="x")
    face_landmarks_all_y_names = get_face_landmarks_feature_names_list(landmark_idxs = None, position_axis="y")
    face_landmarks_all_z_names = get_face_landmarks_feature_names_list(landmark_idxs = None, position_axis="z")
    
    face_landmarks_x_names = intersection(face_landmarks_all_x_names ,features_dataframe.columns)
    face_landmarks_y_names = intersection(face_landmarks_all_y_names ,features_dataframe.columns)
    face_landmarks_z_names = intersection(face_landmarks_all_z_names ,features_dataframe.columns)
    
    features_preprocessed = features_dataframe.copy()
    
    if center:
        # Select x and y coordinates of face landmarks
        x_features = features_dataframe.loc[:, face_landmarks_x_names]
        y_features = features_dataframe.loc[:, face_landmarks_y_names]
        z_features = features_dataframe.loc[:, face_landmarks_z_names]

        # User-wise center
        cx = x_features.mean(axis=1)
        cy = y_features.mean(axis=1)
        cz = z_features.mean(axis=1)
        
        # Center face landmarks
        features_preprocessed.loc[:, face_landmarks_x_names] = x_features.subtract(cx, axis=0)
        features_preprocessed.loc[:, face_landmarks_y_names] = y_features.subtract(cy, axis=0)
        features_preprocessed.loc[:, face_landmarks_z_names] = z_features.subtract(cz, axis=0)
    
    if normalization is not None:
        x_features = features_preprocessed.loc[:, face_landmarks_x_names].values    
        y_features = features_preprocessed.loc[:, face_landmarks_y_names].values    
        z_features = features_preprocessed.loc[:, face_landmarks_z_names].values if z_coordinate else np.zeros_like(x_features)    
        
        x_span = []
        y_span = []
        z_span = []
    
        if normalization=='dimensionwise':
            #  Get span of each dimension.
            x_span = np.mean(np.absolute(x_features), axis=1)
            y_span = np.mean(np.absolute(y_features), axis=1)
            z_span = np.mean(np.absolute(z_features), axis=1)
                        
        if normalization=='norm':
            # Problematic if one dimension dominates the others
            norm = np.mean(np.linalg.norm(np.stack((x_features, y_features, z_features), axis=2), axis=2), axis=1)
    
            x_span = norm
            y_span = norm
            z_span = norm
        
        x_span[x_span == 0] = 1.0
        y_span[y_span == 0] = 1.0
        z_span[z_span == 0] = 1.0
                            
        # Normalize features
        features_preprocessed.loc[:, face_landmarks_x_names] = features_preprocessed.loc[:, face_landmarks_x_names].divide(x_span, axis=0)
        features_preprocessed.loc[:, face_landmarks_y_names] = features_preprocessed.loc[:, face_landmarks_y_names].divide(y_span, axis=0)
        features_preprocessed.loc[:, face_landmarks_z_names] = features_preprocessed.loc[:, face_landmarks_z_names].divide(z_span, axis=0)
       
    return features_preprocessed

def preprocess_body_joints(features_dataframe, normalization=None):
    if normalization is None:
        return features_dataframe
    if normalization == "sin-cos":
        # Find out which joints are being used
        used_body_joint_names = get_used_body_joint_names(features_dataframe.columns)
        
        body_joints_positions_names = get_body_joint_features_names_list(joint_names = used_body_joint_names, 
                                                                        position_axis="xyz",
                                                                        orientation_type="")
        body_joints_orientation_quaternion_names = get_body_joint_features_names_list(joint_names = used_body_joint_names, 
                                                                                    position_axis = "",
                                                                                    orientation_type="quaternion")
        body_joints_orientation_euler_sin_names = get_body_joint_features_names_list(joint_names = used_body_joint_names,
                                                                                    position_axis = "",
                                                                                    orientation_type="euler_sin")
        body_joints_orientation_euler_cos_names = get_body_joint_features_names_list(joint_names = used_body_joint_names, 
                                                                                    position_axis = "",
                                                                                    orientation_type="euler_cos")
        # Body joints dataframes
        body_joints_position_dataframe = features_dataframe.loc[:, body_joints_positions_names]
        
        # Quaternion components matrices [n_users, n_joints]
        quaternion_x = features_dataframe.loc[:, body_joints_orientation_quaternion_names[0::4]].values
        quaternion_y = features_dataframe.loc[:, body_joints_orientation_quaternion_names[1::4]].values
        quaternion_z = features_dataframe.loc[:, body_joints_orientation_quaternion_names[2::4]].values
        quaternion_w = features_dataframe.loc[:, body_joints_orientation_quaternion_names[3::4]].values
        
        # Quaternion matrix [n_users, n_joints, 4] -> [n_users*n_joints, 4]
        quaternion_matrix = np.stack((quaternion_x, quaternion_y, quaternion_z, quaternion_w), axis=2).reshape(-1, 4)

        # order as perceived by cipy is from right to left not from left to right
        # now euler angles are in the order zyx for scipy but xyz for common sense
        euler_angles_convention = "zyx" 
        
        # Euler angles matrix [n_users*n_joints, 3] -> [n_users, n_joints, 3]
        euler_matrix = Rotation.from_quat(quaternion_matrix).as_euler(euler_angles_convention, degrees=False).reshape(-1, len(used_body_joint_names), 3)
        
        # Matrices with sin or cos of euler angles [n_users, n_joints*3]
        euler_cos = np.cos(euler_matrix).reshape(-1, len(used_body_joint_names)*3)
        euler_sin = np.sin(euler_matrix).reshape(-1, len(used_body_joint_names)*3)
        
        # Matrix with cos for every joint and then sin for every joint [n_users, n_joints*3(cos) + n_joints*3(sin)]
        euler_cos_sin = np.concatenate((euler_cos, euler_sin), axis=1)
        
        # Create dataframe from euler angles matrix
        body_joints_orientation_cos_sin_dataframe = pd.DataFrame(euler_cos_sin, index=body_joints_position_dataframe.index, columns=(body_joints_orientation_euler_cos_names + body_joints_orientation_euler_sin_names))

        # All columns besides the body joints ones
        face_landmarks_dataframe = features_dataframe.loc[:, features_dataframe.columns.difference(body_joints_positions_names + body_joints_orientation_quaternion_names)]
        
        features_processed = pd.concat([body_joints_position_dataframe, body_joints_orientation_cos_sin_dataframe, face_landmarks_dataframe], axis=1)
        
        return features_processed
    else:
        raise ValueError("Normalization type not supported")