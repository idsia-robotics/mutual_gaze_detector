# Functions to return correct features names for body joints and face landmarks

import itertools
from users_landmarks_utils.kinect_bt_utils import body_joints_list

def get_body_joint_features_names_list(joint_names = None, 
                                       position_axis="xyz", 
                                       orientation_type=None):
    if joint_names is None:
        # All body joints
        joint_names = body_joints_list
    body_joint_features_names = []
    # Body joints position wrt camera
    body_joint_features_names.extend([f'body_joint_{joint_name}_position_{axis}' for (joint_name,axis) in itertools.product(joint_names, position_axis)])
    # Body joints orientation wrt camera
    if orientation_type is not None:
        if orientation_type == "quaternion":
            body_joint_features_names.extend([f'body_joint_{joint_name}_quaternion_{axis}' for (joint_name,axis) in itertools.product(joint_names, "xyzw")])
        if orientation_type == "euler_sin":
            body_joint_features_names.extend([f'body_joint_{joint_name}_euler_{axis}_sin' for (joint_name,axis) in itertools.product(joint_names, "xyz")])
        if orientation_type == "euler_cos":
            body_joint_features_names.extend([f'body_joint_{joint_name}_euler_{axis}_cos' for (joint_name,axis) in itertools.product(joint_names, "xyz")])
        if orientation_type == "euler":
            body_joint_features_names.extend([f'body_joint_{joint_name}_euler_{axis}' for (joint_name,axis) in itertools.product(joint_names, "xyz")])
    return body_joint_features_names

def get_face_landmarks_feature_names_list(landmark_idxs = None,
                                          position_axis="xyz"):
    if landmark_idxs is None:
        # All face landmarks
        landmark_idxs = range(478)
    return [f"face_landmark_{landmark_idx}_image_{axis}" for (landmark_idx,axis) in itertools.product(landmark_idxs, position_axis)]

def get_user_features_names(user_body_joint_names = None, 
                            user_body_position_axis="xyz", 
                            user_body_orientation_type="quaternion",
                            user_face_landmark_idxs = None,
                            user_face_position_axis="xyz"):
    # Body joints features
    body_joint_features_names = get_body_joint_features_names_list(joint_names = user_body_joint_names, 
                                                                   position_axis = user_body_position_axis, 
                                                                   orientation_type = user_body_orientation_type)
    # Face landmarks features
    face_landmarks_feature_names = get_face_landmarks_feature_names_list(landmark_idxs = user_face_landmark_idxs,
                                                                         position_axis = user_face_position_axis)
    return body_joint_features_names + face_landmarks_feature_names