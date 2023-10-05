# Useful information about Azure Kinect Body tracking SDK solution 
# Info taken from here https://learn.microsoft.com/en-us/azure/kinect-dk/body-joints

body_joints_list = ["PELVIS","SPINE_NAVAL","SPINE_CHEST","NECK","CLAVICLE_LEFT","SHOULDER_LEFT","ELBOW_LEFT","WRIST_LEFT","HAND_LEFT","HANDTIP_LEFT","THUMB_LEFT","CLAVICLE_RIGHT","SHOULDER_RIGHT","ELBOW_RIGHT","WRIST_RIGHT","HAND_RIGHT","HANDTIP_RIGHT","THUMB_RIGHT","HIP_LEFT","KNEE_LEFT","ANKLE_LEFT","FOOT_LEFT","HIP_RIGHT","KNEE_RIGHT","ANKLE_RIGHT","FOOT_RIGHT","HEAD","NOSE","EYE_LEFT","EAR_LEFT","EYE_RIGHT","EAR_RIGHT"]

body_joints_groups_list = ["SPINE", "LEFT_ARM", "LEFT_HAND", "RIGHT_ARM", "RIGHT_HAND", "LEFT_LEG", "RIGHT_LEG", "HEAD", "FACE"]

body_joints_info = {
    "PELVIS": {"number": 0, "parent": ""},
    "SPINE_NAVAL": {"number": 1, "parent": "PELVIS"},
    "SPINE_CHEST": {"number": 2, "parent": "SPINE_NAVAL"},
    "NECK": {"number": 3, "parent": "SPINE_CHEST"},
    "CLAVICLE_LEFT": {"number": 4, "parent": "SPINE_CHEST"},
    "SHOULDER_LEFT": {"number": 5, "parent": "CLAVICLE_LEFT"},
    "ELBOW_LEFT": {"number": 6, "parent": "SHOULDER_LEFT"},
    "WRIST_LEFT": {"number": 7, "parent": "ELBOW_LEFT"},
    "HAND_LEFT": {"number": 8, "parent": "WRIST_LEFT"},
    "HANDTIP_LEFT": {"number": 9, "parent": "HAND_LEFT"},
    "THUMB_LEFT": {"number": 10, "parent": "WRIST_LEFT"},
    "CLAVICLE_RIGHT": {"number": 11, "parent": "SPINE_CHEST"},
    "SHOULDER_RIGHT": {"number": 12, "parent": "CLAVICLE_RIGHT"},
    "ELBOW_RIGHT": {"number": 13, "parent": "SHOULDER_RIGHT"},
    "WRIST_RIGHT": {"number": 14, "parent": "ELBOW_RIGHT"},
    "HAND_RIGHT": {"number": 15, "parent": "WRIST_RIGHT"},
    "HANDTIP_RIGHT": {"number": 16, "parent": "HAND_RIGHT"},
    "THUMB_RIGHT": {"number": 17, "parent": "WRIST_RIGHT"},
    "HIP_LEFT": {"number": 18, "parent": "PELVIS"},
    "KNEE_LEFT": {"number": 19, "parent": "HIP_LEFT"},
    "ANKLE_LEFT": {"number": 20, "parent": "KNEE_LEFT"},
    "FOOT_LEFT": {"number": 21, "parent": "ANKLE_LEFT"},
    "HIP_RIGHT": {"number": 22, "parent": "PELVIS"},
    "KNEE_RIGHT": {"number": 23, "parent": "HIP_RIGHT"},
    "ANKLE_RIGHT": {"number": 24, "parent": "KNEE_RIGHT"},
    "FOOT_RIGHT": {"number": 25, "parent": "ANKLE_RIGHT"},
    "HEAD": {"number": 26, "parent": "NECK"},
    "NOSE": {"number": 27, "parent": "HEAD"},
    "EYE_LEFT": {"number": 28, "parent": "HEAD"},
    "EAR_LEFT": {"number": 29, "parent": "HEAD"},
    "EYE_RIGHT": {"number": 30, "parent": "HEAD"},
    "EAR_RIGHT": {"number": 31, "parent": "HEAD"}
}

body_joints_groups = {
    "SPINE" : ["PELVIS", "SPINE_NAVAL", "SPINE_CHEST", "NECK"],
    "LEFT_ARM" : ["SPINE_CHEST", "CLAVICLE_LEFT", "SHOULDER_LEFT", "ELBOW_LEFT", "WRIST_LEFT", "HAND_LEFT", "HANDTIP_LEFT"],
    "LEFT_HAND" : ["WRIST_LEFT", "THUMB_LEFT"],
    "RIGHT_ARM" : ["SPINE_CHEST", "CLAVICLE_RIGHT", "SHOULDER_RIGHT", "ELBOW_RIGHT", "WRIST_RIGHT", "HAND_RIGHT", "HANDTIP_RIGHT"],
    "RIGHT_HAND" : ["WRIST_RIGHT", "THUMB_RIGHT"],
    "LEFT_LEG" : ["PELVIS", "HIP_LEFT", "KNEE_LEFT", "ANKLE_LEFT", "FOOT_LEFT"],
    "RIGHT_LEG" : ["PELVIS", "HIP_RIGHT", "KNEE_RIGHT", "ANKLE_RIGHT", "FOOT_RIGHT"],
    "HEAD" : ["NECK", "HEAD", "NOSE"],
    "FACE" : ["EAR_LEFT", "EYE_LEFT", "NOSE", "EYE_RIGHT", "EAR_RIGHT"],
    "GAZE_MINIMAL" : ["EYE_LEFT", "EYE_RIGHT", "HEAD", "SPINE_CHEST"]
}

def get_body_joints_names_by_groups(group_names):
    if group_names == "all":
        group_names = body_joints_groups_list
    
    selected_joints_names = []
    
    for group_name in group_names:
        selected_joints_names += body_joints_groups[group_name]
        
    selected_joints_names = list(set(selected_joints_names))
    
    return selected_joints_names

def get_body_segments_joint_numbers():
    spine_coords = [0, 1, 2, 3]
    left_arm_coords = [2, 4, 5, 6, 7, 8, 9]
    left_hand_coords = [7, 10]
    right_arm_coords = [2, 11, 12, 13, 14, 15, 16]
    right_hand_coords = [14, 17]
    left_leg_coords = [0, 18, 19, 20, 21]
    right_leg_coords = [0, 22, 23, 24, 25]
    head_coords = [3, 26, 27]
    face_coords = [29, 28, 27, 30, 31]
    return [spine_coords, left_arm_coords, left_hand_coords, 
            right_arm_coords, right_hand_coords, left_leg_coords, 
            right_leg_coords, head_coords, face_coords]