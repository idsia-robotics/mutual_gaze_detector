# Landmarks groups for the mutual gaze detector divided by priority

##### RIGHT EYE #####
right_eye_iris_indeces_medium = [471, 472, 469, 470, 468] # [right, bottom, left, top, center]
right_eye_iris_indeces_small  = [468] # [center]
right_eye_iris_indeces_nano   = [468] # [center]

right_eyelids_indeces_medium = [7, 163, 144, 145, 153, 154, 155,   # bottom eyelid
                                133,                               # centermost point
                                173, 157, 158, 159, 160, 161, 246, # top eyelid
                                33]                                # outermost point
right_eyelids_indeces_small  = [145, # bottom center
                                133, # centermost point
                                159, # top eyelid center
                                33]  # outermost point
right_eyelids_indeces_nano  =  [145, # bottom center
                                133, # centermost point
                                159, # top eyelid center
                                33]  # outermost point

right_eye_brows_indeces_medium = [156, 46, 53, 52, 65, 55, 66, 105, 63, 70]
right_eye_brows_indeces_small  = []
right_eye_brows_indeces_nano   = []

right_eye_indeces_medium = right_eye_iris_indeces_medium + right_eyelids_indeces_medium + right_eye_brows_indeces_medium
right_eye_indeces_small  = right_eye_iris_indeces_small  + right_eyelids_indeces_small  + right_eye_brows_indeces_small
right_eye_indeces_nano   = right_eye_iris_indeces_nano   + right_eyelids_indeces_nano   + right_eye_brows_indeces_nano

##### LEFT EYE #####
left_eye_iris_indeces_medium = [476, 477, 474, 475, 473] # [right, bottom, left, top, center]
left_eye_iris_indeces_small  = [473] # [center]
left_eye_iris_indeces_nano   = [473] # [center]

left_eyelids_indeces_medium = [362,                               # centermost point   
                               382, 381, 380, 374, 373, 390, 249, # bottom eyelid
                               263,                               # outermost point        
                               466, 388, 387, 386, 385, 384, 398] # top eyelid
left_eyelids_indeces_small  = [362, # centermost point   
                               374, # bottom eyelid center
                               263, # outermost point        
                               386] # top eyelid center
left_eyelids_indeces_nano   = [362, # centermost point   
                               374, # bottom eyelid center
                               263, # outermost point        
                               386] # top eyelid center      

left_eye_brows_indeces_medium = [285, 295, 282, 283, 276, 383, 300, 293, 334, 296]
left_eye_brows_indeces_small = []
left_eye_brows_indeces_nano  = []

left_eye_indeces_medium = left_eye_iris_indeces_medium + left_eyelids_indeces_medium + left_eye_brows_indeces_medium
left_eye_indeces_small  = left_eye_iris_indeces_small  + left_eyelids_indeces_small  + left_eye_brows_indeces_small
left_eye_indeces_nano   = left_eye_iris_indeces_nano   + left_eyelids_indeces_nano   + left_eye_brows_indeces_small

##### FOREHEAD #####
forehead_indeces_medium = [301, 298, 333, 299, 337, 151, 108, 69, 104, 68, 71, 21, 54, 103, 67, 109, 10, 338, 297, 332, 284, 251]
forehead_indeces_small  = []
forehead_indeces_nano   = []
 
##### FACE OVAL #####
# Anti-clockwise from top of head
face_oval_indeces_medium = [10, # top
                            109, 67, 103, 54, 21, 162, 127, 
                            234, # rightmost point
                            93, 132, 58, 172, 136, 150, 149, 176, 148, 
                            152, # bottom
                            377, 400, 378, 379, 365, 397, 288, 361, 323, 
                            454, # leftmost point 
                            356, 389, 251, 284, 332, 297, 338]
face_oval_indeces_small  = [10,  # top
                            54, 
                            234, # rightmost point
                            136,
                            152, # bottom
                            365,
                            454, # leftmost point 
                            284]
face_oval_indeces_nano   = [10,  # top
                            234, # rightmost point
                            152, # bottom
                            454] # leftmost point

##### NOSE #####
nose_indeces_medium = [151,9,8,168,6,197,195,5,4,1,19,94, 2,164, # Nose centerline
                       69,299,107,336, # Forehead center
                       190,193,417,414,243,245,122,351,465,463,112,341, # Around eyes
                       188,412,217,196,419,437,236,456,131,134,51,281,363,360,115,45,275,344,218,237,44,274,457,438,79,239,241,125,354,461,459,309,241,461,79,20,242,462,250,309, #Nose
                       75,97,326,305] # Under nose
nose_indeces_small  = [151, 6, 1, 19,    # Centerline
                       236, 456,         # Sides
                       75, 97, 326, 305] # Under nose
nose_indeces_nano   = [6, 1,     # Centerline
                       236, 456, # Sides
                       94]       # Under nose

##### MOUTH #####
mouth_indeces_medium = [57,76,78,292,291,287, #centerline_sides
                        39,0,269,
                        184,73,11,303,408,
                        42,38,12,268,272,
                        191,81,82,13,312,311,415,
                        95,178,87,14,317,402,324,
                        89,86,15,316,319,
                        77,180,16,404,307,
                        181,17,405]  
mouth_indeces_small  = [62,      # Rightmost point
                        292,     # Leftmost point
                        38, 268, # Top lip
                        86, 316] # Bottom Lip
mouth_indeces_nano   = [62,      # Rightmost point
                        292]     # Leftmost point
                        
##### CHEEKS #####
cheeks_indeces_medium = [116,117,119,348,346,345, #below eyes  
                         123,50,205,425,280,352,
                         187,192,135,210,411,416,364,430,
                         214, 434, # Mouth sides
                         182,406,32,200,262]
cheeks_indeces_small  = [50,280,  # Cheeks center
                         214, 434, # Mouth sides
                         200]      # Chin
cheeks_indeces_nano   = [214, 434] # Mouth sides


face_features_medium = right_eye_indeces_medium + left_eye_indeces_medium + forehead_indeces_medium + face_oval_indeces_medium + nose_indeces_medium + mouth_indeces_medium + cheeks_indeces_medium
face_features_small  = right_eye_indeces_small  + left_eye_indeces_small  + forehead_indeces_small  + face_oval_indeces_small  + nose_indeces_small  + mouth_indeces_small  + cheeks_indeces_small
face_features_nano   = right_eye_indeces_nano   + left_eye_indeces_nano   + forehead_indeces_nano   + face_oval_indeces_nano   + nose_indeces_nano   + mouth_indeces_nano   + cheeks_indeces_nano
 
# Remove duplicates
face_features_medium = list(set(face_features_medium))
face_features_small  = list(set(face_features_small))
face_features_nano  = list(set(face_features_nano))

def get_face_landmarks_groups(landmarks_types='all'):
    # All landmarks
    if landmarks_types == 'all':
        return range(0, 478)
    
    # Subset of 249 landmarks from Eyes, Nose, Forehead, Face Oval, Mouth and Cheeks
    if landmarks_types == 'medium':
        return face_features_medium
    
    # Subset of 39 landmarks from Eyes, Nose, Face Oval, Mouth and Cheeks
    if landmarks_types == 'small':
        return face_features_small
    
    # Subset of 39 landmarks from Eyes, Nose, Face Oval, Mouth and Cheeks
    if landmarks_types == 'nano':
        return face_features_nano