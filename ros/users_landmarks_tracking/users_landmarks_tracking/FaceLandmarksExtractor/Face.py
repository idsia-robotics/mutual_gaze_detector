
import numpy as np
import cv2

class Face():    
    # Key landmark indices
    nose_tip_index = 4
    # Left eyelid
    simplified_left_eyelids_indices = [
                            362,  # right 
                            374,  # bottom
                            263,  # left
                            386   # top
                            ]
    left_eyelids_indices = [
                            362,  # right 
                            382,
                            381,
                            380,
                            374,  # bottom
                            373,
                            390,
                            249,
                            263,  # left
                            466,
                            388,
                            387,
                            386,  # top
                            385,
                            384,
                            398
                            ]                            
    # Left eye
    left_eye_contour_indices = [
                                    476, # Right
                                    477, # Bottom
                                    474, # Left
                                    475  # Top
                                ]
    left_eye_left_right_indices = [
                                    362, # Right
                                    263, # Left
                                ]                                
    left_eye_center_index = 473
    left_eye_iris_indeces = [
                                476, # Right
                                477, # Bottom
                                474, # Left
                                475, # Top
                                473  # Center  
                            ]
    left_eye_orientation_landmarks=[442,450,362,263, 473]
    # Right eyelid
    simplified_right_eyelids_indices = [
                                130, # right
                                145, # bottom
                                133, # left
                                159  # top
                            ]
    right_eyelids_indices = [
                                130, # right
                                7,
                                163,
                                144,
                                145, # bottom
                                153,
                                154,
                                155,
                                133, # left
                                173,
                                157,
                                158,
                                159, # top
                                160,
                                161,
                                246,
                                33
                            ]
    # Right eye
    right_eye_contour_indices = [
                                    471, # right
                                    472, # bottom
                                    469, # left
                                    470  # top
                                ]
    right_eye_left_right_indices = [
                                    130, # Right
                                    133, # Left
                                ]                                   
    right_eye_center_index = 468  
    right_eye_iris_indices = [
                                    471, # right
                                    472, # bottom
                                    469, # left
                                    470,  # top
                                    468  # Center
                                ]  
    right_eye_orientation_landmarks=[223,230, 130, 133, 468]
    
    # Mouth
    simplified_mouth_outer_indices = [
                            61,  # right 
                            17,  # bottom
                            291,  # left
                            0   # top
                            ]
    mouth_outer_indices = [
                            61,  # right 
                            146,
                            91,
                            181,
                            84,
                            17,  # bottom
                            314,
                            405,
                            321,
                            375,
                            291,  # left
                            409,
                            270,
                            269,
                            267,
                            0,  # top
                            37,
                            39,
                            40,
                            185
                            ]      
    simplified_mouth_inner_indices = [
                            78,  # right 
                            14,  # bottom
                            308,  # left
                            13   # top
                            ]
    mouth_inner_indices = [
                            78,  # right 
                            95,
                            88,
                            178,
                            87,
                            14,  # bottom
                            317,
                            402,
                            318,
                            324,
                            308,  # left
                            415,
                            310,
                            311,
                            312,
                            13,  # top
                            82,
                            81,
                            80,
                            191
                            ]  
    # Nose
    simplified_nose_indices = [
                            129,  # right 
                            94,  # bottom
                            358,  # left
                            4   # top
                            ]
    nose_indices = [
                            129,  # right 
                            219,
                            166,
                            239,
                            20,
                            242,
                            141,
                            94,  # bottom
                            370,
                            462,
                            250,
                            459,
                            392,
                            439,
                            358,  # left
                            344,
                            440,
                            275,
                            4,  # top
                            45,
                            220,
                            115,
                            49,
                            131,
                            134,
                            51,
                            5,
                            281,
                            363,
                            360,
                            279,
                            429,
                            420,
                            456,
                            248,
                            195,
                            3,
                            236,
                            198,
                            209,
                            217,
                            174,
                            196,
                            197,
                            419,
                            399,
                            437,
                            343,
                            412,
                            351,
                            6,
                            122,
                            188,
                            114,
                            245,
                            193,
                            168,
                            417,
                            465
                            ]
    # Forehead
    forehead_center_index = [151]
    forehead_indices = [
        301,
        298,
        333,
        299,
        337,
        151,
        108,
        69,
        104,
        68,
        71,
        21,
        54,
        103,
        67,
        109,
        10,
        338,
        297,
        332,
        284,
        251
    ]
    # Eye_brows
    left_eye_brows_indices = [
        
        285,
        295,
        282,
        283,
        276,
        383,
        300,
        293,
        334,
        296,

    ]
    right_eye_brows_indices = [

        156,
        46,
        53,
        52,
        65,
        55,
        66,
        105,
        63,
        70
    ]    
    # A list of simplified facial features used to reduce computation cost of drawing and morphing faces
    simplified_face_features = [
        10, 67, 54, 162, 127, 234, 93, 132,172,150,176,148,152,377,378,365,435,323,447,454,264,389,251, 332, 338, #Oval
        139, 105, 107, 151, 8, 9, 336, 334, 368,                    #  Eyelids
        130, 145, 155, 6, 382, 374, 263, 159, 386,                  #  Eyes
        129, 219, 79, 238, 2, 458, 457, 439, 358, 1, 4, 5, 197,     #  Nose
        61, 84, 314, 409, 14, 87, 81, 12,37,267, 402, 311, 321, 269, 39, 415, 91, 178, 73, 303, 325,
        50, 207, 280, 427
    ]

    face_oval_indices = [
        10,
        109,
        67,
        103,
        54,
        21,
        162,
        127,
        234,
        93,
        132,
        58,
        172,
        136,
        150,
        149,
        176,
        148,
        152,
        377,
        400,
        378,
        379,
        365,
        397,
        288,
        361,
        323,
        454,
        356,
        389,
        251,
        284,
        332,
        297,
        338
    ]

    face_forhead_indices = [
        10,
        109,
        67,
        103,
        104,
        105,
        66,
        107,
        9,
        336,
        296,
        334,
        333,
        332,
        297,
        338
    ]
    all_face_features = list(range(468))

    def draw_landmarks(self, image: np.ndarray, landmarks: np.ndarray=None, radius:int=1, color: tuple = (255, 0, 0), thickness: int = 1, link=False) -> np.ndarray:
        lm_l=landmarks.shape[0]
        for i in range(lm_l):
            image = cv2.circle(image, (int(landmarks[i,0]), int(landmarks[i,1])), radius,color, thickness)
            if link:
                image = cv2.line(image, (int(landmarks[i,0]), int(landmarks[i,1])),(int(landmarks[(i+1)%lm_l,0]), int(landmarks[(i+1)%lm_l,1])),color, thickness)
        return image