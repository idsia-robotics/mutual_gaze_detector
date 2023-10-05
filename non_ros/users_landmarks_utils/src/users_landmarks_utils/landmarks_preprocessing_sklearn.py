# Functions for features preprocessing, features selection and normalization in sklearn pipelines
from sklearn.base import BaseEstimator, TransformerMixin
from users_landmarks_utils.landmarks_preprocessing import select_features, preprocess_face_landmarks, preprocess_body_joints

##### FUNCTION TRANSFORMERS IMPLEMENTATIONS #####
# Features selection
class SelectUsersFeaturesSklearn(BaseEstimator, TransformerMixin):
    def __init__(
        self,
        body_joint_names =      None,
        body_position_axis =    "xyz",
        body_orientation_type = "quaternion",
        face_landmark_idxs =    None,
        face_position_axis =    "xyz",
        additional_features =   [],
    ):
        self.body_joint_names =      body_joint_names
        self.body_position_axis =    body_position_axis
        self.body_orientation_type = body_orientation_type
        self.face_landmark_idxs =    face_landmark_idxs
        self.face_position_axis =    face_position_axis
        self.additional_features =   additional_features
    
    def fit(self, X, y=None):
        return self
    
    def transform(self, features_dataframe, y=None):
        return select_features(features_dataframe, 
                                     body_joint_names =      self.body_joint_names,
                                     body_position_axis =    self.body_position_axis,
                                     body_orientation_type = self.body_orientation_type,
                                     face_landmark_idxs =    self.face_landmark_idxs,
                                     face_position_axis =    self.face_position_axis,
                                     additional_features =   self.additional_features
                                     ) 

# Feature preprocessing
class PreprocessFaceLandmarksSklearn(BaseEstimator, TransformerMixin):
    def __init__(
        self,
        center=True,
        normalization = None,
    ):
        self.center = center
        self.normalization = normalization
    
    def fit(self, X, y=None):
        return self
    
    def transform(self, features_dataframe, y=None):
        return preprocess_face_landmarks(features_dataframe, center=self.center, normalization=self.normalization)
    
class PreprocessBodyJointsSklearn(BaseEstimator, TransformerMixin):
    def __init__(
        self,
        normalization = None,
    ):
        self.normalization = normalization
    
    def fit(self, X, y=None):
        return self
    
    def transform(self, features_dataframe, y=None):
        return preprocess_body_joints(features_dataframe, normalization=self.normalization)