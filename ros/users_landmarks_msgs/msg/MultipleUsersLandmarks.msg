# Multiple users landmark message definition 
std_msgs/Header header
uint32 users_number # number of users, max number is defined by a parameter in a config file

users_landmarks_msgs/SingleUserLandmarks[] users