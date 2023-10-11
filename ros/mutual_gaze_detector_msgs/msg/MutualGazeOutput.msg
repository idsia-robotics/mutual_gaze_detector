std_msgs/Header header
int32[] body_ids # list of body ids, matching each user with the corresponding detector output

float32[] output # output[n] is the detector output for body_ids[n]

bool[] mutual_gaze # mutual_gaze[n] is the thresholded output for body_ids[n]