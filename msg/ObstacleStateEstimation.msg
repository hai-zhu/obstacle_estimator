# message definition of published topic on obstacle state estimation
Header header

# pose
geometry_msgs/Pose pose

# velocity
geometry_msgs/Twist velocity

# 6x6 state covariance, row-major representation
float64[36] covariance
