Header header
# pose is in DSO frame (camToWorld), not in metric scale.
geometry_msgs/Pose pose
float64 scale
# Gravity direction encoded as the rotation to convert from metric world to DSO world.
geometry_msgs/Quaternion rotationMetricToDSO
# Transformation between camera and IMU.
geometry_msgs/Pose imuToCam