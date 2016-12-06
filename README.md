# icp_laser
This program corrects small(?) localization errors using icp algorithm.

Maximum and minimum correction distances are set by setJumpParameters function. If our ICP guess is too close, no relocalization is needed. If it is too far away, it is probably wrong.

Pose update interval is set by setUpdateInterval function.

Some ICP parameters are set by setICPParameters function.

Some laser range data can be neglected if it is too far away. Max. distance and min. number of range data can be set by setLaserCloudParameters function.
