# Velodyne LIDAR scan packets.

Header           header         # standard ROS message header
VelodynePacket[] packets        # vector of raw packets

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.
#
# sequence ID: consecutively increasing ID
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: velodyne_msgs/VelodynePacket
# Raw Velodyne LIDAR packet.
time stamp              # packet timestamp
uint8[1206] data        # packet contents
