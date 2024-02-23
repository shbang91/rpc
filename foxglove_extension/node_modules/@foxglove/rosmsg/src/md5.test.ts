import { md5 } from "./md5";
import { parse } from "./parse";

type Md5Test = [name: string, msgDef: string, expected: string];

const md5Tests: Md5Test[] = [
  ["std_msgs/Bool", "bool data", "8b94c1b53db61fb6aed406028ad6332a"],
  ["std_msgs/Int8", "int8 data", "27ffa0c9c4b8fb8492252bcad9e5c57b"],
  ["Int8Array", "int8[] data", "ac9c931aaf6ce145ea0383362e83c70b"],
  ["Int8FixedArray", "int8[7] data", "2868efa54c3e6c80cd3e096af5102940"],
  ["BoolFalseConstant", "bool A=False", "d3011fbf97518e43e51a9ef7f5f352f1"],
  ["BoolZeroConstant", "bool A=0", "8a08473e3b7d762244201aa8a3f42ed9"],
  ["Int64WeirdConstant", "int64 A = 0000000000000001", "b3264adb244f3230420d804c5ed3b417"],
  ["ConstantAndField", "bool data\nint32 A=1", "9c4674e31b60282f7c0f4c41b0147d3c"],
  [
    "Unformatted",
    `uint32 foo = 55
int32 bar=-11 # Comment # another comment
string test
 float32 baz= \t	 -32.25
bool someBoolean = 0       ${""}
  float64[] values
string fooStr = Foo
string EXAMPLE="#comments" are ignored, and leading and trailing whitespace removed`,
    "12ddded42eac8f049635e8f51ee50077",
  ],
  [
    "std_msgs/Header",
    `# Standard metadata for higher-level stamped data types.
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
string frame_id`,
    "2176decaecbce78abc3b96ef049fabed",
  ],
  [
    "sensor_msgs/PointCloud2",
    `# This message holds a collection of N-dimensional points, which may
  # contain additional information such as normals, intensity, etc. The
  # point data is stored as a binary blob, its layout described by the
  # contents of the "fields" array.

  # The point cloud data may be organized 2d (image-like) or 1d
  # (unordered). Point clouds organized as 2d images may be produced by
  # camera depth sensors such as stereo or time-of-flight.

  # Time of sensor data acquisition, and the coordinate frame ID (for 3d
  # points).
  Header header

  # 2D structure of the point cloud. If the cloud is unordered, height is
  # 1 and width is the length of the point cloud.
  uint32 height
  uint32 width

  # Describes the channels and their layout in the binary data blob.
  PointField[] fields

  bool    is_bigendian # Is this data bigendian?
  uint32  point_step   # Length of a point in bytes
  uint32  row_step     # Length of a row in bytes
  uint8[] data         # Actual point data, size is (row_step*height)

  bool is_dense        # True if there are no invalid points

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
  MSG: sensor_msgs/PointField
  # This message holds the description of one point entry in the
  # PointCloud2 message format.
  uint8 INT8    = 1
  uint8 UINT8   = 2
  uint8 INT16   = 3
  uint8 UINT16  = 4
  uint8 INT32   = 5
  uint8 UINT32  = 6
  uint8 FLOAT32 = 7
  uint8 FLOAT64 = 8

  string name      # Name of field
  uint32 offset    # Offset from start of point struct
  uint8  datatype  # Datatype enumeration, see above
  uint32 count     # How many elements in the field
  `,
    "1158d486dd51d683ce2f1be655c3c181",
  ],
];

describe("md5", () => {
  it.each(md5Tests)("should checksum %s", (_name, msgDef, expected) => {
    expect(md5(parse(msgDef))).toBe(expected);
  });
});
