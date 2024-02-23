import { parseRos2idl as parse } from "./parseRos2idl";

const trackedObjectSchema = `================================================================================
IDL: autoware_auto_perception_msgs/msg/TrackedObjects
#include "autoware_auto_perception_msgs/msg/TrackedObject.idl"
#include "std_msgs/msg/Header.idl"

module autoware_auto_perception_msgs {
  module msg {
    @verbatim (language="comment", text=
      " This is the output of object tracking and the input to prediction.")
    struct TrackedObjects {
      std_msgs::msg::Header header;
      sequence<autoware_auto_perception_msgs::msg::TrackedObject> objects;
    };
  };
};

================================================================================
IDL: autoware_auto_perception_msgs/msg/TrackedObject
#include "autoware_auto_perception_msgs/msg/ObjectClassification.idl"
#include "autoware_auto_perception_msgs/msg/Shape.idl"
#include "autoware_auto_perception_msgs/msg/TrackedObjectKinematics.idl"
#include "unique_identifier_msgs/msg/UUID.idl"

module autoware_auto_perception_msgs {
  module msg {
    struct TrackedObject {
      unique_identifier_msgs::msg::UUID object_id;

      @range (min=0.0, max=1.0)
      float existence_probability;

      sequence<autoware_auto_perception_msgs::msg::ObjectClassification> classification;
      autoware_auto_perception_msgs::msg::TrackedObjectKinematics kinematics;

      autoware_auto_perception_msgs::msg::Shape shape;
    };
  };
};

================================================================================
IDL: autoware_auto_perception_msgs/msg/ObjectClassification
module autoware_auto_perception_msgs {
  module msg {
    module ObjectClassification_Constants {
      const uint8 UNKNOWN = 0;
      const uint8 CAR = 1;
      const uint8 TRUCK = 2;
      const uint8 BUS = 3;
      const uint8 TRAILER = 4;
      const uint8 MOTORCYCLE = 5;
      const uint8 BICYCLE = 6;
      const uint8 PEDESTRIAN = 7;
    };

    struct ObjectClassification {
      @verbatim (language="comment", text=
        " Valid values for the label field are provided in"
        " ObjectClassification_Constants.")
      uint8 label;

      @range (min=0.0, max=1.0)
      float probability;
    };
  };
};

================================================================================
IDL: autoware_auto_perception_msgs/msg/Shape
#include "geometry_msgs/msg/Polygon.idl"

module autoware_auto_perception_msgs {
  module msg {
    module Shape_Constants {
      const uint8 BOUNDING_BOX=0;
      const uint8 CYLINDER=1;
      const uint8 POLYGON=2;
    };

    struct Shape {
      @verbatim (language="comment", text=
        " Type of the shape")
      uint8 type;

      @verbatim (language="comment", text=
        " The contour of the shape (POLYGON)")
      geometry_msgs::msg::Polygon footprint;

      @verbatim (language="comment", text=
        " x: the length of the object (BOUNDING_BOX) or diameter (CYLINDER)"
        " y: the width of the object (BOUNDING_BOX)"
        " z: the overall height of the object")
      geometry_msgs::msg::Vector3 dimensions;
    };
  };
};

================================================================================
IDL: geometry_msgs/msg/Polygon
// generated from rosidl_adapter/resource/msg.idl.em
// with input from geometry_msgs/msg/Polygon.msg
// generated code does not contain a copyright notice

#include "geometry_msgs/msg/Point32.idl"

module geometry_msgs {
  module msg {
    @verbatim (language="comment", text=
      "A specification of a polygon where the first and last points are assumed to be connected")
    struct Polygon {
      sequence<geometry_msgs::msg::Point32> points;
    };
  };
};

================================================================================
IDL: geometry_msgs/msg/Point32
// generated from rosidl_adapter/resource/msg.idl.em
// with input from geometry_msgs/msg/Point32.msg
// generated code does not contain a copyright notice


module geometry_msgs {
  module msg {
    @verbatim (language="comment", text=
      "This contains the position of a point in free space(with 32 bits of precision)." "\n"
      "It is recommended to use Point wherever possible instead of Point32." "\n"
      "" "\n"
      "This recommendation is to promote interoperability." "\n"
      "" "\n"
      "This message is designed to take up less space when sending" "\n"
      "lots of points at once, as in the case of a PointCloud.")
    struct Point32 {
      float x;

      float y;

      float z;
    };
  };
};

================================================================================
IDL: autoware_auto_perception_msgs/msg/TrackedObjectKinematics
#include "geometry_msgs/msg/AccelWithCovariance.idl"
#include "geometry_msgs/msg/Point.idl"
#include "geometry_msgs/msg/Quaternion.idl"
#include "geometry_msgs/msg/TwistWithCovariance.idl"

module autoware_auto_perception_msgs {
  module msg {
    module TrackedObjectKinematics_Constants {
      /**
       * Only position is available, orientation is empty. Note that the shape can be an oriented
       * bounding box but the direction the object is facing is unknown, in which case
       * orientation should be empty.
       */
      const uint8 UNAVAILABLE = 0;
      /**
       * The orientation is determined only up to a sign flip. For instance, assume that cars are
       * longer than they are wide, and the perception pipeline can accurately estimate the
       * dimensions of a car. It should set the orientation to coincide with the major axis, with
       * the sign chosen arbitrarily, and use this tag to signify that the orientation could
       * point to the front or the back.
       */
      const uint8 SIGN_UNKNOWN = 1;
      /**
       * The full orientation is available. Use e.g. for machine-learning models that can
       * differentiate between the front and back of a vehicle.
       */
      const uint8 AVAILABLE = 2;
    };

    struct TrackedObjectKinematics {
      @verbatim (language="comment", text=
      " Pose covariance is always provided by tracking.")
      geometry_msgs::msg::PoseWithCovariance pose_with_covariance;

      uint8 orientation_availability;

      geometry_msgs::msg::TwistWithCovariance twist_with_covariance;

      geometry_msgs::msg::AccelWithCovariance acceleration_with_covariance;

      @value (default=False)
      boolean is_stationary;
    };
  };
};

================================================================================
IDL: geometry_msgs/msg/AccelWithCovariance
// generated from rosidl_adapter/resource/msg.idl.em
// with input from geometry_msgs/msg/AccelWithCovariance.msg
// generated code does not contain a copyright notice

#include "geometry_msgs/msg/Accel.idl"

module geometry_msgs {
  module msg {
    typedef double double__36[36];
    @verbatim (language="comment", text=
      "This expresses acceleration in free space with uncertainty.")
    struct AccelWithCovariance {
      geometry_msgs::msg::Accel accel;

      @verbatim (language="comment", text=
        "Row-major representation of the 6x6 covariance matrix" "\n"
        "The orientation parameters use a fixed-axis representation." "\n"
        "In order, the parameters are:" "\n"
        "(x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)")
      double__36 covariance;
    };
  };
};

================================================================================
IDL: geometry_msgs/msg/Accel
// generated from rosidl_adapter/resource/msg.idl.em
// with input from geometry_msgs/msg/Accel.msg
// generated code does not contain a copyright notice

#include "geometry_msgs/msg/Vector3.idl"

module geometry_msgs {
  module msg {
    @verbatim (language="comment", text=
      "This expresses acceleration in free space broken into its linear and angular parts.")
    struct Accel {
      geometry_msgs::msg::Vector3 linear;

      geometry_msgs::msg::Vector3 angular;
    };
  };
};

================================================================================
IDL: geometry_msgs/msg/Vector3
// generated from rosidl_adapter/resource/msg.idl.em
// with input from geometry_msgs/msg/Vector3.msg
// generated code does not contain a copyright notice


module geometry_msgs {
  module msg {
    @verbatim (language="comment", text=
      "This represents a vector in free space.")
    struct Vector3 {
      @verbatim (language="comment", text=
        "This is semantically different than a point." "\n"
        "A vector is always anchored at the origin." "\n"
        "When a transform is applied to a vector, only the rotational component is applied.")
      double x;

      double y;

      double z;
    };
  };
};

================================================================================
IDL: geometry_msgs/msg/Point
// generated from rosidl_adapter/resource/msg.idl.em
// with input from geometry_msgs/msg/Point.msg
// generated code does not contain a copyright notice


module geometry_msgs {
  module msg {
    @verbatim (language="comment", text=
      "This contains the position of a point in free space")
    struct Point {
      double x;

      double y;

      double z;
    };
  };
};

================================================================================
IDL: geometry_msgs/msg/Quaternion
// generated from rosidl_adapter/resource/msg.idl.em
// with input from geometry_msgs/msg/Quaternion.msg
// generated code does not contain a copyright notice


module geometry_msgs {
  module msg {
    @verbatim (language="comment", text=
      "This represents an orientation in free space in quaternion form.")
    struct Quaternion {
      @default (value=0.0)
      double x;

      @default (value=0.0)
      double y;

      @default (value=0.0)
      double z;

      @default (value=1.0)
      double w;
    };
  };
};

================================================================================
IDL: geometry_msgs/msg/TwistWithCovariance
// generated from rosidl_adapter/resource/msg.idl.em
// with input from geometry_msgs/msg/TwistWithCovariance.msg
// generated code does not contain a copyright notice

#include "geometry_msgs/msg/Twist.idl"

module geometry_msgs {
  module msg {
    typedef double double__36[36];
    @verbatim (language="comment", text=
      "This expresses velocity in free space with uncertainty.")
    struct TwistWithCovariance {
      geometry_msgs::msg::Twist twist;

      @verbatim (language="comment", text=
        "Row-major representation of the 6x6 covariance matrix" "\n"
        "The orientation parameters use a fixed-axis representation." "\n"
        "In order, the parameters are:" "\n"
        "(x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)")
      double__36 covariance;
    };
  };
};

================================================================================
IDL: geometry_msgs/msg/Twist
// generated from rosidl_adapter/resource/msg.idl.em
// with input from geometry_msgs/msg/Twist.msg
// generated code does not contain a copyright notice

#include "geometry_msgs/msg/Vector3.idl"

module geometry_msgs {
  module msg {
    @verbatim (language="comment", text=
      "This expresses velocity in free space broken into its linear and angular parts.")
    struct Twist {
      geometry_msgs::msg::Vector3 linear;

      geometry_msgs::msg::Vector3 angular;
    };
  };
};

================================================================================
IDL: unique_identifier_msgs/msg/UUID
// generated from rosidl_adapter/resource/msg.idl.em
// with input from unique_identifier_msgs/msg/UUID.msg
// generated code does not contain a copyright notice


module unique_identifier_msgs {
  module msg {
    typedef uint8 uint8__16[16];
    @verbatim (language="comment", text=
      "A universally unique identifier (UUID)." "\n"
      "" "\n"
      " http://en.wikipedia.org/wiki/Universally_unique_identifier" "\n"
      " http://tools.ietf.org/html/rfc4122.html")
    struct UUID {
      uint8__16 uuid;
    };
  };
};

================================================================================
IDL: std_msgs/msg/Header
// generated from rosidl_adapter/resource/msg.idl.em
// with input from std_msgs/msg/Header.msg
// generated code does not contain a copyright notice

#include "builtin_interfaces/msg/Time.idl"

module std_msgs {
  module msg {
    @verbatim (language="comment", text=
      "Standard metadata for higher-level stamped data types." "\n"
      "This is generally used to communicate timestamped data" "\n"
      "in a particular coordinate frame.")
    struct Header {
      @verbatim (language="comment", text=
        "Two-integer timestamp that is expressed as seconds and nanoseconds.")
      builtin_interfaces::msg::Time stamp;

      @verbatim (language="comment", text=
        "Transform frame with which this data is associated.")
      string frame_id;
    };
  };
};

================================================================================
IDL: builtin_interfaces/msg/Time
// generated from rosidl_adapter/resource/msg.idl.em
// with input from builtin_interfaces/msg/Time.msg
// generated code does not contain a copyright notice


module builtin_interfaces {
  module msg {
    @verbatim (language="comment", text=
      "This message communicates ROS Time defined here:" "\n"
      "https://design.ros2.org/articles/clock_and_time.html")
    struct Time {
      @verbatim (language="comment", text=
        "The seconds component, valid over all int32 values.")
      int32 sec;

      @verbatim (language="comment", text=
        "The nanoseconds component, valid in the range [0, 10e9).")
      uint32 nanosec;
    };
  };
};
`;

describe("ros2idl large autoware message (TrackedMessages)", () => {
  it("should parse the message without failing", () => {
    const parsed = parse(trackedObjectSchema);
    expect(parsed).toMatchSnapshot("autoware-TrackedObjects");
  });
});
