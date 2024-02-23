# @foxglove/rosmsg

> _ROS1 and ROS2 message definition parser_

[![npm version](https://img.shields.io/npm/v/@foxglove/rosmsg.svg?style=flat)](https://www.npmjs.com/package/@foxglove/rosmsg)

## Introduction

[The Robot Operating System (ROS)](https://www.ros.org/) defines a simplified message description language for describing data types. This library parses those message definitions and can round trip them back into a canonical string format suitable for checksum generation. The parsed definitions are useful for serialization or deserialization when paired with other libraries.

This library supports both [ROS1](http://wiki.ros.org/msg), [ROS 2](https://docs.ros.org/en/galactic/Concepts/About-ROS-Interfaces.html), and the [ROS 2 IDL subset](https://design.ros2.org/articles/idl_interface_definition.html)message definitions.

## Usage

```Typescript
import { parse, parseRos2idl, stringify } from "@foxglove/rosmsg";

const definitionStr = `# geometry_msgs/msg/Pose
geometry_msgs/msg/Point position
geometry_msgs/msg/Quaternion orientation

===
MSG: geometry_msgs/msg/Point
float64 x
float64 y
float64 z

===
MSG: geometry_msgs/msg/Quaternion
float64 x
float64 y
float64 z
float64 w`;

const messageDefinition = parse(definitionStr);
const messageDefinition = parse(definitionStr, {ros2: true}); // for ROS 2 definitions

// stringify(messageDefinition) will return a canonical string, similar to
// _definitionStr_

// ROS2IDL equivalent example
const ros2idlDefinitionStr = `
================================================================================
IDL: geometry_msgs/msg/Pose

module geometry_msgs {
  module msg {
    struct Pose {
      geometry_msgs::msg::Point position;
      geometry_msgs::msg::Quaternion orientation;
    };
  };
};

================================================================================
IDL: geometry_msgs/msg/Point

module geometry_msgs {
  module msg {
    struct Point {
      double x;
      double y;
      double z;
    };
  };
};

================================================================================
IDL: geometry_msgs/msg/Quaternion

module geometry_msgs {
  module msg {
    struct Quaternion {
      double x;
      double y;
      double z;
      double w;
    };
  };
};
`;

const messageDefinition = parseRos2idl(ros2idlDefinitionStr);

// print the parsed message definition structure
console.log(JSON.stringify(messageDefinition, null, 2));
```

Prints:

```JSON
[
  {
    "definitions": [
      {
        "type": "geometry_msgs/msg/Point",
        "isArray": false,
        "name": "position",
        "isComplex": true
      },
      {
        "type": "geometry_msgs/msg/Quaternion",
        "isArray": false,
        "name": "orientation",
        "isComplex": true
      }
    ]
  },
  {
    "name": "geometry_msgs/msg/Point",
    "definitions": [
      {
        "type": "float64",
        "isArray": false,
        "name": "x",
        "isComplex": false
      },
      {
        "type": "float64",
        "isArray": false,
        "name": "y",
        "isComplex": false
      },
      {
        "type": "float64",
        "isArray": false,
        "name": "z",
        "isComplex": false
      }
    ]
  },
  {
    "name": "geometry_msgs/msg/Quaternion",
    "definitions": [
      {
        "type": "float64",
        "isArray": false,
        "name": "x",
        "isComplex": false
      },
      {
        "type": "float64",
        "isArray": false,
        "name": "y",
        "isComplex": false
      },
      {
        "type": "float64",
        "isArray": false,
        "name": "z",
        "isComplex": false
      },
      {
        "type": "float64",
        "isArray": false,
        "name": "w",
        "isComplex": false
      }
    ]
  }
]
```

## License

@foxglove/rosmsg is licensed under the [MIT License](https://opensource.org/licenses/MIT).

## Releasing

1. Run `yarn version --[major|minor|patch]` to bump version
2. Run `git push && git push --tags` to push new tag
3. GitHub Actions will take care of the rest

## Stay in touch

Join our [Slack channel](https://foxglove.dev/join-slack) to ask questions, share feedback, and stay up to date on what our team is working on.
