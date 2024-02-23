# rosmsg-msgs-common

[![npm version](https://img.shields.io/npm/v/@foxglove/rosmsg-msgs-common.svg?style=flat)](https://www.npmjs.com/package/@foxglove/rosmsg-msgs-common)

This library exports a map of ROS 1 and ROS 2 datatype string keys to [@foxglove/message-definition](https://github.com/foxglove/message-definition) `MessageDefinition` values for most common ROS 1 and ROS 2 message definitions. The ROS 1 message definitions were extracted from the `ros:noetic-robot-focal` Docker container using the `gendeps --cat` command. ROS 2 message definitions were extracted from [rcl_interfaces](https://github.com/ros2/rcl_interfaces), [common_interfaces](https://github.com/ros2/common_interfaces), and [unique_identifier_msgs](https://github.com/ros2/unique_identifier_msgs) repository branches using the [gendeps2](https://github.com/foxglove/rosmsg/blob/main/src/gendeps2.ts) utility.

## License

@foxglove/rosmsg-msgs-common is licensed under the [MIT License](https://opensource.org/licenses/MIT).

## Releasing

1. Run `yarn version --[major|minor|patch]` to bump version
2. Run `git push && git push --tags` to push new tag
3. GitHub Actions will take care of the rest

## Stay in touch

Join our [Slack channel](https://foxglove.dev/join-slack) to ask questions, share feedback, and stay up to date on what our team is working on.
