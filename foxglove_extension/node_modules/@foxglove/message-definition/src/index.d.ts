/**
 * Valid JavaScript types for constants.
 */
export type ConstantValue = string | number | bigint | boolean | undefined;

/**
 * Valid JavaScript types for default field values.
 */
export type DefaultValue =
  | string
  | number
  | bigint
  | boolean
  | string[]
  | number[]
  | bigint[]
  | boolean[]
  | undefined;

/**
 * A single field in a message definition.
 *
 * Example:
 * In the protobufs programming guide (https://protobuf.dev/programming-guides/proto3), the
 * following message definition is given:
 *
 * ```protobuf
 * syntax = "proto3";
 *
 * message SearchRequest {
 *   string query = 1;
 *   int32 page_number = 2;
 *   int32 result_per_page = 3;
 * }
 * ```
 *
 * The three fields in this message definition would be represented as follows:
 * ```typescript
 * { type: "string", name: "query" }
 * { type: "int32", name: "page_number" }
 * { type: "int32", name: "result_per_page" }
 * ```
 */
export type MessageDefinitionField = {
  /**
   * Type name of this field. Valid primitive type names will differ across each
   * interface definition language, and complex types are often user-defined.
   * Examples: "string", "int32", "geometry_msgs/Point", "foxglove.Grid".
   */
  type: string;
  /**
   * Name of this field.
   */
  name: string;
  /**
   * Set to true if this field's type is a user-defined type (i.e. not a
   * primitive type, a custom struct).
   */
  isComplex?: boolean;

  /**
   * Set to true if this field is an array. For example, the following Protobuf
   * field:
   *
   * ```protobuf
   * repeated Result results = 1;
   * ```
   *
   * Would be represented as:
   * ```typescript
   * { type: "Result", name: "results", isArray: true }
   * ```
   */
  isArray?: boolean;
  /**
   * Defines the array length of this field for fixed-length arrays. Only set if
   * `isArray` is true. For example, the following ROS 1 message definition
   * field:
   *
   * ```
   * float64[12] projection_matrix
   * ```
   *
   * Would be represented as:
   * ```typescript
   * { type: "float64", name: "projection_matrix", isArray: true, arrayLength: 12 }
   * ```
   */
  arrayLength?: number | undefined;

  /**
   * Set to true if this field defines a constant and is not directly part of
   * the message definition (i.e. it is not a field in the message definition
   * and does not affect serialization or deserialization). For example, the
   * following ROS 1 message definition field:
   *
   * ```
   * float64 PI = 3.14159
   * ```
   *
   * Would be represented as:
   * ```typescript
   * { type: "float64", name: "PI", isConstant: true, value: 3.14159, valueText: "3.14159" }
   * ```
   */
  isConstant?: boolean;
  /**
   * The value of this constant. Only set if `isConstant` is true. The set of
   * allowed types for constants will differ across each interface definition
   * language.
   */
  value?: ConstantValue;
  /**
   * The original text representation of this constant's value. Only set if
   * `isConstant` is true. For example, the following ROS 1 message definition
   * field:
   *
   * ```
   * bool ALIVE=True
   * ```
   *
   * Would be represented as:
   * ```typescript
   * { type: "bool", name: "ALIVE", isConstant: true, value: true, valueText: "True" }
   */
  valueText?: string;

  /**
   * If set, this defines a maximum upper bound on string length of this field.
   * Only set if `type` is "string". For example, the following ROS 2 message
   * definition field:
   *
   * ```
   * string<=5 bar
   * ```
   *
   * Would be represented as:
   *
   * ```typescript
   * { type: "string", name: "bar", upperBound: 5 }
   * ```
   */
  upperBound?: number;
  /**
   * If set, this defines a maximum upper bound on array length of this field.
   * Only set if `isArray` is true. For example, the following ROS 2 message
   * definition field:
   *
   * ```
   * int32[<=5] foo
   * ```
   *
   * Would be represented as:
   *
   * ```typescript
   * { type: "int32", name: "foo", isArray: true, arrayUpperBound: 5 }
   * ```
   */
  arrayUpperBound?: number;
  /**
   * If set, this defines a default value for this field. For example, the
   * following ROS 2 message definition field:
   *
   * ```
   * int32 foo=42
   * ```
   *
   * Would be represented as:
   * ```typescript
   * { type: "int32", name: "foo", defaultValue: 42 }
   * ```
   *
   * And would only effect serialization if the field is not set, having no
   * effect on deserialization. The effect this field has on serialization or
   * deserialization will differ across each interface definition language.
   */
  defaultValue?: DefaultValue;
};

/**
 * A message definition, i.e. a struct definition containing fields in some
 * interface definition language. Examples: Protobuf .proto, ROS 1 .msg,
 * ROS 2 .msg, IDL .idl, JSON Schema .json, etc.
 */
export type MessageDefinition = {
  /**
   * An optional name for this message definition. It's common for top-level
   * message definitions to be unnamed, and nested message definitions (custom
   * complex types) to be named.
   */
  name?: string;
  /**
   * The ordered list of fields in this message definition.
   */
  definitions: MessageDefinitionField[];
};
