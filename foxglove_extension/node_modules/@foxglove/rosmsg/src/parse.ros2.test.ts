// This file incorporates work covered by the following copyright and
// permission notice:
//
//   Copyright 2018-2021 Cruise LLC
//
//   This source code is licensed under the Apache License, Version 2.0,
//   found at http://www.apache.org/licenses/LICENSE-2.0
//   You may not use this file except in compliance with the License.

import { parse } from "./parse";

describe("parseMessageDefinition", () => {
  it("parses a single field from a single message", () => {
    const types = parse("string name", { ros2: true });
    expect(types).toEqual([
      {
        definitions: [
          {
            arrayLength: undefined,
            isArray: false,
            isComplex: false,
            name: "name",
            type: "string",
          },
        ],
        name: undefined,
      },
    ]);
  });

  it.each([
    [`[a,b]`, ["a", "b"]],
    [`[a,b]#`, ["a", "b"]],
    [`[a,'b#c']`, ["a", "b#c"]],
    [`[a,'b]']`, ["a", "b]"]],
    [`#comment`, undefined],
    [` #comment`, undefined],
  ])("parses string array default value %s", (literal, value) => {
    expect(parse(`string[] name ${literal}`, { ros2: true })).toEqual([
      {
        definitions: [
          {
            arrayLength: undefined,
            isArray: true,
            isComplex: false,
            name: "name",
            type: "string",
            defaultValue: value,
          },
        ],
        name: undefined,
      },
    ]);
  });

  it.each([`[,]`, `[,a]`, `[a,']`, `[`, `]`, `[a,b]x`])(
    "rejects invalid string array literal %s, but accepts it as a string literal",
    (literal) => {
      expect(() => parse(`string[] name ${literal}`, { ros2: true })).toThrow(
        /Expected comma or end of array|Expected array element before comma|Array must start with \[ and end with \]/,
      );

      expect(parse(`string name ${literal}`, { ros2: true })).toEqual([
        {
          definitions: [
            {
              arrayLength: undefined,
              isArray: false,
              isComplex: false,
              name: "name",
              type: "string",
              defaultValue: literal,
            },
          ],
          name: undefined,
        },
      ]);

      expect(parse(`string name ${literal}#comment`, { ros2: true })).toEqual([
        {
          definitions: [
            {
              arrayLength: undefined,
              isArray: false,
              isComplex: false,
              name: "name",
              type: "string",
              defaultValue: literal,
            },
          ],
          name: undefined,
        },
      ]);

      expect(parse(`string name ${literal} #comment`, { ros2: true })).toEqual([
        {
          definitions: [
            {
              arrayLength: undefined,
              isArray: false,
              isComplex: false,
              name: "name",
              type: "string",
              defaultValue: literal,
            },
          ],
          name: undefined,
        },
      ]);
    },
  );
  it("rejects valid tokens that don't fully match a parser rule", () => {
    expect(() => parse("abc", { ros2: true })).toThrow("Could not parse line: 'abc'");
  });

  it.each(["A", "aB", "a_", "_a", "a__b", "3a"])("rejects invalid field name %s", (name) => {
    expect(() => parse(`string ${name}`, { ros2: true })).toThrow();
  });
  it.each(["a", "aB", "A_", "_A", "A__B", "3A"])("rejects invalid constant name %s", (name) => {
    expect(() => parse(`string ${name} = 'x'`, { ros2: true })).toThrow();
  });
  it.each(["a", "foo_bar", "foo1_2bar"])("accepts valid field name %s", (name) => {
    expect(parse(`string ${name}`, { ros2: true })).toEqual([
      {
        definitions: [
          { arrayLength: undefined, isArray: false, isComplex: false, name, type: "string" },
        ],
        name: undefined,
      },
    ]);
  });
  it.each(["A", "A_B", "FOO1_2BAR"])("accepts valid constant name %s", (name) => {
    expect(parse(`string ${name} = 'x'`, { ros2: true })).toEqual([
      {
        definitions: [{ name, type: "string", isConstant: true, value: "x", valueText: "'x'" }],
        name: undefined,
      },
    ]);
  });

  it("resolves unqualified names", () => {
    const messageDefinition = `
      Point[] points
      ============
      MSG: geometry_msgs/Point
      float64 x
    `;
    const types = parse(messageDefinition, { ros2: true });
    expect(types).toEqual([
      {
        definitions: [
          {
            isArray: true,
            isComplex: true,
            name: "points",
            type: "geometry_msgs/Point",
          },
        ],
        name: undefined,
      },
      {
        definitions: [
          {
            isArray: false,
            isComplex: false,
            name: "x",
            type: "float64",
          },
        ],
        name: "geometry_msgs/Point",
      },
    ]);
  });

  it("normalizes aliases", () => {
    const types = parse("char x\nbyte y", { ros2: true });
    expect(types).toEqual([
      {
        definitions: [
          {
            isArray: false,
            isComplex: false,
            name: "x",
            type: "uint8",
          },
          {
            isArray: false,
            isComplex: false,
            name: "y",
            type: "int8",
          },
        ],
        name: undefined,
      },
    ]);
  });

  it("ignores comment lines", () => {
    const messageDefinition = `
    # your first name goes here
    string first_name

    # last name here
    ### foo bar baz?
    string last_name
    `;
    const types = parse(messageDefinition, { ros2: true });
    expect(types).toEqual([
      {
        definitions: [
          {
            isArray: false,
            isComplex: false,
            name: "first_name",
            type: "string",
          },
          {
            isArray: false,
            isComplex: false,
            name: "last_name",
            type: "string",
          },
        ],
        name: undefined,
      },
    ]);
  });

  it("parses variable length string array", () => {
    const types = parse("string[] names", { ros2: true });
    expect(types).toEqual([
      {
        definitions: [
          {
            isArray: true,
            isComplex: false,
            name: "names",
            type: "string",
          },
        ],
        name: undefined,
      },
    ]);
  });

  it("parses fixed length string array", () => {
    const types = parse("string[3] names", { ros2: true });
    expect(types).toEqual([
      {
        definitions: [
          {
            arrayLength: 3,
            isArray: true,
            isComplex: false,
            name: "names",
            type: "string",
          },
        ],
        name: undefined,
      },
    ]);
  });

  it("parses nested complex types", () => {
    const messageDefinition = `
    string username
    Account account
    ============
    MSG: custom_type/Account
    string name
    uint16 id
    `;
    const types = parse(messageDefinition, { ros2: true });
    expect(types).toEqual([
      {
        definitions: [
          {
            isArray: false,
            isComplex: false,
            name: "username",
            type: "string",
          },
          {
            isArray: false,
            isComplex: true,
            name: "account",
            type: "custom_type/Account",
          },
        ],
        name: undefined,
      },
      {
        definitions: [
          {
            isArray: false,
            isComplex: false,
            name: "name",
            type: "string",
          },
          {
            isArray: false,
            isComplex: false,
            name: "id",
            type: "uint16",
          },
        ],
        name: "custom_type/Account",
      },
    ]);
  });

  it("returns constants", () => {
    const messageDefinition = `
      uint32 FOO = 55
      int32 BAR=-11 # Comment! # another comment
      float32 BAZ= \t -32.25
      bool SOME_BOOLEAN = 0
      string FOO_STR = 'Foo'    ${""}
      string EMPTY=
      string EXAMPLE="#comments" # are handled properly
      string UNQUOTED= Bar
      string UNQUOTEDSPACE = Bar Foo
      string UNQUOTEDSPECIAL = afse_doi@f4!  :834$%G$%
      string BLANK=
      string BLANKCOMMENT=# Blank with comment
      string BLANKSPACECOMMENT= # Blank with comment after space
      string ESCAPED_QUOTE = \\'a#comment
    `;
    const types = parse(messageDefinition, { ros2: true });
    expect(types).toEqual([
      {
        definitions: [
          {
            name: "FOO",
            type: "uint32",
            isConstant: true,
            value: 55,
            valueText: "55",
          },
          {
            name: "BAR",
            type: "int32",
            isConstant: true,
            value: -11,
            valueText: "-11",
          },
          {
            name: "BAZ",
            type: "float32",
            isConstant: true,
            value: -32.25,
            valueText: "-32.25",
          },
          {
            name: "SOME_BOOLEAN",
            type: "bool",
            isConstant: true,
            value: false,
            valueText: "0",
          },
          {
            name: "FOO_STR",
            type: "string",
            isConstant: true,
            value: "Foo",
            valueText: "'Foo'",
          },
          {
            name: "EMPTY",
            type: "string",
            isConstant: true,
            value: "",
            valueText: "",
          },
          {
            name: "EXAMPLE",
            type: "string",
            isConstant: true,
            value: "#comments",
            valueText: '"#comments"',
          },
          {
            name: "UNQUOTED",
            type: "string",
            isConstant: true,
            value: "Bar",
            valueText: "Bar",
          },
          {
            name: "UNQUOTEDSPACE",
            type: "string",
            isConstant: true,
            value: "Bar Foo",
            valueText: "Bar Foo",
          },
          {
            name: "UNQUOTEDSPECIAL",
            type: "string",
            isConstant: true,
            value: "afse_doi@f4!  :834$%G$%",
            valueText: "afse_doi@f4!  :834$%G$%",
          },
          {
            name: "BLANK",
            type: "string",
            isConstant: true,
            value: "",
            valueText: "",
          },
          {
            name: "BLANKCOMMENT",
            type: "string",
            isConstant: true,
            value: "",
            valueText: "",
          },
          {
            name: "BLANKSPACECOMMENT",
            type: "string",
            isConstant: true,
            value: "",
            valueText: "",
          },
          {
            name: "ESCAPED_QUOTE",
            type: "string",
            isConstant: true,
            value: "'a",
            valueText: "\\'a",
          },
        ],
        name: undefined,
      },
    ]);
  });

  it("works with python boolean values", () => {
    const messageDefinition = `
      bool ALIVE=True
      bool DEAD=False
    `;
    const types = parse(messageDefinition, { ros2: true });
    expect(types).toEqual([
      {
        definitions: [
          {
            name: "ALIVE",
            type: "bool",
            isConstant: true,
            value: true,
            valueText: "True",
          },
          {
            name: "DEAD",
            type: "bool",
            isConstant: true,
            value: false,
            valueText: "False",
          },
        ],
        name: undefined,
      },
    ]);
  });

  it("handles type names for fields", () => {
    expect(parse(`time time`)).toEqual([
      {
        definitions: [
          {
            name: "time",
            type: "time",
            isArray: false,
            isComplex: false,
          },
        ],
        name: undefined,
      },
    ]);

    expect(parse(`time time_ref`)).toEqual([
      {
        definitions: [
          {
            name: "time_ref",
            type: "time",
            isArray: false,
            isComplex: false,
          },
        ],
        name: undefined,
      },
    ]);

    expect(
      parse(`
    true true
    ============
    MSG: custom/true
    bool false
    `),
    ).toEqual([
      {
        definitions: [
          {
            name: "true",
            type: "custom/true",
            isArray: false,
            isComplex: true,
          },
        ],
        name: undefined,
      },
      {
        definitions: [
          {
            name: "false",
            type: "bool",
            isArray: false,
            isComplex: false,
          },
        ],
        name: "custom/true",
      },
    ]);
  });

  it("allows numbers in package names", () => {
    expect(
      parse(
        `
    abc1/Foo2 value0
    ==========
    MSG: abc1/Foo2
    int32 data
    `,
        { ros2: true },
      ),
    ).toEqual([
      {
        definitions: [{ isArray: false, isComplex: true, name: "value0", type: "abc1/Foo2" }],
        name: undefined,
      },
      {
        definitions: [{ isArray: false, isComplex: false, name: "data", type: "int32" }],
        name: "abc1/Foo2",
      },
    ]);
  });

  it.each(["\\", "hi\\", String.raw`'abc\'`])("rejects invalid string literal %s", (str) => {
    expect(() => parse(`string x ${str}`, { ros2: true })).toThrow("Could not parse line");
    // "Invalid field name" is not an ideal error message but that's ok. It just means the = parsed
    // as potentially part of a default value, but the name is a constant name (uppercase) rather
    // than a field name.
    expect(() => parse(`string X = ${str}`, { ros2: true })).toThrow(
      /Invalid field name|Could not parse line/,
    );
  });

  it.each([
    [`x`, `x`],
    [`\\b`, `\b`],
    [`\\p`, `\\p`],
    [`\\foo`, `\foo`],
    [`[a\\,b]`, `[a\\,b]`],
  ])("parses unquoted string default/constant value %s", (literal, value) => {
    expect(parse(`string x ${literal}`, { ros2: true })).toEqual([
      {
        definitions: [
          { isArray: false, isComplex: false, name: "x", type: "string", defaultValue: value },
        ],
        name: undefined,
      },
    ]);
    expect(parse(`string X = ${literal}`, { ros2: true })).toEqual([
      {
        definitions: [{ isConstant: true, name: "X", type: "string", valueText: literal, value }],
        name: undefined,
      },
    ]);
  });

  it.each([
    String.raw`'hello\'\"\a\b\f\n\r\t\v\\\012\019\x10\u1010\U0002F804'`,
    String.raw`'hello\'"\a\b\f\n\r\t\v\\\012\019\x10\u1010\U0002F804'`,
    String.raw`"hello\'\"\a\b\f\n\r\t\v\\\012\019\x10\u1010\U0002F804"`,
    String.raw`"hello'\"\a\b\f\n\r\t\v\\\012\019\x10\u1010\U0002F804"`,
    String.raw`hello\'\"\a\b\f\n\r\t\v\\\012\019\x10\u1010\U0002F804`,
    String.raw`hello'"\a\b\f\n\r\t\v\\\012\019\x10\u1010\U0002F804`,
  ])("parses escapes in string constant/default value `%s`", (str) => {
    const expected = `hello'"\x07\b\f\n\r\t\v\\${String.fromCodePoint(
      0o12,
    )}\x019\x10\u1010${String.fromCodePoint(0x2f804)}`;
    expect(parse(`string x ${str}`, { ros2: true })).toEqual([
      {
        definitions: [
          { name: "x", type: "string", defaultValue: expected, isArray: false, isComplex: false },
        ],
      },
    ]);
    expect(parse(`string X = ${str} #comment`, { ros2: true })).toEqual([
      {
        definitions: [
          { name: "X", type: "string", isConstant: true, value: expected, valueText: str },
        ],
      },
    ]);
    expect(parse(`string X =${str}`, { ros2: true })).toEqual([
      {
        definitions: [
          { name: "X", type: "string", isConstant: true, value: expected, valueText: str },
        ],
      },
    ]);
    expect(parse(`string X =${str}#comment`, { ros2: true })).toEqual([
      {
        definitions: [
          { name: "X", type: "string", isConstant: true, value: expected, valueText: str },
        ],
      },
    ]);
  });

  it.each(["int32 x abc", "bool x abc"])("rejects literals of incorrect type: %s", (line) => {
    expect(() => parse(line, { ros2: true })).toThrow();
  });

  it("parses default values", () => {
    const messageDefinition = `
      int8 a 0
      int8 b -1
      bool c false
      bool d False
      bool e true
      bool f True
      string g "hello"
      string h 'hello'
      string i "'hello'"
      string j '"hello"'
      string k "\\"hello\\""
      string l '\\'hello\\''
      string m \\foo
    `;
    const types = parse(messageDefinition, { ros2: true });
    expect(types).toEqual([
      {
        definitions: [
          {
            name: "a",
            type: "int8",
            defaultValue: 0,
            isArray: false,
            isComplex: false,
          },
          {
            name: "b",
            type: "int8",
            defaultValue: -1,
            isArray: false,
            isComplex: false,
          },
          {
            name: "c",
            type: "bool",
            defaultValue: false,
            isArray: false,
            isComplex: false,
          },
          {
            name: "d",
            type: "bool",
            defaultValue: false,
            isArray: false,
            isComplex: false,
          },
          {
            name: "e",
            type: "bool",
            defaultValue: true,
            isArray: false,
            isComplex: false,
          },
          {
            name: "f",
            type: "bool",
            defaultValue: true,
            isArray: false,
            isComplex: false,
          },
          {
            name: "g",
            type: "string",
            defaultValue: "hello",
            isArray: false,
            isComplex: false,
          },
          {
            name: "h",
            type: "string",
            defaultValue: "hello",
            isArray: false,
            isComplex: false,
          },
          {
            name: "i",
            type: "string",
            defaultValue: `'hello'`,
            isArray: false,
            isComplex: false,
          },
          {
            name: "j",
            type: "string",
            defaultValue: `"hello"`,
            isArray: false,
            isComplex: false,
          },
          {
            name: "k",
            type: "string",
            defaultValue: `"hello"`,
            isArray: false,
            isComplex: false,
          },
          {
            name: "l",
            type: "string",
            defaultValue: "'hello'",
            isArray: false,
            isComplex: false,
          },
          {
            name: "m",
            type: "string",
            defaultValue: "\foo",
            isArray: false,
            isComplex: false,
          },
        ],
        name: undefined,
      },
    ]);
  });

  it("parses rcl_interfaces/msg/Log", () => {
    const messageDefinition = `##
## Severity level constants
## 
## These logging levels follow the Python Standard
## https://docs.python.org/3/library/logging.html#logging-levels
## And are implemented in rcutils as well
## https://github.com/ros2/rcutils/blob/35f29850064e0c33a4063cbc947ebbfeada11dba/include/rcutils/logging.h#L164-L172
## This leaves space for other standard logging levels to be inserted in the middle in the future,
## as well as custom user defined levels.
## Since there are several other logging enumeration standard for different implementations,
## other logging implementations may need to provide level mappings to match their internal implementations.
##

# Debug is for pedantic information, which is useful when debugging issues.
byte DEBUG=10

# Info is the standard informational level and is used to report expected
# information.
byte INFO=20

# Warning is for information that may potentially cause issues or possibly unexpected
# behavior.
byte WARN=30

# Error is for information that this node cannot resolve.
byte ERROR=40

# Information about a impending node shutdown.
byte FATAL=50

##
## Fields
##

# Timestamp when this message was generated by the node.
builtin_interfaces/Time stamp

# Corresponding log level, see above definitions.
uint8 level

# The name representing the logger this message came from.
string name

# The full log message.
string msg

# The file the message came from.
string file

# The function the message came from.
string function

# The line in the file the message came from.
uint32 line`;
    const types = parse(messageDefinition, { ros2: true });
    expect(types).toEqual([
      {
        definitions: [
          {
            type: "int8",
            name: "DEBUG",
            isConstant: true,
            value: 10,
            valueText: "10",
          },
          {
            type: "int8",
            name: "INFO",
            isConstant: true,
            value: 20,
            valueText: "20",
          },
          {
            type: "int8",
            name: "WARN",
            isConstant: true,
            value: 30,
            valueText: "30",
          },
          {
            type: "int8",
            name: "ERROR",
            isConstant: true,
            value: 40,
            valueText: "40",
          },
          {
            type: "int8",
            name: "FATAL",
            isConstant: true,
            value: 50,
            valueText: "50",
          },
          {
            type: "time",
            isArray: false,
            name: "stamp",
            isComplex: false,
          },
          {
            type: "uint8",
            isArray: false,
            name: "level",
            isComplex: false,
          },
          {
            type: "string",
            isArray: false,
            name: "name",
            isComplex: false,
          },
          {
            type: "string",
            isArray: false,
            name: "msg",
            isComplex: false,
          },
          {
            type: "string",
            isArray: false,
            name: "file",
            isComplex: false,
          },
          {
            type: "string",
            isArray: false,
            name: "function",
            isComplex: false,
          },
          {
            type: "uint32",
            isArray: false,
            name: "line",
            isComplex: false,
          },
        ],
      },
    ]);
  });

  it("handles type names with 3 components", () => {
    expect(
      parse(
        `
std_msgs/msg/Header header
rosbridge_msgs/msg/ConnectedClient[] clients

================================================================================
MSG: rosbridge_msgs/msg/ConnectedClient
string ip_address
builtin_interfaces/Time connection_time

================================================================================
MSG: std_msgs/msg/Header
builtin_interfaces/msg/Time stamp
string frame_id
    `,
        { ros2: true },
      ),
    ).toEqual([
      {
        definitions: [
          {
            isArray: false,
            isComplex: true,
            name: "header",
            type: "std_msgs/msg/Header",
          },
          {
            isArray: true,
            isComplex: true,
            name: "clients",
            type: "rosbridge_msgs/msg/ConnectedClient",
          },
        ],
        name: undefined,
      },
      {
        definitions: [
          {
            isArray: false,
            isComplex: false,
            name: "ip_address",
            type: "string",
            upperBound: undefined,
          },
          {
            isArray: false,
            isComplex: false,
            name: "connection_time",
            type: "time",
          },
        ],
        name: "rosbridge_msgs/msg/ConnectedClient",
      },
      {
        definitions: [
          {
            isArray: false,
            isComplex: false,
            name: "stamp",
            type: "time",
          },
          {
            isArray: false,
            isComplex: false,
            name: "frame_id",
            type: "string",
            upperBound: undefined,
          },
        ],
        name: "std_msgs/msg/Header",
      },
    ]);
  });

  it("handles bounded arrays", () => {
    expect(
      parse(
        // From https://docs.ros.org/en/galactic/Concepts/About-ROS-Interfaces.html
        `
int32[] unbounded_integer_array
int32[5] five_integers_array
int32[<=5] up_to_five_integers_array

string string_of_unbounded_size
string<=10 up_to_ten_characters_string

string[<=5] up_to_five_unbounded_strings
string<=10[] unbounded_array_of_string_up_to_ten_characters_each
string<=10[<=5] up_to_five_strings_up_to_ten_characters_each
`,
        { ros2: true },
      ),
    ).toEqual([
      {
        definitions: [
          {
            isArray: true,
            isComplex: false,
            name: "unbounded_integer_array",
            type: "int32",
          },
          {
            arrayLength: 5,
            isArray: true,
            isComplex: false,
            name: "five_integers_array",
            type: "int32",
          },
          {
            arrayUpperBound: 5,
            isArray: true,
            isComplex: false,
            name: "up_to_five_integers_array",
            type: "int32",
          },
          {
            isArray: false,
            isComplex: false,
            name: "string_of_unbounded_size",
            type: "string",
            upperBound: undefined,
          },
          {
            isArray: false,
            isComplex: false,
            name: "up_to_ten_characters_string",
            type: "string",
            upperBound: 10,
          },
          {
            arrayUpperBound: 5,
            isArray: true,
            isComplex: false,
            name: "up_to_five_unbounded_strings",
            type: "string",
            upperBound: undefined,
          },
          {
            isArray: true,
            isComplex: false,
            name: "unbounded_array_of_string_up_to_ten_characters_each",
            type: "string",
            upperBound: 10,
          },
          {
            arrayUpperBound: 5,
            isArray: true,
            isComplex: false,
            name: "up_to_five_strings_up_to_ten_characters_each",
            type: "string",
            upperBound: 10,
          },
        ],
        name: undefined,
      },
    ]);
  });
});
