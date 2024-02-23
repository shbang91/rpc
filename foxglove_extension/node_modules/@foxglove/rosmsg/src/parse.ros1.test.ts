// This file incorporates work covered by the following copyright and
// permission notice:
//
//   Copyright 2018-2021 Cruise LLC
//
//   This source code is licensed under the Apache License, Version 2.0,
//   found at http://www.apache.org/licenses/LICENSE-2.0
//   You may not use this file except in compliance with the License.

import { MessageDefinition } from "@foxglove/message-definition";

import { fixupTypes, parse } from "./parse";

describe("parseMessageDefinition", () => {
  it("parses a single field from a single message", () => {
    const types = parse("string name");
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

  it("rejects valid tokens that don't fully match a parser rule", () => {
    expect(() => parse("abc")).toThrow("Could not parse line: 'abc'");
  });

  it.each(["_a", "3a"])("rejects invalid field name %s", (name) => {
    expect(() => parse(`string ${name}`)).toThrow();
  });
  it.each(["3a"])("rejects invalid constant name %s", (name) => {
    expect(() => parse(`string ${name} = 'x'`)).toThrow();
  });
  it.each(["a", "a_", "foo_bar", "foo__bar", "foo1_2bar"])(
    "accepts valid field name %s",
    (name) => {
      expect(parse(`string ${name}`)).toEqual([
        {
          definitions: [
            { arrayLength: undefined, isArray: false, isComplex: false, name, type: "string" },
          ],
          name: undefined,
        },
      ]);
    },
  );
  it.each(["a", "_a", "a_", "foo_bar", "foo__Bar", "FOO1_2BAR"])(
    "accepts valid constant name %s",
    (name) => {
      expect(parse(`string ${name} = x`)).toEqual([
        {
          definitions: [{ name, type: "string", isConstant: true, value: "x", valueText: "x" }],
          name: undefined,
        },
      ]);
    },
  );

  it("resolves unqualified names", () => {
    const messageDefinition = `
      Point[] points
      ============
      MSG: geometry_msgs/Point
      float64 x
    `;
    const types = parse(messageDefinition);
    expect(types).toEqual([
      {
        definitions: [
          {
            arrayLength: undefined,
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
            arrayLength: undefined,
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
    const types = parse("char x\nbyte y");
    expect(types).toEqual([
      {
        definitions: [
          {
            arrayLength: undefined,
            isArray: false,
            isComplex: false,
            name: "x",
            type: "uint8",
          },
          {
            arrayLength: undefined,
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
    string firstName

    # last name here
    ### foo bar baz?
    string lastName
    `;
    const types = parse(messageDefinition);
    expect(types).toEqual([
      {
        definitions: [
          {
            arrayLength: undefined,
            isArray: false,
            isComplex: false,
            name: "firstName",
            type: "string",
          },
          {
            arrayLength: undefined,
            isArray: false,
            isComplex: false,
            name: "lastName",
            type: "string",
          },
        ],
        name: undefined,
      },
    ]);
  });

  it.each(["string", "int32", "int64"])("parses variable length %s array", (type) => {
    const types = parse(`${type}[] names`);
    expect(types).toEqual([
      {
        definitions: [
          {
            arrayLength: undefined,
            isArray: true,
            isComplex: false,
            name: "names",
            type,
          },
        ],
        name: undefined,
      },
    ]);
  });

  it("parses fixed length string array", () => {
    const types = parse("string[3] names");
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
    const types = parse(messageDefinition);
    expect(types).toEqual([
      {
        definitions: [
          {
            arrayLength: undefined,
            isArray: false,
            isComplex: false,
            name: "username",
            type: "string",
          },
          {
            arrayLength: undefined,
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
            arrayLength: undefined,
            isArray: false,
            isComplex: false,
            name: "name",
            type: "string",
          },
          {
            arrayLength: undefined,
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
      uint32 foo = 55
      int32 bar=-11 # Comment # another comment
      float32 baz= \t -32.25
      bool someBoolean = 0
      string fooStr = Foo    ${""}
      string EMPTY1 =  ${""}
      string EMPTY2 =
      string HASH = #
      string EXAMPLE="#comments" are ignored, and leading and trailing whitespace removed
      uint64 SMOOTH_MOVE_START    = 0000000000000001 # e.g. kobuki_msgs/VersionInfo
      int64 LARGE_VALUE = -9223372036854775807
    `;
    const types = parse(messageDefinition);
    expect(types).toEqual([
      {
        definitions: [
          {
            name: "foo",
            type: "uint32",
            isConstant: true,
            value: 55,
            valueText: "55",
          },
          {
            name: "bar",
            type: "int32",
            isConstant: true,
            value: -11,
            valueText: "-11",
          },
          {
            name: "baz",
            type: "float32",
            isConstant: true,
            value: -32.25,
            valueText: "-32.25",
          },
          {
            name: "someBoolean",
            type: "bool",
            isConstant: true,
            value: false,
            valueText: "0",
          },
          {
            name: "fooStr",
            type: "string",
            isConstant: true,
            value: "Foo",
            valueText: "Foo",
          },
          {
            name: "EMPTY1",
            type: "string",
            isConstant: true,
            value: "",
            valueText: "",
          },
          {
            name: "EMPTY2",
            type: "string",
            isConstant: true,
            value: "",
            valueText: "",
          },
          {
            name: "HASH",
            type: "string",
            isConstant: true,
            value: "#",
            valueText: "#",
          },
          {
            name: "EXAMPLE",
            type: "string",
            isConstant: true,
            value: '"#comments" are ignored, and leading and trailing whitespace removed',
            valueText: '"#comments" are ignored, and leading and trailing whitespace removed',
          },
          {
            name: "SMOOTH_MOVE_START",
            type: "uint64",
            isConstant: true,
            value: 1n,
            valueText: "0000000000000001",
          },
          {
            name: "LARGE_VALUE",
            type: "int64",
            isConstant: true,
            value: -9223372036854775807n,
            valueText: "-9223372036854775807",
          },
        ],
        name: undefined,
      },
    ]);
  });

  it("works with python boolean values", () => {
    const messageDefinition = `
      bool Alive=True
      bool Dead=False
    `;
    const types = parse(messageDefinition);
    expect(types).toEqual([
      {
        definitions: [
          {
            name: "Alive",
            type: "bool",
            isConstant: true,
            value: true,
            valueText: "True",
          },
          {
            name: "Dead",
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
      parse(`
    abc1/Foo2 value0
    ==========
    MSG: abc1/Foo2
    int32 data
    `),
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
});

describe("fixupTypes", () => {
  it("works with an empty list", () => {
    const types: MessageDefinition[] = [];
    fixupTypes(types);
    expect(types).toEqual([]);
  });

  it("rewrites type names as expected", () => {
    const messageDefinition = `
      Point[] points
      ============
      MSG: geometry_msgs/Point
      float64 x
    `;
    const types = parse(messageDefinition, { skipTypeFixup: true });

    expect(types).toHaveLength(2);
    expect(types[0]!.definitions).toHaveLength(1);
    expect(types[0]!.definitions[0]!.type).toEqual("Point");
    expect(types[1]!.definitions).toHaveLength(1);
    expect(types[1]!.definitions[0]!.type).toEqual("float64");

    fixupTypes(types);

    expect(types).toHaveLength(2);
    expect(types[0]!.definitions).toHaveLength(1);
    expect(types[0]!.definitions[0]!.type).toEqual("geometry_msgs/Point");
    expect(types[1]!.definitions).toHaveLength(1);
    expect(types[1]!.definitions[0]!.type).toEqual("float64");
  });
});
