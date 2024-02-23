import { parseRos2idl as parse } from "./parseRos2idl";

describe("rosidl grammar tests", () => {
  it("parses a module with an enclosed struct and module", () => {
    const types = parse(
      `
    module rosidl_parser {
      module action {
        module MyAction_Goal_Constants {
          const short SHORT_CONSTANT = -23;
        };
        struct MyAction_Goal {
          int32 input_value;
        };
      };
    };
    `,
    );
    expect(types).toEqual([
      {
        definitions: [
          {
            isConstant: true,
            value: -23,
            valueText: "-23",
            isComplex: false,
            name: "SHORT_CONSTANT",
            type: "int16",
          },
        ],
        name: "rosidl_parser/action/MyAction_Goal_Constants",
      },
      {
        definitions: [
          {
            isComplex: false,
            name: "input_value",
            type: "int32",
          },
        ],
        name: "rosidl_parser/action/MyAction_Goal",
      },
    ]);
  });
  it("parses a module with a typedefs used in a struct", () => {
    const types = parse(
      `
    module rosidl_parser {
      module action {
        typedef sequence<int32, 10> int32arr;
        @default (value=5)
        typedef short shortWithDefault;
        struct MyAction_Goal {
          int32arr intArray;
          shortWithDefault short5;
        };
      };
    };
    `,
    );
    expect(types).toEqual([
      {
        definitions: [
          {
            isComplex: false,
            name: "intArray",
            type: "int32",
            isArray: true,
            arrayUpperBound: 10,
          },
          {
            isComplex: false,
            name: "short5",
            type: "int16",
            defaultValue: 5,
          },
        ],
        name: "rosidl_parser/action/MyAction_Goal",
      },
    ]);
  });
  it("parses a module with an multiple enclosed structs and modules", () => {
    const types = parse(
      `
      module rosidl_parser {
        module action {
          module MyAction_Goal_Constants {
            const short SHORT_CONSTANT = -23;
          };
          struct MyAction_Goal {
            int32 input_value;
          };
          module MyAction_Result_Constants {
            const unsigned long UNSIGNED_LONG_CONSTANT = 42;
          };
          struct MyAction_Result {
            uint32 output_value;
          };
          module MyAction_Feedback_Constants {
            const float FLOAT_CONSTANT = 1.25;
          };
          struct MyAction_Feedback {
            float progress_value;
          };
        };
      };
    `,
    );
    expect(types).toEqual([
      {
        name: "rosidl_parser/action/MyAction_Goal_Constants",
        definitions: [
          {
            isConstant: true,
            type: "int16",
            name: "SHORT_CONSTANT",
            valueText: "-23",
            value: -23,
            isComplex: false,
          },
        ],
      },
      {
        name: "rosidl_parser/action/MyAction_Goal",
        definitions: [
          {
            type: "int32",
            name: "input_value",
            isComplex: false,
          },
        ],
      },
      {
        name: "rosidl_parser/action/MyAction_Result_Constants",
        definitions: [
          {
            isConstant: true,
            type: "uint32",
            name: "UNSIGNED_LONG_CONSTANT",
            valueText: "42",
            value: 42,
            isComplex: false,
          },
        ],
      },
      {
        name: "rosidl_parser/action/MyAction_Result",
        definitions: [
          {
            type: "uint32",
            name: "output_value",
            isComplex: false,
          },
        ],
      },
      {
        name: "rosidl_parser/action/MyAction_Feedback_Constants",
        definitions: [
          {
            isConstant: true,
            type: "float32",
            name: "FLOAT_CONSTANT",
            valueText: "1.25",
            value: 1.25,
            isComplex: false,
          },
        ],
      },
      {
        name: "rosidl_parser/action/MyAction_Feedback",
        definitions: [
          {
            type: "float32",
            name: "progress_value",
            isComplex: false,
          },
        ],
      },
    ]);
  });
  it("ignore #include statements in AST", () => {
    const types = parse(
      `
   #include "OtherMessage.idl"
   #include <pkgname/msg/OtherMessage.idl>

    module rosidl_parser {
      module action {
        module MyAction_Goal_Constants {
          const short SHORT_CONSTANT = -23;
        };
        struct MyAction_Goal {
          int32 input_value;
        };
      };
    };
    `,
    );

    // same as above
    expect(types).toEqual([
      {
        definitions: [
          {
            isConstant: true,
            value: -23,
            valueText: "-23",
            isComplex: false,
            name: "SHORT_CONSTANT",
            type: "int16",
          },
        ],
        name: "rosidl_parser/action/MyAction_Goal_Constants",
      },
      {
        definitions: [
          {
            isComplex: false,
            name: "input_value",
            type: "int32",
          },
        ],
        name: "rosidl_parser/action/MyAction_Goal",
      },
    ]);
  });
  it("parses a module full of string constants", () => {
    const types = parse(
      `
module rosidl_parser {
  module msg {
    module MyMessage_Constants {
      const string STRING_CONSTANT = "string_value";
      const wstring WSTRING_CONSTANT = "wstring_value_\u2122";
      const string EMPTY_STRING_CONSTANT = "";
      const string COMBINED_STRING_CONSTANT = "part1 " "part2" " part3";
    };
  };
};
    `,
    );
    expect(types).toEqual([
      {
        definitions: [
          {
            isConstant: true,
            value: "string_value",
            valueText: "string_value",
            isComplex: false,
            name: "STRING_CONSTANT",
            type: "string",
          },
          {
            isConstant: true,
            value: "wstring_value_\u2122",
            valueText: "wstring_value_\u2122",
            isComplex: false,
            name: "WSTRING_CONSTANT",
            type: "string",
          },
          {
            isConstant: true,
            value: "",
            valueText: "",
            isComplex: false,
            name: "EMPTY_STRING_CONSTANT",
            type: "string",
          },
          {
            isConstant: true,
            value: "part1 part2 part3",
            valueText: "part1 part2 part3",
            isComplex: false,
            name: "COMBINED_STRING_CONSTANT",
            type: "string",
          },
        ],
        name: "rosidl_parser/msg/MyMessage_Constants",
      },
    ]);
  });
  it("parses all non-array type declarations", () => {
    const types = parse(
      `
      module rosidl_parser {
        module msg {
          struct MyMessage {
            unsigned short unsigned_short_value;
            long long_value;
            unsigned long unsigned_long_value;
            long long long_long_value;
            unsigned long long unsigned_long_long_value;
            float float_value;
            double double_value;
            char char_value;
            wchar wchar_value;
            boolean boolean_value;
            octet octet_value;
            int8 int8_value;
            uint8 uint8_value;
            int16 int16_value;
            uint16 uint16_value;
            int32 int32_value;
            uint32 uint32_value;
            int64 int64_value;
            uint64 uint64_value;
            string string_value;
          };
        };
      };
    `,
    );
    expect(types).toEqual([
      {
        name: "rosidl_parser/msg/MyMessage",
        definitions: [
          {
            type: "uint16",
            name: "unsigned_short_value",
            isComplex: false,
          },
          {
            type: "int32",
            name: "long_value",
            isComplex: false,
          },
          {
            type: "uint32",
            name: "unsigned_long_value",
            isComplex: false,
          },
          {
            type: "int64",
            name: "long_long_value",
            isComplex: false,
          },
          {
            type: "uint64",
            name: "unsigned_long_long_value",
            isComplex: false,
          },
          {
            type: "float32",
            name: "float_value",
            isComplex: false,
          },
          {
            type: "float64",
            name: "double_value",
            isComplex: false,
          },
          {
            type: "uint8",
            name: "char_value",
            isComplex: false,
          },
          {
            type: "uint8",
            name: "wchar_value",
            isComplex: false,
          },
          {
            type: "bool",
            name: "boolean_value",
            isComplex: false,
          },
          {
            type: "int8",
            name: "octet_value",
            isComplex: false,
          },
          {
            type: "int8",
            name: "int8_value",
            isComplex: false,
          },
          {
            type: "uint8",
            name: "uint8_value",
            isComplex: false,
          },
          {
            type: "int16",
            name: "int16_value",
            isComplex: false,
          },
          {
            type: "uint16",
            name: "uint16_value",
            isComplex: false,
          },
          {
            type: "int32",
            name: "int32_value",
            isComplex: false,
          },
          {
            type: "uint32",
            name: "uint32_value",
            isComplex: false,
          },
          {
            type: "int64",
            name: "int64_value",
            isComplex: false,
          },
          {
            type: "uint64",
            name: "uint64_value",
            isComplex: false,
          },
          {
            type: "string",
            name: "string_value",
            isComplex: false,
          },
        ],
      },
    ]);
  });
  it("parses a module of all array types", () => {
    const types = parse(
      `
      module rosidl_parser {
        module msg {
          module MyMessage_Constants {
            const unsigned long UNSIGNED_LONG_CONSTANT = 42;
          };
          struct MyMessage {   
            string<5> bounded_string_value;
            wstring wstring_value;
            wstring<23> bounded_wstring_value;
            wstring<UNSIGNED_LONG_CONSTANT> constant_bounded_wstring_value;
            sequence<short> unbounded_short_values;
            sequence<short, 5> bounded_short_values;
            sequence<string<3>> unbounded_values_of_bounded_strings;
            sequence<string<3>, 4> bounded_values_of_bounded_strings;
            short array_short_values[23];
          };
        };
      };
    `,
    );
    expect(types).toEqual([
      {
        name: "rosidl_parser/msg/MyMessage_Constants",
        definitions: [
          {
            name: "UNSIGNED_LONG_CONSTANT",
            type: "uint32",
            isConstant: true,
            isComplex: false,
            value: 42,
            valueText: "42",
          },
        ],
      },
      {
        name: "rosidl_parser/msg/MyMessage",
        definitions: [
          {
            type: "string",
            upperBound: 5,
            name: "bounded_string_value",
            isComplex: false,
          },
          {
            type: "string",
            name: "wstring_value",
            isComplex: false,
          },
          {
            type: "string",
            upperBound: 23,
            name: "bounded_wstring_value",
            isComplex: false,
          },
          {
            type: "string",
            upperBound: 42,
            name: "constant_bounded_wstring_value",
            isComplex: false,
          },
          {
            type: "int16",
            isArray: true,
            name: "unbounded_short_values",
            isComplex: false,
          },
          {
            type: "int16",
            isArray: true,
            arrayUpperBound: 5,
            name: "bounded_short_values",
            isComplex: false,
          },
          {
            type: "string",
            upperBound: 3,
            isArray: true,
            name: "unbounded_values_of_bounded_strings",
            isComplex: false,
          },
          {
            type: "string",
            upperBound: 3,
            isArray: true,
            arrayUpperBound: 4,
            name: "bounded_values_of_bounded_strings",
            isComplex: false,
          },
          {
            type: "int16",
            name: "array_short_values",
            isArray: true,
            arrayLength: 23,
            isComplex: false,
          },
        ],
      },
    ]);
  });

  it("parses a module with arbitrary annotations including default values", () => {
    const types = parse(
      `
module rosidl_parser {
  module msg {
    @verbatim ( language="comment", text="Documentation of MyMessage." "Adjacent string literal." )
    @transfer_mode(SHMEM_REF)
    struct MyMessage {
      @default ( value=123 )
      unsigned short unsigned_short_value;
      @key
      @range ( min=-10, max=10 )
      long long_value;
      @verbatim (language="comment", text="")
      @arbitrary_annotation ( key1="value1", key2=TRUE, key3=0.0, key4=10 )
      @key unsigned long unsigned_long_value;
    };
  };
};
    `,
    );
    expect(types).toEqual([
      {
        name: "rosidl_parser/msg/MyMessage",
        definitions: [
          {
            defaultValue: 123,
            type: "uint16",
            name: "unsigned_short_value",
            isComplex: false,
          },
          {
            type: "int32",
            name: "long_value",
            isComplex: false,
          },
          {
            type: "uint32",
            name: "unsigned_long_value",
            isComplex: false,
          },
        ],
      },
    ]);
  });
  it('parses a module with customTypes and properly replaces "::"', () => {
    const types = parse(
      `
module rosidl_parser {
  module msg {
    struct MyMessage {
      geometry::msg::Point single_point;
      geometry::msg::Point points_with_length[10];
      sequence<geometry::msg::Point> points_with_length_sequence;
    };
  };
};
    `,
    );
    expect(types).toEqual([
      {
        name: "rosidl_parser/msg/MyMessage",
        definitions: [
          {
            type: "geometry/msg/Point",
            name: "single_point",
            isComplex: true,
          },
          {
            type: "geometry/msg/Point",
            name: "points_with_length",
            isArray: true,
            arrayLength: 10,
            isComplex: true,
          },
          {
            type: "geometry/msg/Point",
            name: "points_with_length_sequence",
            isArray: true,
            isComplex: true,
          },
        ],
      },
    ]);
  });
  it("parses a module with various floating point default values", () => {
    const types = parse(
      `
      module rosidl_parser {
        module msg {
          struct MyMessage {
            @default ( value=1.9e10 )
            float int_and_frac_with_positive_scientific;
            @default ( value=1.9e+10 )
            float int_and_frac_with_explicit_positive_scientific;
            @default ( value=1.1e-10)
            float int_and_frac_with_negative_scientific;
            @default ( value=0.00009 )
            float int_and_frac;
            @default ( value = 1. )
            float int_with_empty_frac;
            @default ( value = .1 )
            float frac_only;
            @default ( value=9e05 )
            float int_with_positive_scientific;
            @default ( value=9e+05 )
            float int_with_explicit_positive_scientific;
            @default ( value=9e-05 )
            float int_with_negative_scientific;
            @default ( value=8.7d )
            float fixed_int_and_frac;
            @default ( value=4.d )
            float fixed_int_with_dot_only;
            @default ( value=.3d )
            float fixed_frac_only;
            @default ( value=7d )
            float fixed_int_only;
          };
        };
      };
    `,
    );
    expect(types).toEqual([
      {
        name: "rosidl_parser/msg/MyMessage",
        definitions: [
          {
            defaultValue: 19000000000,
            type: "float32",
            name: "int_and_frac_with_positive_scientific",
            isComplex: false,
          },
          {
            defaultValue: 19000000000,
            type: "float32",
            name: "int_and_frac_with_explicit_positive_scientific",
            isComplex: false,
          },
          {
            defaultValue: 1.1e-10,
            type: "float32",
            name: "int_and_frac_with_negative_scientific",
            isComplex: false,
          },
          {
            defaultValue: 0.00009,
            type: "float32",
            name: "int_and_frac",
            isComplex: false,
          },
          {
            defaultValue: 1,
            type: "float32",
            name: "int_with_empty_frac",
            isComplex: false,
          },
          {
            defaultValue: 0.1,
            type: "float32",
            name: "frac_only",
            isComplex: false,
          },
          {
            defaultValue: 900000,
            type: "float32",
            name: "int_with_positive_scientific",
            isComplex: false,
          },
          {
            defaultValue: 900000,
            type: "float32",
            name: "int_with_explicit_positive_scientific",
            isComplex: false,
          },
          {
            defaultValue: 0.00009,
            type: "float32",
            name: "int_with_negative_scientific",
            isComplex: false,
          },
          {
            defaultValue: 8.7,
            type: "float32",
            name: "fixed_int_and_frac",
            isComplex: false,
          },
          {
            defaultValue: 4,
            type: "float32",
            name: "fixed_int_with_dot_only",
            isComplex: false,
          },
          {
            defaultValue: 0.3,
            type: "float32",
            name: "fixed_frac_only",
            isComplex: false,
          },
          {
            defaultValue: 7,
            type: "float32",
            name: "fixed_int_only",
            isComplex: false,
          },
        ],
      },
    ]);
  });
  it("parses a module full of numeric constants", () => {
    const types = parse(
      `
module rosidl_parser {
  module msg {
    module MyMessage_Constants {
      const short SHORT_CONSTANT = -23;
      const unsigned long UNSIGNED_LONG_CONSTANT = 42;
      const float FLOAT_CONSTANT = 1.25;
      const double EXP_DOUBLE_CONSTANT = 1.25e-3;
    };
  };
};
    `,
    );
    expect(types).toEqual([
      {
        definitions: [
          {
            isConstant: true,
            value: -23,
            valueText: "-23",
            isComplex: false,
            name: "SHORT_CONSTANT",
            type: "int16",
          },
          {
            isConstant: true,
            value: 42,
            valueText: "42",
            isComplex: false,
            name: "UNSIGNED_LONG_CONSTANT",
            type: "uint32",
          },
          {
            isConstant: true,
            value: 1.25,
            valueText: "1.25",
            isComplex: false,
            name: "FLOAT_CONSTANT",
            type: "float32",
          },
          {
            isConstant: true,
            value: 1.25e-3,
            valueText: "1.25e-3",
            isComplex: false,
            name: "EXP_DOUBLE_CONSTANT",
            type: "float64",
          },
        ],
        name: "rosidl_parser/msg/MyMessage_Constants",
      },
    ]);
  });

  it("can parse comments", () => {
    const msgDef = ` 
      // All of these comments should be ignored
      module action {
        /** another comment */
        module MyAction_Goal_Constants /** maybe a sneaky one here */ {
          // two here of the same type
          // _another one_
          const string tricky = "/** is this a comment? */ // hopefully not"; // like I'm not even here
        };
        /** a multi
         * line
         * comment
         */
        struct MyAction_Goal {
          // two here of different types
          /** maybe one more that */
          int32 input_value; /** inline */
        };
      };
    `;
    const types = parse(msgDef);
    expect(types).toEqual([
      {
        definitions: [
          {
            isConstant: true,
            value: "/** is this a comment? */ // hopefully not",
            valueText: "/** is this a comment? */ // hopefully not",
            upperBound: undefined,
            isComplex: false,
            name: "tricky",
            type: "string",
          },
        ],
        name: "action/MyAction_Goal_Constants",
      },
      {
        definitions: [
          {
            isComplex: false,
            name: "input_value",
            type: "int32",
          },
        ],
        name: "action/MyAction_Goal",
      },
    ]);
  });
  it("can parse multiple forward declarations on same line with default annotation", () => {
    const msgDef = ` 
    module action {
      struct MyAction_Goal {
        @default(value=5)
        int32 int1, int2;
      };
    };
    `;
    const types = parse(msgDef);
    expect(types).toEqual([
      {
        definitions: [
          {
            defaultValue: 5,
            isComplex: false,
            name: "int1",
            type: "int32",
          },
          {
            defaultValue: 5,
            isComplex: false,
            name: "int2",
            type: "int32",
          },
        ],
        name: "action/MyAction_Goal",
      },
    ]);
  });
  /****************  Not supported by IDL (as far as I can tell) */
  it("cannot parse multiple const declarations in a single line", () => {
    const msgDef = ` 
      module action {
        module MyAction_Goal_Constants {
          const short short1, short2 = -23;
        };
      };
    `;
    expect(() => parse(msgDef)).toThrow(/unexpected , token: ","/i);
  });
  /****************  Syntax Errors */
  it("missing bracket at the end will result in end of input error", () => {
    const msgDef = ` 
    module rosidl_parser {
      module action {
        module MyAction_Goal_Constants {
          const short SHORT_CONSTANT = -23;
        };
        struct MyAction_Goal {
          int32 input_value;
        };
      };
    `;
    expect(() => parse(msgDef)).toThrow(
      `Could not parse message definition (unexpected end of input): '${msgDef}'`,
    );
  });
  it("cannot parse empty module", () => {
    const msgDef = ` 
    module rosidl_parser {
      module action {
        module MyAction_Goal_Constants {
        };
        struct MyAction_Goal {
          int32 input_value;
        };
      };
    };`;
    expect(() => parse(msgDef)).toThrow(/unexpected RCBR token: "}"/i);
  });
});
