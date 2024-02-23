import { parse } from "./parse";
import { stringify } from "./stringify";

describe("stringify", () => {
  it("round trips a definition into canonical format", () => {
    const messageDefinition = `
      uint32 foo = 55
      int32 bar=-11 # Comment # another comment
      string test
      float32 baz= \t -32.25
      bool someBoolean = 0
      Point[] points
      int64 A = 0000000000000001
      string fooStr = Foo    ${""}
      string EXAMPLE="#comments" are ignored, and leading and trailing whitespace removed
      ============
      MSG: geometry_msgs/Point
      float64 x
    `;
    const types = parse(messageDefinition);

    const output = stringify(types);
    expect(output).toEqual(`uint32 foo = 55
int32 bar = -11
float32 baz = -32.25
bool someBoolean = 0
int64 A = 0000000000000001
string fooStr = Foo
string EXAMPLE = "#comments" are ignored, and leading and trailing whitespace removed

string test
geometry_msgs/Point[] points

================================================================================
MSG: geometry_msgs/Point

float64 x`);
  });

  it("supports ROS2 features", () => {
    const messageDefinition = `
      string<=5 str1 'abc'
      string str2 "def"
      int8[<=2] arr1 [ 1 ,-1 ]    ${""}
      string<=1[<=3] arr2   # comment
      bool a  true
      float32 b  -1.0
      float64 c  42.42
      int8 d -100
      uint8 e 100
      int16 f -1000
      uint16 g 1000
      int32 h -100000
      uint32 i 100000
      int64 j -5000000000
      uint64 k 5000000000
      string my_string1 "I heard \\"Hello\\""# is valid
      string my_string2 "I heard 'Hello'" # is valid
      string my_string3 'I heard \\'Hello\\''  # is valid
      string my_string4 'I heard "Hello"'   # is valid
    `;
    const types = parse(messageDefinition, { ros2: true });

    const output = stringify(types);
    expect(output).toEqual(`string<=5 str1 "abc"
string str2 "def"
int8[<=2] arr1 [1, -1]
string<=1[<=3] arr2
bool a true
float32 b -1
float64 c 42.42
int8 d -100
uint8 e 100
int16 f -1000
uint16 g 1000
int32 h -100000
uint32 i 100000
int64 j -5000000000
uint64 k 5000000000
string my_string1 "I heard \\"Hello\\""
string my_string2 "I heard 'Hello'"
string my_string3 "I heard 'Hello'"
string my_string4 "I heard \\"Hello\\""`);
  });
});
