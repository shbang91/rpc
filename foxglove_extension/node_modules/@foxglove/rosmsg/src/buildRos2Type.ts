import { MessageDefinition, MessageDefinitionField } from "@foxglove/message-definition";

/**
 * Parser for ROS 2 type definition lines.
 * Reference implementation: https://github.com/ros2/rosidl/blob/master/rosidl_adapter/rosidl_adapter/parser.py
 */

const TYPE = String.raw`(?<type>[a-zA-Z0-9_/]+)`;
const STRING_BOUND = String.raw`(?:<=(?<stringBound>\d+))`;
const ARRAY_BOUND = String.raw`(?:(?<unboundedArray>\[\])|\[(?<arrayLength>\d+)\]|\[<=(?<arrayBound>\d+)\])`;
const NAME = String.raw`(?<name>[a-zA-Z0-9_]+)`;
const QUOTED_STRING = String.raw`'(?:\\.|[^'\\])*'|"(?:\\.|[^"\\])*"`;
const COMMENT_TERMINATED_LITERAL = String.raw`(?:${QUOTED_STRING}|(?:\\.|[^\s'"#\\])(?:\\.|[^#\\])*)`;
const ARRAY_TERMINATED_LITERAL = String.raw`(?:${QUOTED_STRING}|(?:\\.|[^\s'"\],#\\])(?:\\.|[^\],#\\])*)`;
const CONSTANT_ASSIGNMENT = String.raw`\s*=\s*(?<constantValue>${COMMENT_TERMINATED_LITERAL}?)`;
const DEFAULT_VALUE_ARRAY = String.raw`\[(?:${ARRAY_TERMINATED_LITERAL},)*${ARRAY_TERMINATED_LITERAL}?\]`;
const DEFAULT_VALUE = String.raw`(?<defaultValue>${DEFAULT_VALUE_ARRAY}|${COMMENT_TERMINATED_LITERAL})`;
const COMMENT = String.raw`(?:#.*)`;
const DEFINITION_LINE_REGEX = new RegExp(
  String.raw`^${TYPE}${STRING_BOUND}?${ARRAY_BOUND}?\s+${NAME}(?:${CONSTANT_ASSIGNMENT}|\s+${DEFAULT_VALUE})?\s*${COMMENT}?$`,
);

const STRING_ESCAPES = String.raw`\\(?<char>['"abfnrtv\\])|\\(?<oct>[0-7]{1,3})|\\x(?<hex2>[a-fA-F0-9]{2})|\\u(?<hex4>[a-fA-F0-9]{4})|\\U(?<hex8>[a-fA-F0-9]{8})`;

const BUILTIN_TYPES = [
  "bool",
  "byte",
  "char",
  "float32",
  "float64",
  "int8",
  "uint8",
  "int16",
  "uint16",
  "int32",
  "uint32",
  "int64",
  "uint64",
  "string",
  "wstring",
  "time",
  "duration",
  "builtin_interfaces/Time",
  "builtin_interfaces/Duration",
  "builtin_interfaces/msg/Time",
  "builtin_interfaces/msg/Duration",
];

function parseBigIntLiteral(str: string, min: bigint, max: bigint) {
  const value = BigInt(str);
  if (value < min || value > max) {
    throw new Error(`Number ${str} out of range [${min}, ${max}]`);
  }
  return value;
}

function parseNumberLiteral(str: string, min: number, max: number): number {
  const value = parseInt(str);
  if (Number.isNaN(value)) {
    throw new Error(`Invalid numeric literal: ${str}`);
  }
  if (value < min || value > max) {
    throw new Error(`Number ${str} out of range [${min}, ${max}]`);
  }
  return value;
}

const LITERAL_REGEX = new RegExp(ARRAY_TERMINATED_LITERAL, "y");
const COMMA_OR_END_REGEX = /\s*(,)\s*|\s*$/y;
function parseArrayLiteral(
  type: string,
  rawStr: string,
): boolean[] | bigint[] | number[] | string[] {
  if (!rawStr.startsWith("[") || !rawStr.endsWith("]")) {
    throw new Error("Array must start with [ and end with ]");
  }
  const str = rawStr.substring(1, rawStr.length - 1);
  if (type === "string" || type === "wstring") {
    const results: string[] = [];
    let offset = 0;
    while (offset < str.length) {
      if (str[offset] === ",") {
        throw new Error("Expected array element before comma");
      }
      LITERAL_REGEX.lastIndex = offset;
      let match = LITERAL_REGEX.exec(str);
      if (match) {
        results.push(parseStringLiteral(match[0]!));
        offset = LITERAL_REGEX.lastIndex;
      }

      COMMA_OR_END_REGEX.lastIndex = offset;
      match = COMMA_OR_END_REGEX.exec(str);
      if (!match) {
        throw new Error("Expected comma or end of array");
      }
      if (!match[1]) {
        break;
      }
      offset = COMMA_OR_END_REGEX.lastIndex;
    }
    return results;
  }
  return str.split(",").map((part) => parsePrimitiveLiteral(type, part.trim())) as
    | boolean[]
    | bigint[]
    | number[]
    | string[];
}

function parseStringLiteral(maybeQuotedStr: string): string {
  let quoteThatMustBeEscaped = "";
  let str = maybeQuotedStr;
  for (const quote of ["'", '"']) {
    if (maybeQuotedStr.startsWith(quote)) {
      if (!maybeQuotedStr.endsWith(quote)) {
        throw new Error(`Expected terminating ${quote} in string literal: ${maybeQuotedStr}`);
      }
      quoteThatMustBeEscaped = quote;
      str = maybeQuotedStr.substring(quote.length, maybeQuotedStr.length - quote.length);
      break;
    }
  }
  if (
    !new RegExp(String.raw`^(?:[^\\${quoteThatMustBeEscaped}]|${STRING_ESCAPES})*$`).test(str) ==
    undefined
  ) {
    throw new Error(`Invalid string literal: ${str}`);
  }
  return str.replace(new RegExp(STRING_ESCAPES, "g"), (...args) => {
    const { char, oct, hex2, hex4, hex8 } = args[args.length - 1] as NonNullable<
      RegExpMatchArray["groups"]
    >;
    const hex = hex2 ?? hex4 ?? hex8;
    if (char != undefined) {
      return {
        "'": "'",
        '"': '"',
        a: "\x07",
        b: "\b",
        f: "\f",
        n: "\n",
        r: "\r",
        t: "\t",
        v: "\v",
        "\\": "\\",
      }[char]!;
    } else if (oct != undefined) {
      return String.fromCodePoint(parseInt(oct, 8));
    } else if (hex != undefined) {
      return String.fromCodePoint(parseInt(hex, 16));
    } else {
      throw new Error("Expected exactly one matched group");
    }
  });
}

function parsePrimitiveLiteral(type: string, str: string): boolean | number | bigint | string {
  switch (type) {
    case "bool":
      if (["true", "True", "1"].includes(str)) {
        return true;
      } else if (["false", "False", "0"].includes(str)) {
        return false;
      }
      break;
    case "float32":
    case "float64": {
      const value = parseFloat(str);
      if (!Number.isNaN(value)) {
        return value;
      }
      break;
    }
    case "int8":
      return parseNumberLiteral(str, ~0x7f, 0x7f);
    case "uint8":
      return parseNumberLiteral(str, 0, 0xff);
    case "int16":
      return parseNumberLiteral(str, ~0x7fff, 0x7fff);
    case "uint16":
      return parseNumberLiteral(str, 0, 0xffff);
    case "int32":
      return parseNumberLiteral(str, ~0x7fffffff, 0x7fffffff);
    case "uint32":
      return parseNumberLiteral(str, 0, 0xffffffff);
    case "int64":
      return parseBigIntLiteral(str, ~0x7fffffffffffffffn, 0x7fffffffffffffffn);
    case "uint64":
      return parseBigIntLiteral(str, 0n, 0xffffffffffffffffn);
    case "string":
    case "wstring":
      return parseStringLiteral(str);
  }
  throw new Error(`Invalid literal of type ${type}: ${str}`);
}

function normalizeType(type: string): string {
  switch (type) {
    case "char":
      return "uint8";
    case "byte":
      return "int8";
    case "builtin_interfaces/Time":
    case "builtin_interfaces/msg/Time":
      return "time";
    case "builtin_interfaces/Duration":
    case "builtin_interfaces/msg/Duration":
      return "duration";
  }
  return type;
}
export function buildRos2Type(lines: { line: string }[]): MessageDefinition {
  const definitions: MessageDefinitionField[] = [];
  let complexTypeName: string | undefined;
  for (const { line } of lines) {
    let match;
    if (line.startsWith("#")) {
      continue;
    } else if ((match = /^MSG: ([^ ]+)\s*(?:#.+)?$/.exec(line))) {
      complexTypeName = match[1];
      continue;
    } else if ((match = DEFINITION_LINE_REGEX.exec(line))) {
      const {
        type: rawType,
        stringBound,
        unboundedArray,
        arrayLength,
        arrayBound,
        name,
        constantValue,
        defaultValue,
      } = match.groups!;
      const type = normalizeType(rawType!);

      if (stringBound != undefined && type !== "string" && type !== "wstring") {
        throw new Error(`Invalid string bound for type ${type}`);
      }
      if (constantValue != undefined) {
        if (!/^[A-Z](?:_?[A-Z0-9]+)*$/.test(name!)) {
          throw new Error(`Invalid constant name: ${name!}`);
        }
      } else {
        if (!/^[a-z](?:_?[a-z0-9]+)*$/.test(name!)) {
          throw new Error(`Invalid field name: ${name!}`);
        }
      }
      const isComplex = !BUILTIN_TYPES.includes(type);
      const isArray =
        unboundedArray != undefined || arrayLength != undefined || arrayBound != undefined;
      definitions.push({
        name: name!,
        type,
        isComplex: constantValue != undefined ? isComplex || undefined : isComplex,
        isConstant: constantValue != undefined || undefined,
        isArray: constantValue != undefined ? isArray || undefined : isArray,
        arrayLength: arrayLength != undefined ? parseInt(arrayLength) : undefined,
        arrayUpperBound: arrayBound != undefined ? parseInt(arrayBound) : undefined,
        upperBound: stringBound != undefined ? parseInt(stringBound) : undefined,
        defaultValue:
          defaultValue != undefined
            ? isArray
              ? parseArrayLiteral(type, defaultValue.trim())
              : parsePrimitiveLiteral(type, defaultValue.trim())
            : undefined,
        value:
          constantValue != undefined
            ? parsePrimitiveLiteral(type, constantValue.trim())
            : undefined,
        valueText: constantValue?.trim(),
      });
    } else {
      throw new Error(`Could not parse line: '${line}'`);
    }
  }
  return { name: complexTypeName, definitions };
}
