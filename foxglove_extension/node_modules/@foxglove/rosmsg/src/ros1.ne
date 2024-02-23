@{%
const moo = require("moo");
const lexer = moo.compile({
  space: {match: /\s+/, lineBreaks: true},
  number: /-?(?:[0-9]|[1-9][0-9]+)(?:\.[0-9]+)?(?:[eE][-+]?[0-9]+)?\b/,
  comment: /#[^\n]*/,
  '[': '[',
  ']': ']',
  assignment: /=[^\n]*/,
  // Leading underscores are disallowed in field names, while constant names have no explicit restrictions.
  // So we are more lenient in lexing here, and the validation steps below are more strict.
  // See: https://github.com/ros/genmsg/blob/7d8b6ce6f43b6e39ea8261125d270f2d3062356f/src/genmsg/msg_loader.py#L188-L238
  fieldOrType: /[a-zA-Z_][a-zA-Z0-9_]*(?:\/[a-zA-Z][a-zA-Z0-9_]*)?/,
});
%}

@lexer lexer

main ->
    _ boolType arrayType __ field _ comment:? simple {% function(d) { return extend(d) } %}
  | _ bigintType arrayType __ field _ comment:? simple {% function(d) { return extend(d) } %}
  | _ numericType arrayType __ field _ comment:? simple {% function(d) { return extend(d) } %}
  | _ stringType arrayType __ field _ comment:? simple {% function(d) { return extend(d) } %}
  | _ timeType arrayType __ field _ comment:? simple {% function(d) { return extend(d) } %}
  | _ customType arrayType __ field _ comment:? complex {% function(d) { return extend(d) } %}
  | _ boolType __ constantField _ boolConstantValue _ comment:? {% function(d) { return extend(d) } %}
  | _ bigintType __ constantField _ bigintConstantValue _ comment:? {% function(d) { return extend(d) } %}
  | _ numericType __ constantField _ numericConstantValue _ comment:? {% function(d) { return extend(d) } %}
  | _ stringType __ constantField _ stringConstantValue _ comment:? {% function(d) { return extend(d) } %}
  | comment {% function(d) { return null } %}
  | blankLine {% function(d) { return null } %}

# Types

boolType -> "bool" {% function(d) { return { type: d[0].value } } %}

bigintType -> ("int64" | "uint64") {% function(d) { return { type: d[0][0].value } } %}

numericType -> (
    "byte"
  | "char"
  | "float32"
  | "float64"
  | "int8"
  | "uint8"
  | "int16"
  | "uint16"
  | "int32"
  | "uint32"
) {% function(d) { return { type: d[0][0].value } } %}

stringType -> "string" {% function(d) { return { type: d[0].value } } %}

timeType -> ("time" | "duration") {% function(d) { return { type: d[0][0].value } } %}

customType -> %fieldOrType {% function(d, _, reject) {
  const PRIMITIVE_TYPES = ["bool", "byte", "char", "float32", "float64", "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "string", "time", "duration"];
  const type = d[0].value;
  if (PRIMITIVE_TYPES.includes(type)) return reject;
  return { type };
} %}

arrayType ->
    "[" _ "]" {% function(d) { return { isArray: true } } %}
  | "[" _ number _ "]" {% function(d) { return { isArray: true, arrayLength: d[2] } } %}
  | _ {% function(d) { return { isArray: false } } %}

# Fields

field -> %fieldOrType {% function(d, _, reject) {
  const name = d[0].value;
  // Leading underscores or digits are not allowed in field names
  if (name.match(/^[a-zA-Z][a-zA-Z0-9_]*$/) == undefined) return reject;
  return { name };
} %}

constantField -> %fieldOrType {% function(d, _, reject) {
  const name = d[0].value;
  // Leading digits are not allowed in constant names (the ROS genmsg parser
  // allows them, but loading the generated Python code fails later)
  if (name.match(/^[a-zA-Z_][a-zA-Z0-9_]*$/) == undefined) return reject;
  return { name, isConstant: true };
} %}

# Constant Values

boolConstantValue -> assignment {% function(d, _, reject) {
  const valueText = d[0].split("#")[0].trim();
  if (valueText === "True" || valueText === "1") return { value: true, valueText };
  if (valueText === "False" || valueText === "0") return { value: false, valueText };
  return reject;
} %}

numericConstantValue -> assignment {% function(d, _, reject) {
  const valueText = d[0].split("#")[0].trim();
  const value = parseFloat(valueText);
  return !isNaN(value) ? { value, valueText } : reject;
} %}

bigintConstantValue -> assignment {% function(d, _, reject) {
  const valueText = d[0].split("#")[0].trim();
  try {
    const value = BigInt(valueText);
    return { value, valueText };
  } catch {
    return reject;
  }
} %}

stringConstantValue -> assignment {% function(d) { return { value: d[0], valueText: d[0] } } %}

# Basic Tokens

bool ->
    ("True" | "1") {% function(d) { return true } %}
  | ("False" | "0") {% function(d) { return false } %}

number -> %number {% function(d) { return parseFloat(d[0].value) } %}

# =...
assignment -> %assignment {% function(d) { return d[0].value.substr(1).trim() } %}

# Comments
comment -> %comment {% function(d) { return null } %}

# Line containing only whitespace
blankLine -> _ {% function(d) { return null } %}

# Optional whitespace
_ -> (null | %space) {% function(d) { return null } %}

# Required whitespace
__ -> %space {% function(d) { return null } %}

# Mark primitive types
simple -> null {% function() { return { isComplex: false } } %}

# Mark non-primitive types
complex -> null {% function() { return { isComplex: true } } %}

@{%
function extend(objs) {
  return objs.reduce((r, p) => ({ ...r, ...p }), {});
}
%}
