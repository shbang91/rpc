import { MessageDefinition } from "@foxglove/message-definition";
import { Md5 } from "md5-typescript";

const BUILTIN_TYPES = new Set([
  "int8",
  "uint8",
  "int16",
  "uint16",
  "int32",
  "uint32",
  "int64",
  "uint64",
  "float32",
  "float64",
  "string",
  "bool",
  "char",
  "byte",
  "time",
  "duration",
]);

/**
 * Converts a ROS 1 message definition (http://wiki.ros.org/msg) into an md5 checksum using the same
 * approach as `gendeps --md5` from ROS 1.
 * @param msgDefs The ROS message definition to generate a checksum for, and all dependent
 * sub-messages
 * @returns An md5 checksum string
 */
export function md5(msgDefs: MessageDefinition[]): string {
  if (msgDefs.length === 0) {
    throw new Error(`Cannot produce md5sum for empty msgDefs`);
  }

  const subMsgDefs = new Map<string, MessageDefinition>();
  for (const msgDef of msgDefs) {
    if (msgDef.name != undefined) {
      subMsgDefs.set(msgDef.name, msgDef);
    }
  }

  const first = msgDefs[0]!;
  return computeMessageMd5(first, subMsgDefs);
}

function computeMessageMd5(
  msgDef: MessageDefinition,
  subMsgDefs: Map<string, MessageDefinition>,
): string {
  let output = "";
  const constants = msgDef.definitions.filter(({ isConstant }) => isConstant);
  const variables = msgDef.definitions.filter(
    ({ isConstant }) => isConstant == undefined || !isConstant,
  );

  for (const def of constants) {
    output += `${def.type} ${def.name}=${def.valueText ?? String(def.value)}\n`;
  }

  for (const def of variables) {
    if (isBuiltin(def.type)) {
      const arrayLength = def.arrayLength != undefined ? String(def.arrayLength) : "";
      const array = def.isArray === true ? `[${arrayLength}]` : "";
      output += `${def.type}${array} ${def.name}\n`;
    } else {
      const subMsgDef = subMsgDefs.get(def.type);
      if (subMsgDef == undefined) {
        throw new Error(`Missing definition for submessage type "${def.type}"`);
      }
      const subMd5 = computeMessageMd5(subMsgDef, subMsgDefs);
      output += `${subMd5} ${def.name}\n`;
    }
  }

  output = output.trimEnd();
  return Md5.init(output);
}

function isBuiltin(typeName: string): boolean {
  return BUILTIN_TYPES.has(typeName);
}
