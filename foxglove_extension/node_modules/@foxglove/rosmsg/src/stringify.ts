import { MessageDefinition, DefaultValue } from "@foxglove/message-definition";

// Converts a ROS message definition (http://wiki.ros.org/msg) into a canonical
// message description format that is suitable for MD5 checksum generation
export function stringify(msgDefs: MessageDefinition[]): string {
  let output = "";
  for (let i = 0; i < msgDefs.length; i++) {
    const msgDef = msgDefs[i] as MessageDefinition;
    const constants = msgDef.definitions.filter(({ isConstant }) => isConstant);
    const variables = msgDef.definitions.filter(
      ({ isConstant }) => isConstant == undefined || !isConstant,
    );

    if (i > 0) {
      output +=
        "\n================================================================================\n";
      output += `MSG: ${msgDef.name ?? ""}\n`;
    }

    for (const def of constants) {
      output += `${def.type} ${def.name} = ${def.valueText ?? String(def.value)}\n`;
    }
    if (variables.length > 0) {
      if (output.length > 0) {
        output += "\n";
      }
      for (const def of variables) {
        const upperBound = def.upperBound != undefined ? `<=${def.upperBound}` : "";
        const arrayLength =
          def.arrayLength != undefined
            ? String(def.arrayLength)
            : def.arrayUpperBound != undefined
            ? `<=${def.arrayUpperBound}`
            : "";
        const array = def.isArray === true ? `[${arrayLength}]` : "";
        const defaultValue =
          def.defaultValue != undefined ? ` ${stringifyDefaultValue(def.defaultValue)}` : "";
        output += `${def.type}${upperBound}${array} ${def.name}${defaultValue}\n`;
      }
    }
  }

  return output.trimEnd();
}

function stringifyDefaultValue(value: DefaultValue): string {
  if (Array.isArray(value)) {
    return `[${value
      .map((x) => (typeof x === "bigint" ? x.toString() : JSON.stringify(x)))
      .join(", ")}]`;
  }
  return typeof value === "bigint" ? value.toString() : JSON.stringify(value);
}
