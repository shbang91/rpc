import {
  ConstantValue,
  MessageDefinition,
  MessageDefinitionField,
} from "@foxglove/message-definition";
import { Grammar, Parser } from "nearley";

import { ROS2IDL_GRAMMAR, normalizeType } from "./parse";

/**
 *
 * @param messageDefinition - ros2idl decoded message definition string
 * @returns - parsed message definition
 */

export function parseRos2idl(messageDefinition: string): MessageDefinition[] {
  return buildRos2idlType(messageDefinition, ROS2IDL_GRAMMAR);
}
type RawIdlDefinition = {
  definitions: (RawIdlDefinition | RawIdlFieldDefinition)[];
  name: string;
  definitionType: "module" | "struct";
};
type RawIdlFieldDefinition = Partial<MessageDefinitionField> & {
  definitions: undefined;
  definitionType: "typedef";
  constantUsage?: [keyof MessageDefinitionField, string][];
};
function buildRos2idlType(messageDefinition: string, grammar: Grammar): MessageDefinition[] {
  const parser = new Parser(grammar);
  parser.feed(messageDefinition);
  const results = parser.finish();

  if (results.length === 0) {
    throw new Error(
      `Could not parse message definition (unexpected end of input): '${messageDefinition}'`,
    );
  }
  const result = results[0] as RawIdlDefinition[];
  const processedResult = postProcessIdlDefinitions(result);
  for (const { definitions } of processedResult) {
    for (const definition of definitions) {
      definition.type = normalizeType(definition.type);
    }
  }

  return processedResult;
}
function traverseIdl(
  path: (RawIdlDefinition | RawIdlFieldDefinition)[],
  processNode: (path: (RawIdlDefinition | RawIdlFieldDefinition)[]) => void,
) {
  const currNode = path[path.length - 1]!;
  const children: (RawIdlDefinition | RawIdlFieldDefinition)[] | undefined = currNode.definitions;
  if (children) {
    children.forEach((n) => traverseIdl([...path, n], processNode));
  }
  processNode(path);
}
function postProcessIdlDefinitions(definitions: RawIdlDefinition[]): MessageDefinition[] {
  const finalDefs: MessageDefinition[] = [];
  // Need to update the names of modules and structs to be in their respective namespaces
  for (const definition of definitions) {
    const typedefMap = new Map<string, Partial<RawIdlFieldDefinition>>();
    const constantValueMap = new Map<string, ConstantValue>();
    // build constant and typedef maps
    traverseIdl([definition], (path) => {
      const node = path[path.length - 1] as RawIdlFieldDefinition;
      if (node.definitionType === "typedef") {
        // typedefs must have a name
        const { definitionType: _definitionType, name: _name, ...partialDef } = node;
        typedefMap.set(node.name!, partialDef);
      } else if (node.isConstant === true) {
        constantValueMap.set(node.name!, node.value);
      }
    });

    // modify ast field nodes in-place to replace typedefs and constants
    // also fix up names to use ros package resource names
    traverseIdl([definition], (path) => {
      const node = path[path.length - 1]!;
      if (node.definitions != undefined) {
        return;
      }
      // replace field definition with corresponding typedef aliased definition
      if (node.type && typedefMap.has(node.type)) {
        Object.assign(node, { ...typedefMap.get(node.type), name: node.name });
      }

      // need to iterate through keys because this can occur on arrayLength, upperBound, arrayUpperBound, value, defaultValue
      for (const [key, constantName] of node.constantUsage ?? []) {
        if (constantValueMap.has(constantName)) {
          (node[key] as ConstantValue) = constantValueMap.get(constantName);
        } else {
          throw new Error(
            `Could not find constant <${constantName}> for field <${
              node.name ?? "undefined"
            }> in <${definition.name}>`,
          );
        }
      }
      delete node.constantUsage;

      if (node.type != undefined) {
        node.type = node.type.replace(/::/g, "/");
      }
    });

    const flattened = flattenIdlNamespaces(definition);
    finalDefs.push(...flattened);
  }

  return finalDefs;
}
function flattenIdlNamespaces(definition: RawIdlDefinition): MessageDefinition[] {
  const flattened: MessageDefinition[] = [];

  traverseIdl([definition], (path) => {
    const node = path[path.length - 1] as RawIdlDefinition;
    if (node.definitionType === "module") {
      const moduleDefs = node.definitions.filter((d) => d.definitionType !== "typedef");
      // only add modules if all fields are constants (complex leaf)
      if (moduleDefs.every((child) => (child as RawIdlFieldDefinition).isConstant)) {
        flattened.push({
          name: path.map((n) => n.name).join("/"),
          definitions: moduleDefs as MessageDefinitionField[],
        });
      }
    } else if (node.definitionType === "struct") {
      // all structs are leaf nodes to be added
      flattened.push({
        name: path.map((n) => n.name).join("/"),
        definitions: node.definitions as MessageDefinitionField[],
      });
    }
  });

  return flattened;
}
