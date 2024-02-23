import { readFile, readdir } from "fs/promises";
import { join, sep } from "path";

import { parse } from "./parse";

type TypeInformation = {
  fullType: string;
  complexTypes: string[];
  msgDefinitionString: string;
};

async function main() {
  if (process.argv.length !== 4) {
    console.error("Usage: gendeps <msgdefs-dir> <msg-file>");
    process.exit(1);
  }

  const rootPath = process.argv[2]!;
  const filename = process.argv[3]!;
  const fullTypeName = getFullTypeFromFilename(filename, rootPath);

  const res = await loadType(fullTypeName, rootPath, getPackageName(fullTypeName));
  console.log(res.msgDefinitionString);

  const complexTypes = res.complexTypes;
  const seenTypes = new Set<string>();
  seenTypes.add(res.fullType);

  while (complexTypes.length > 0) {
    const complexType = complexTypes.shift()!;
    const curRes = await loadType(complexType, rootPath, getPackageName(complexType));

    console.log("================================================================================");
    console.log(`MSG: ${curRes.fullType}`);
    console.log(curRes.msgDefinitionString);

    for (const complexSubType of curRes.complexTypes) {
      if (!seenTypes.has(complexSubType)) {
        complexTypes.push(complexSubType);
        seenTypes.add(complexSubType);
      }
    }
  }
}

function getFullType(typeName: string, currentPackage: string | undefined): string {
  if (typeName.includes("/")) {
    return typeName;
  }
  if (!currentPackage) {
    throw new Error(`Cannot resolve relative type name ${typeName}`);
  }
  return `${currentPackage}/${typeName}`;
}

function getPackageName(typeName: string): string {
  return typeName.split("/")[0]!;
}

function getBaseType(typeName: string): string {
  return typeName.split("/").pop()!;
}

async function loadType(
  typeName: string,
  rootPath: string,
  currentPackage: string,
): Promise<TypeInformation> {
  const fullType = getFullType(typeName, currentPackage);
  const packageName = getPackageName(fullType);
  const baseType = getBaseType(typeName);
  const msgDefinitionString = await readFileFromBaseDir(
    `${baseType}.msg`,
    join(rootPath, packageName),
  );
  if (msgDefinitionString == undefined) {
    throw new Error(
      `Failed to load definition for type ${typeName} (current package: ${currentPackage})`,
    );
  }

  const complexTypes: string[] = [];
  const msgDefinitions = parse(msgDefinitionString, { ros2: true, skipTypeFixup: true });
  for (const msgdef of msgDefinitions) {
    for (const definition of msgdef.definitions) {
      if (definition.isComplex === true && !complexTypes.includes(definition.type)) {
        complexTypes.push(getFullType(definition.type, currentPackage));
      }
    }
  }

  return { fullType, complexTypes, msgDefinitionString };
}

function getFullTypeFromFilename(filename: string, rootPath: string): string {
  const pathParts = rootPath.split(sep);
  const filenameParts = filename.replace(/\.msg$/, "").split(sep);

  // Remove pathParts from msgFileParts
  for (const pathPart of pathParts) {
    if (pathPart !== filenameParts[0]) {
      console.log(`${pathPart} !== ${filenameParts[0]!}`);
      throw new Error(`<msg-file> "${filename}" must be under <msgdefs-dir> "${rootPath}"`);
    }
    filenameParts.shift();
  }

  const packageName = filenameParts.shift()!;
  const baseType = filenameParts[filenameParts.length - 1]!;
  return `${packageName}/${baseType}`;
}

// Recursively search inside a directory for a file with a given name
async function readFileFromBaseDir(filename: string, baseDir: string): Promise<string | undefined> {
  const files = await readdir(baseDir, { withFileTypes: true });
  for (const file of files) {
    if (file.isDirectory()) {
      const contents = await readFileFromBaseDir(filename, join(baseDir, file.name));
      if (contents != undefined) {
        return contents;
      }
    } else if (file.isFile() && file.name === filename) {
      return await readFile(join(baseDir, file.name), { encoding: "utf8" });
    }
  }
  return undefined;
}

void main();
