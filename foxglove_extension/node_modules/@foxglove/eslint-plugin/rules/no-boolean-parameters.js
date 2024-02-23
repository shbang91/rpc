const { ESLintUtils } = require("@typescript-eslint/utils");
const { unionTypeParts } = require("tsutils");
const ts = require("typescript");

function getData(param) {
  let paramName;
  if (param.type === "AssignmentPattern" && param.left.type === "Identifier") {
    paramName = param.left.name;
  } else if (param.type === "Identifier") {
    paramName = param.name;
  }

  let funcName;
  if (param.parent.id) {
    funcName = param.parent.id.name;
  } else if (param.parent.parent?.type === "VariableDeclarator") {
    funcName = param.parent.parent.id.name;
  } else if (
    ["Property", "MethodDefinition", "TSAbstractMethodDefinition"].includes(
      param.parent.parent?.type
    )
  ) {
    funcName = param.parent.parent.key.name;
  } else if (param.parent.parent?.type === "TSTypeAnnotation") {
    if (
      ["TSPropertySignature", "ClassProperty", "PropertyDefinition"].includes(
        param.parent.parent.parent?.type
      )
    ) {
      funcName = param.parent.parent.parent?.key.name;
    } else {
      funcName = param.parent.parent.parent?.name;
    }
  }

  return { paramName, funcName };
}

function getSuggestions(param, context) {
  const sourceCode = context.getSourceCode();
  const suggestions = [];

  let pattern;
  let typeAnnotation;
  if (param.type === "AssignmentPattern" && param.left.type === "Identifier") {
    pattern = `{ ${param.left.name} = ${sourceCode.getText(param.right)} }`;
    typeAnnotation = `{ ${sourceCode.getText(param.left)}${
      param.left.typeAnnotation == undefined ? ": boolean" : ""
    } }`;
  } else if (param.type === "Identifier") {
    pattern = `{ ${param.name} }`;
    typeAnnotation = `{ ${sourceCode.getText(param)}${
      param.typeAnnotation == undefined ? ": boolean" : ""
    } }`;
  }

  if (pattern && typeAnnotation) {
    suggestions.push({
      messageId: "wrapParamInObject",
      data: { pattern },
      fix(fixer) {
        return fixer.replaceText(param, `${pattern}: ${typeAnnotation}`);
      },
    });
  }

  return suggestions;
}

module.exports = {
  meta: {
    type: "suggestion",
    hasSuggestions: true,
    fixable: "code",
    schema: [
      {
        type: "object",
        properties: {
          allowLoneParameter: {
            type: "boolean",
            default: false,
          },
        },
        additionalProperties: false,
      },
    ],
    messages: {
      booleanTrap: `Don't use raw boolean value{{paramInfo}} as a parameter{{funcInfo}}; call sites will appear ambiguous (the "boolean trap")`,
      wrapParamInObject: `Replace with {{pattern}}`,
    },
  },

  create(context) {
    const { allowLoneParameter } = context.options?.[0] ?? [
      { allowLoneParameter: false },
    ];
    const { esTreeNodeToTSNodeMap, program } =
      ESLintUtils.getParserServices(context);
    const checker = program.getTypeChecker();
    return {
      ":function > AssignmentPattern.params, :function > [typeAnnotation].params, TSFunctionType > [typeAnnotation].params, TSEmptyBodyFunctionExpression > [typeAnnotation].params":
        (param) => {
          if (allowLoneParameter && param.parent.params.length === 1) {
            return;
          }
          const tsNode = esTreeNodeToTSNodeMap.get(param);
          const type = checker.getTypeAtLocation(tsNode);
          const parts = unionTypeParts(type);
          const isBoolean =
            parts.some((part) => part.flags & ts.TypeFlags.BooleanLike) &&
            parts.every(
              (part) =>
                part.flags &
                (ts.TypeFlags.BooleanLike |
                  ts.TypeFlags.Void |
                  ts.TypeFlags.Undefined |
                  ts.TypeFlags.Null |
                  ts.TypeFlags.Never)
            );
          const { paramName, funcName } = getData(param);
          if (isBoolean) {
            context.report({
              node: param,
              messageId: "booleanTrap",
              data: {
                paramInfo: paramName ? ` '${paramName}'` : "",
                funcInfo: funcName ? ` to '${funcName}'` : "",
              },
              suggest: getSuggestions(param, context),
            });
          }
        },
    };
  },
};
