// https://github.com/eslint/eslint/blob/dd58cd4afa6ced9016c091fc99a702c97a3e44f0/lib/rules/utils/ast-utils.js#L155
function isNullLiteral(node) {
  return (
    node.type === "Literal" &&
    // eslint-disable-next-line @foxglove/strict-equality
    node.value === null &&
    !node.regex &&
    !node.bigint
  );
}

// - Prefer double equals when comparing with undefined
// - Prefer undefined over null
// - Prefer triple equals everywhere else
module.exports = {
  meta: {
    type: "suggestion",
    fixable: "code",
    schema: [],
    messages: {
      unexpected: "Prefer '{{expectedOperator}}' over '{{actualOperator}}'.",
      unexpectedStrict: `Prefer 'x {{expectedOp}} {{literal}}' to catch both null and undefined`,
    },
  },

  create(context) {
    const sourceCode = context.getSourceCode();

    function isUndefinedLiteral(node) {
      return node.type === "Identifier" && node.name === "undefined";
    }

    return {
      BinaryExpression: (node) => {
        function preferDoubleEqual(node, loc, literal) {
          let expectedOp = node.operator.substring(0, 2);

          context.report({
            node,
            loc,
            messageId: "unexpectedStrict",
            data: { expectedOp, literal },
          });
        }

        const operatorToken = sourceCode.getFirstTokenBetween(
          node.left,
          node.right,
          (token) => token.value === node.operator
        );

        // If either side is undefined, prefer double equals
        if (isUndefinedLiteral(node.left) || isUndefinedLiteral(node.right)) {
          if (node.operator === "===" || node.operator === "!==") {
            preferDoubleEqual(node, operatorToken.loc, "undefined");
          }
        }

        // If either side is null, prefer double equals
        else if (isNullLiteral(node.left) || isNullLiteral(node.right)) {
          if (node.operator === "===" || node.operator === "!==") {
            preferDoubleEqual(node, operatorToken.loc, "null");
          }
        }

        // Otherwise, prefer triple equals
        else if (node.operator === "==" || node.operator === "!=") {
          context.report({
            node,
            loc: operatorToken.loc,
            messageId: "unexpected",
            data: {
              expectedOperator: node.operator + "=",
              actualOperator: node.operator,
            },
          });
        }
      },
    };
  },
};
