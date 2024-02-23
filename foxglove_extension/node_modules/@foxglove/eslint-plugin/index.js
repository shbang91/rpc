module.exports = {
  rules: {
    "license-header": require("./rules/license-header"),
    "strict-equality": require("./rules/strict-equality"),
    "no-return-promise-resolve": require("./rules/no-return-promise-resolve"),
    "no-boolean-parameters": require("./rules/no-boolean-parameters"),
    "no-restricted-imports": require("./rules/no-restricted-imports"),
    "prefer-hash-private": require("./rules/prefer-hash-private"),
  },
  configs: {
    base: require("./configs/base"),
    jest: require("./configs/jest"),
    react: require("./configs/react"),
    typescript: require("./configs/typescript"),
  },
};
