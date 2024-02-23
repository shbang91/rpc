module.exports = {
  extends: [
    "plugin:@typescript-eslint/strict-type-checked",
    "plugin:@typescript-eslint/stylistic-type-checked",
    "plugin:import/typescript",
  ],
  parser: "@typescript-eslint/parser",
  rules: {
    "@typescript-eslint/no-meaningless-void-operator": [
      "error",
      { checkNever: true },
    ],
    "@foxglove/no-boolean-parameters": "error",

    // `<T>x` style assertions are not compatible with JSX code,
    // so for consistency we prefer `x as T` everywhere.
    "@typescript-eslint/consistent-type-assertions": [
      "error",
      {
        assertionStyle: "as",
      },
    ],

    "@typescript-eslint/explicit-module-boundary-types": "error",
    "@typescript-eslint/no-confusing-non-null-assertion": "error",
    "@typescript-eslint/no-explicit-any": "error",
    "@typescript-eslint/no-floating-promises": "error",
    "@typescript-eslint/no-shadow": "error",
    "@typescript-eslint/no-unnecessary-boolean-literal-compare": "error",

    "no-unused-expressions": "off",
    "@typescript-eslint/no-unused-expressions": [
      "error",
      { enforceForJSX: true },
    ],

    // The ! assertion may be used sparingly in cases where tsc cannot automatically do bounds
    // checking such as indexed array iteration
    "@typescript-eslint/no-non-null-assertion": "off",

    // unused vars must have `_` prefix
    "@typescript-eslint/no-unused-vars": [
      "error",
      {
        vars: "all",
        args: "after-used",
        varsIgnorePattern: "^_",
        argsIgnorePattern: "^_",
      },
    ],

    "@typescript-eslint/prefer-includes": "error",
    "@typescript-eslint/promise-function-async": "error",

    // Async functions without await are used to satisfy interface requirements
    "@typescript-eslint/require-await": "off",

    "@typescript-eslint/return-await": ["error", "always"],

    // require all cases to be checked in switch statements
    "@typescript-eslint/switch-exhaustiveness-check": "error",

    // require strict boolean comparisons for all types, except allow
    // nullable string conditionals to avoid verbose "empty or undefined" checks
    "@typescript-eslint/strict-boolean-expressions": [
      "error",
      {
        allowString: true,
        allowNullableString: true,
        allowNumber: false,
        allowNullableNumber: false,
        allowNullableBoolean: false,
        allowNullableObject: true,
        allowAny: false,
      },
    ],

    "@typescript-eslint/prefer-nullish-coalescing": [
      "error",
      { ignoreConditionalTests: true },
    ],

    // We find the following rules from the strict/stylistic configs to be overly pedantic and not
    // so useful
    "@typescript-eslint/consistent-type-definitions": "off",
    "@typescript-eslint/consistent-indexed-object-style": "off",
    "@typescript-eslint/no-dynamic-delete": "off",
    "@typescript-eslint/dot-notation": "off",
    "@typescript-eslint/array-type": "off",
    "@typescript-eslint/no-invalid-void-type": "off",
    "@typescript-eslint/unified-signatures": "off",
  },
};
