const builtinRules = require("eslint/use-at-your-own-risk").builtinRules;

/**
 * Re-export the `no-restricted-imports` rules so that we can use them as
 * `@foxglove/no-restricted-imports` instead of `no-restricted-imports`.
 *
 * This is to make accidental overrides more difficult. Without this any local
 * eslint config that defines its own `no-restricted-imports` rule will ignore
 * the list from the base config.
 *
 * Note that `builtInRules` isn't actually deprecated. It's marked "unstable"
 * because ESLint doesn't guarantee that they remain stable between minor
 * versions.
 */
module.exports = builtinRules.get("no-restricted-imports");
