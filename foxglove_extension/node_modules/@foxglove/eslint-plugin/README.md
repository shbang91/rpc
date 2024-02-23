# @foxglove/eslint-plugin

[![npm package](https://img.shields.io/npm/v/@foxglove/eslint-plugin)](https://www.npmjs.com/package/@foxglove/eslint-plugin)

Foxglove default eslint configuration & rules.

Please err on the side of conservative changes to this repo - multiple Foxglove projects should adopt a change before making it a default.

## Rules

See [rules/README.md](rules/README.md) for details on each rule.

## Installation

The following configurations are available:

- `plugin:@foxglove/base`
- `plugin:@foxglove/jest`
- `plugin:@foxglove/react`
- `plugin:@foxglove/typescript`

**Typescript + React Example**

```sh
yarn add -D \
    @foxglove/eslint-plugin \
    @typescript-eslint/eslint-plugin \
    @typescript-eslint/parser \
    eslint \
    eslint-config-prettier \
    eslint-plugin-es \
    eslint-plugin-filenames \
    eslint-plugin-import \
    eslint-plugin-jest \
    eslint-plugin-prettier \
    eslint-plugin-react \
    eslint-plugin-react-hooks \
    prettier
```

In your `.eslintrc.js`:

```js
module.exports = {
  extends: [
    "plugin:@foxglove/base",
    "plugin:@foxglove/jest",
    "plugin:@foxglove/react",
  ],
  overrides: [
    {
      files: ["*.ts", "*.tsx"],
      extends: ["plugin:@foxglove/typescript"],
      parserOptions: {
        project: "tsconfig.json",
      },
    },
  ],
};
```

You can add `"plugin:@foxglove/typescript"` to the top level `extends` instead of using `overrides` if your project contains no `.js` files.

## License

@foxglove/eslint-plugin is released under the [MIT License](/LICENSE.md).

## Releasing

**Note**: You must use npm 7+ (not yarn) to test this repo locally, due to the self link in `package.json`.

```sh
tag=$(npm version minor) && echo "$tag"
git push && git push origin "$tag"
```

## Stay in touch

Join our [Slack channel](https://foxglove.dev/join-slack) to ask questions, share feedback, and stay up to date on what our team is working on.
