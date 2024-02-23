# @foxglove/message-definition

> Defines common TypeScript types for message definition schemas (ROS .msg, Protobuf, FlatBuffers, IDL, PX4 ULog, JSON Schema, etc).

## Why is this useful?

Several interface definition languages exist today for describing the structure of messages. These languages are often used to generate code for serialization and deserialization of messages. This package defines a common representation in TypeScript for interface definitions, sometimes referred to as message definitions, so they can be reasoned about in a generic way. A concrete example of this is in [Foxglove Studio](https://github.com/foxglove/studio), which supports many different message serializations but provides common functionality across all of them such as [Message Path Syntax](https://foxglove.dev/docs/studio/app-concepts/message-path-syntax) and structured message display.

## Examples

Here is a an example of a ROS 2 message definition (`.msg` file) and its corresponding TypeScript representation:

`UserAccount.msg`:

```
string username
Account account
============
MSG: custom_type/Account
string name
uint16 id
```

`UserAccount.ts`:

```typescript
[
  {
    definitions: [
      { type: "string", name: "username" },
      { type: "custom_type/Account", name: "account", isComplex: true },
    ],
  },
  {
    name: "custom_type/Account",
    definitions: [
      { type: "string", name: "name" },
      { type: "uint16", name: "id" },
    ],
  },
];
```

Note that this package only provides type definitions, not any functionality for parsing or generating message definitions. For that, see [foxglove/rosmsg](https://github.com/foxglove/rosmsg) or other packages that depend on this one.

## License

@foxglove/message-definition is licensed under [MIT License](https://opensource.org/licenses/MIT).

## Releasing

1. Run `yarn version --[major|minor|patch]` to bump version
2. Run `git push && git push --tags` to push new tag
3. GitHub Actions will take care of the rest

## Stay in touch

Join our [Slack channel](https://foxglove.dev/join-slack) to ask questions, share feedback, and stay up to date on what our team is working on.
