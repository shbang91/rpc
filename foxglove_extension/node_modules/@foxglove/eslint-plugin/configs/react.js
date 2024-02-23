module.exports = {
  extends: ["plugin:react/recommended", "plugin:react-hooks/recommended"],
  settings: {
    react: {
      version: "detect",
    },
  },
  rules: {
    "react/no-unused-prop-types": "error",

    // unnecessary to import React with next.js and common patterns
    "react/react-in-jsx-scope": "off",

    // this rule is slow, and unnecessary with typescript
    "react/prop-types": "off",

    // require rel="noopener" or rel="noreferrer" with target="_blank" for security
    "react/jsx-no-target-blank": ["error", { allowReferrer: true }],

    // deprecated mui/styled in our frontend code in favor of tss-react/mui
    "@foxglove/no-restricted-imports": [
      "error",
      {
        name: "@mui/material",
        importNames: ["styled"],
        message:
          "@mui/styled has performance implications. Use tss-react/mui instead.",
      },
      {
        name: "@mui/styles",
        message:
          "@mui/styles has performance implications. Use tss-react/mui instead.",
      },
      {
        name: "@mui/material/styles/styled",
        message:
          "@mui/styled has performance implications. Use tss-react/mui instead.",
      },
      {
        name: "@emotion/styled",
        message:
          "@emotion/styled has performance implications. Use tss-react/mui instead.",
      },
    ],
  },
};
