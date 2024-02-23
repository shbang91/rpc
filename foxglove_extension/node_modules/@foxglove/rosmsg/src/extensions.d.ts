declare module "*.ne" {
  import type { CompiledRules } from "nearley";

  const compiledRules: CompiledRules;
  export default compiledRules;
}
