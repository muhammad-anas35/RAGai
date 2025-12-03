import { o as Auth } from "../index-COnelCGa.mjs";
import "../types-CEepZ-RG.mjs";
import "../helper-DFzV6jvx.mjs";
import "../plugins-Dmw3tk5H.mjs";
import { BetterAuthOptions } from "@better-auth/core";

//#region src/auth/minimal.d.ts
/**
 * Better Auth initializer for minimal mode (without Kysely)
 */
declare const betterAuth: <Options extends BetterAuthOptions>(options: Options & Record<never, never>) => Auth<Options>;
//#endregion
export { type BetterAuthOptions, betterAuth };