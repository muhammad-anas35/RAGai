import "../dialect-D9ZUZA4J.mjs";
import "../get-model-name-D4DUV7S2.mjs";
import { d as getBaseAdapter } from "../get-migration-Bf0TuCzm.mjs";
import "../utils-db7gNqd-.mjs";
import "../crypto-CFUhAR9W.mjs";
import "../misc-BwNc0MKr.mjs";
import "../schema-Bb7wzeK_.mjs";
import "../get-request-ip-D6st-mto.mjs";
import "../json-CnHxKYpj.mjs";
import "../url-CB8xCwz-.mjs";
import "../api-CkmycQ2x.mjs";
import "../cookies-D72PbWdz.mjs";
import "../session-AaRl3_x-.mjs";
import { n as createAuthContext, t as createBetterAuth } from "../base-CiRMFqet.mjs";
import "../password-BFQK0cLg.mjs";
import { BetterAuthError } from "@better-auth/core/error";

//#region src/context/init-minimal.ts
const initMinimal = async (options) => {
	const adapter = await getBaseAdapter(options, async () => {
		throw new BetterAuthError("Direct database connection requires Kysely. Please use `better-auth` instead of `better-auth/minimal`, or provide an adapter (drizzleAdapter, prismaAdapter, etc.)");
	});
	const getDatabaseType = (_database) => "unknown";
	const ctx = await createAuthContext(adapter, options, getDatabaseType);
	ctx.runMigrations = async function() {
		throw new BetterAuthError("Migrations are not supported in 'better-auth/minimal'. Please use 'better-auth' for migration support.");
	};
	return ctx;
};

//#endregion
//#region src/auth/minimal.ts
/**
* Better Auth initializer for minimal mode (without Kysely)
*/
const betterAuth = (options) => {
	return createBetterAuth(options, initMinimal);
};

//#endregion
export { betterAuth };