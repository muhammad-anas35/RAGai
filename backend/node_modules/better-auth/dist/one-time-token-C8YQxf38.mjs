import { r as generateRandomString } from "./crypto-CFUhAR9W.mjs";
import { u as sessionMiddleware } from "./session-AaRl3_x-.mjs";
import * as z from "zod";
import { createHash } from "@better-auth/utils/hash";
import { createAuthEndpoint } from "@better-auth/core/api";
import { base64Url } from "@better-auth/utils/base64";

//#region src/plugins/one-time-token/utils.ts
const defaultKeyHasher = async (token) => {
	const hash = await createHash("SHA-256").digest(new TextEncoder().encode(token));
	return base64Url.encode(new Uint8Array(hash), { padding: false });
};

//#endregion
//#region src/plugins/one-time-token/index.ts
const oneTimeToken = (options) => {
	const opts = {
		storeToken: "plain",
		...options
	};
	async function storeToken(ctx, token) {
		if (opts.storeToken === "hashed") return await defaultKeyHasher(token);
		if (typeof opts.storeToken === "object" && "type" in opts.storeToken && opts.storeToken.type === "custom-hasher") return await opts.storeToken.hash(token);
		return token;
	}
	return {
		id: "one-time-token",
		endpoints: {
			generateOneTimeToken: createAuthEndpoint("/one-time-token/generate", {
				method: "GET",
				use: [sessionMiddleware]
			}, async (c) => {
				if (opts?.disableClientRequest && c.request) throw c.error("BAD_REQUEST", { message: "Client requests are disabled" });
				const session = c.context.session;
				const token = opts?.generateToken ? await opts.generateToken(session, c) : generateRandomString(32);
				const expiresAt = new Date(Date.now() + (opts?.expiresIn ?? 3) * 60 * 1e3);
				const storedToken = await storeToken(c, token);
				await c.context.internalAdapter.createVerificationValue({
					value: session.session.token,
					identifier: `one-time-token:${storedToken}`,
					expiresAt
				});
				return c.json({ token });
			}),
			verifyOneTimeToken: createAuthEndpoint("/one-time-token/verify", {
				method: "POST",
				body: z.object({ token: z.string().meta({ description: "The token to verify. Eg: \"some-token\"" }) })
			}, async (c) => {
				const { token } = c.body;
				const storedToken = await storeToken(c, token);
				const verificationValue = await c.context.internalAdapter.findVerificationValue(`one-time-token:${storedToken}`);
				if (!verificationValue) throw c.error("BAD_REQUEST", { message: "Invalid token" });
				await c.context.internalAdapter.deleteVerificationValue(verificationValue.id);
				if (verificationValue.expiresAt < /* @__PURE__ */ new Date()) throw c.error("BAD_REQUEST", { message: "Token expired" });
				const session = await c.context.internalAdapter.findSession(verificationValue.value);
				if (!session) throw c.error("BAD_REQUEST", { message: "Session not found" });
				return c.json(session);
			})
		}
	};
};

//#endregion
export { oneTimeToken as t };