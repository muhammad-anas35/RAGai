import { BetterAuthError } from "@better-auth/core/error";
import { createHash } from "@better-auth/utils/hash";
import { xchacha20poly1305 } from "@noble/ciphers/chacha.js";
import { bytesToHex, hexToBytes, managedNonce, utf8ToBytes } from "@noble/ciphers/utils.js";
import { hkdf } from "@noble/hashes/hkdf.js";
import { sha256 } from "@noble/hashes/sha2.js";
import { EncryptJWT, SignJWT, base64url, calculateJwkThumbprint, jwtDecrypt, jwtVerify } from "jose";
import { hex } from "@better-auth/utils/hex";
import { scryptAsync } from "@noble/hashes/scrypt.js";
import { hexToBytes as hexToBytes$1 } from "@noble/hashes/utils.js";
import { createRandomStringGenerator } from "@better-auth/utils/random";

//#region src/crypto/buffer.ts
/**
* Compare two buffers in constant time.
*/
function constantTimeEqual(a, b) {
	if (typeof a === "string") a = new TextEncoder().encode(a);
	if (typeof b === "string") b = new TextEncoder().encode(b);
	const aBuffer = new Uint8Array(a);
	const bBuffer = new Uint8Array(b);
	let c = aBuffer.length ^ bBuffer.length;
	const length = Math.max(aBuffer.length, bBuffer.length);
	for (let i = 0; i < length; i++) c |= (i < aBuffer.length ? aBuffer[i] : 0) ^ (i < bBuffer.length ? bBuffer[i] : 0);
	return c === 0;
}

//#endregion
//#region src/crypto/jwt.ts
async function signJWT(payload, secret, expiresIn = 3600) {
	return await new SignJWT(payload).setProtectedHeader({ alg: "HS256" }).setIssuedAt().setExpirationTime(Math.floor(Date.now() / 1e3) + expiresIn).sign(new TextEncoder().encode(secret));
}
async function verifyJWT(token, secret) {
	try {
		return (await jwtVerify(token, new TextEncoder().encode(secret))).payload;
	} catch (error) {
		return null;
	}
}
const info = new Uint8Array([
	66,
	101,
	116,
	116,
	101,
	114,
	65,
	117,
	116,
	104,
	46,
	106,
	115,
	32,
	71,
	101,
	110,
	101,
	114,
	97,
	116,
	101,
	100,
	32,
	69,
	110,
	99,
	114,
	121,
	112,
	116,
	105,
	111,
	110,
	32,
	75,
	101,
	121
]);
const now = () => Date.now() / 1e3 | 0;
const alg = "dir";
const enc = "A256CBC-HS512";
async function symmetricEncodeJWT(payload, secret, salt, expiresIn = 3600) {
	const encryptionSecret = hkdf(sha256, new TextEncoder().encode(secret), new TextEncoder().encode(salt), info, 64);
	const thumbprint = await calculateJwkThumbprint({
		kty: "oct",
		k: base64url.encode(encryptionSecret)
	}, "sha256");
	return await new EncryptJWT(payload).setProtectedHeader({
		alg,
		enc,
		kid: thumbprint
	}).setIssuedAt().setExpirationTime(now() + expiresIn).setJti(crypto.randomUUID()).encrypt(encryptionSecret);
}
async function symmetricDecodeJWT(token, secret, salt) {
	if (!token) return null;
	try {
		const { payload } = await jwtDecrypt(token, async ({ kid }) => {
			const encryptionSecret = hkdf(sha256, new TextEncoder().encode(secret), new TextEncoder().encode(salt), info, 64);
			if (kid === void 0) return encryptionSecret;
			if (kid === await calculateJwkThumbprint({
				kty: "oct",
				k: base64url.encode(encryptionSecret)
			}, "sha256")) return encryptionSecret;
			throw new Error("no matching decryption secret");
		}, {
			clockTolerance: 15,
			keyManagementAlgorithms: [alg],
			contentEncryptionAlgorithms: [enc, "A256GCM"]
		});
		return payload;
	} catch (error) {
		return null;
	}
}

//#endregion
//#region src/crypto/password.ts
const config = {
	N: 16384,
	r: 16,
	p: 1,
	dkLen: 64
};
async function generateKey(password, salt) {
	return await scryptAsync(password.normalize("NFKC"), salt, {
		N: config.N,
		p: config.p,
		r: config.r,
		dkLen: config.dkLen,
		maxmem: 128 * config.N * config.r * 2
	});
}
const hashPassword = async (password) => {
	const salt = hex.encode(crypto.getRandomValues(new Uint8Array(16)));
	const key = await generateKey(password, salt);
	return `${salt}:${hex.encode(key)}`;
};
const verifyPassword = async ({ hash, password }) => {
	const [salt, key] = hash.split(":");
	if (!salt || !key) throw new BetterAuthError("Invalid password hash");
	return constantTimeEqual(await generateKey(password, salt), hexToBytes$1(key));
};

//#endregion
//#region src/crypto/random.ts
const generateRandomString = createRandomStringGenerator("a-z", "0-9", "A-Z", "-_");

//#endregion
//#region src/crypto/index.ts
const symmetricEncrypt = async ({ key, data }) => {
	const keyAsBytes = await createHash("SHA-256").digest(key);
	const dataAsBytes = utf8ToBytes(data);
	return bytesToHex(managedNonce(xchacha20poly1305)(new Uint8Array(keyAsBytes)).encrypt(dataAsBytes));
};
const symmetricDecrypt = async ({ key, data }) => {
	const keyAsBytes = await createHash("SHA-256").digest(key);
	const dataAsBytes = hexToBytes(data);
	const chacha = managedNonce(xchacha20poly1305)(new Uint8Array(keyAsBytes));
	return new TextDecoder().decode(chacha.decrypt(dataAsBytes));
};

//#endregion
export { verifyPassword as a, symmetricEncodeJWT as c, hashPassword as i, verifyJWT as l, symmetricEncrypt as n, signJWT as o, generateRandomString as r, symmetricDecodeJWT as s, symmetricDecrypt as t, constantTimeEqual as u };