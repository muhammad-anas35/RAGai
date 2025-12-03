import { i as initGetDefaultModelName, n as initGetFieldName, r as initGetDefaultFieldName, t as initGetModelName } from "../get-model-name-D4DUV7S2.mjs";
import "../utils-db7gNqd-.mjs";
import "../crypto-CFUhAR9W.mjs";
import "../misc-BwNc0MKr.mjs";
import "../json-CnHxKYpj.mjs";
import { n as initGetFieldAttributes, r as initGetIdField, t as createAdapterFactory } from "../adapter-factory-HF3JB9cT.mjs";

export * from "@better-auth/core/db/adapter"

//#region src/adapters/index.ts
/**
* @deprecated Use `createAdapterFactory` instead. This export will be removed in the next major version.
*/
const createAdapter = createAdapterFactory;

//#endregion
export { createAdapter, createAdapterFactory, initGetDefaultFieldName, initGetDefaultModelName, initGetFieldAttributes, initGetFieldName, initGetIdField, initGetModelName };