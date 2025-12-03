import { t as capitalizeFirstLetter } from "./misc-BwNc0MKr.mjs";
import { n as getClientConfig, t as createDynamicPathProxy } from "./proxy-DplNCOES.mjs";

//#region src/client/vanilla.ts
function createAuthClient(options) {
	const { pluginPathMethods, pluginsActions, pluginsAtoms, $fetch, atomListeners, $store } = getClientConfig(options);
	let resolvedHooks = {};
	for (const [key, value] of Object.entries(pluginsAtoms)) resolvedHooks[`use${capitalizeFirstLetter(key)}`] = value;
	return createDynamicPathProxy({
		...pluginsActions,
		...resolvedHooks,
		$fetch,
		$store
	}, $fetch, pluginPathMethods, pluginsAtoms, atomListeners);
}

//#endregion
//#region src/client/index.ts
const InferPlugin = () => {
	return {
		id: "infer-server-plugin",
		$InferServerPlugin: {}
	};
};
function InferAuth() {
	return {};
}

//#endregion
export { InferPlugin as n, createAuthClient as r, InferAuth as t };