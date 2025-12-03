import { t as capitalizeFirstLetter } from "../../misc-BwNc0MKr.mjs";
import "../../url-CB8xCwz-.mjs";
import { n as getClientConfig, t as createDynamicPathProxy } from "../../proxy-DplNCOES.mjs";
import "../../parser-pHp5yoAv.mjs";

//#region src/client/svelte/index.ts
function createAuthClient(options) {
	const { pluginPathMethods, pluginsActions, pluginsAtoms, $fetch, atomListeners, $store } = getClientConfig(options);
	let resolvedHooks = {};
	for (const [key, value] of Object.entries(pluginsAtoms)) resolvedHooks[`use${capitalizeFirstLetter(key)}`] = () => value;
	return createDynamicPathProxy({
		...pluginsActions,
		...resolvedHooks,
		$fetch,
		$store
	}, $fetch, pluginPathMethods, pluginsAtoms, atomListeners);
}

//#endregion
export { createAuthClient };