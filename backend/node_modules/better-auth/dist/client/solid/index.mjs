import { t as capitalizeFirstLetter } from "../../misc-BwNc0MKr.mjs";
import "../../url-CB8xCwz-.mjs";
import { n as getClientConfig, t as createDynamicPathProxy } from "../../proxy-DplNCOES.mjs";
import "../../parser-pHp5yoAv.mjs";
import { onCleanup } from "solid-js";
import { createStore, reconcile } from "solid-js/store";

//#region src/client/solid/solid-store.ts
/**
* Subscribes to store changes and gets store’s value.
*
* @param store Store instance.
* @returns Store value.
*/
function useStore(store) {
	const unbindActivation = store.listen(() => {});
	const [state, setState] = createStore({ value: store.get() });
	const unsubscribe = store.subscribe((newValue) => {
		setState("value", reconcile(newValue));
	});
	onCleanup(() => unsubscribe());
	unbindActivation();
	return () => state.value;
}

//#endregion
//#region src/client/solid/index.ts
function getAtomKey(str) {
	return `use${capitalizeFirstLetter(str)}`;
}
function createAuthClient(options) {
	const { pluginPathMethods, pluginsActions, pluginsAtoms, $fetch, atomListeners } = getClientConfig(options);
	let resolvedHooks = {};
	for (const [key, value] of Object.entries(pluginsAtoms)) resolvedHooks[getAtomKey(key)] = () => useStore(value);
	return createDynamicPathProxy({
		...pluginsActions,
		...resolvedHooks
	}, $fetch, pluginPathMethods, pluginsAtoms, atomListeners);
}

//#endregion
export { createAuthClient };