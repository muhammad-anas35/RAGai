const require_node = require('./node.cjs');
let __better_fetch_fetch = require("@better-fetch/fetch");

//#region src/client.ts
const createClient = (options) => {
	const fetch = (0, __better_fetch_fetch.createFetch)(options);
	return async (path, ...options$1) => {
		return await fetch(path, { ...options$1[0] });
	};
};

//#endregion
exports.createClient = createClient;
//# sourceMappingURL=client.cjs.map