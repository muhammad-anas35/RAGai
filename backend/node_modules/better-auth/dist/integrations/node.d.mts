import { o as Auth } from "../index-COnelCGa.mjs";
import "../types-CEepZ-RG.mjs";
import "../helper-DFzV6jvx.mjs";
import "../plugins-Dmw3tk5H.mjs";
import * as http0 from "http";
import { IncomingHttpHeaders } from "node:http";

//#region src/integrations/node.d.ts
declare const toNodeHandler: (auth: {
  handler: Auth["handler"];
} | Auth["handler"]) => (req: http0.IncomingMessage, res: http0.ServerResponse) => Promise<void>;
declare function fromNodeHeaders(nodeHeaders: IncomingHttpHeaders): Headers;
//#endregion
export { fromNodeHeaders, toNodeHandler };