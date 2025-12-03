import * as _better_auth_core16 from "@better-auth/core";
import * as better_call183 from "better-call";

//#region src/integrations/tanstack-start.d.ts
declare const tanstackStartCookies: () => {
  id: "tanstack-start-cookies";
  hooks: {
    after: {
      matcher(ctx: _better_auth_core16.HookEndpointContext): true;
      handler: (inputContext: better_call183.MiddlewareInputContext<better_call183.MiddlewareOptions>) => Promise<void>;
    }[];
  };
};
//#endregion
export { tanstackStartCookies };