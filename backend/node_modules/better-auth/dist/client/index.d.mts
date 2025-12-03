import { $t as InferUserFromClient, Gt as ClientOptions, Ht as BetterAuthClientOptions, Jt as InferAdditionalFromClient, Kt as ClientStore, Qt as InferSessionFromClient, Ut as BetterAuthClientPlugin, Vt as AtomListener, Wt as ClientAtomListener, Xt as InferErrorCodes, Yt as InferClientAPI, Zt as InferPluginsFromClient, en as IsSignal, nn as Store, qt as InferActions, tn as SessionQueryParams } from "../index-COnelCGa.mjs";
import "../types-CEepZ-RG.mjs";
import { a as LiteralNumber, c as OmitId, d as PrettifyDeep, g as WithoutEmpty, h as UnionToIntersection, i as HasRequiredKeys, l as PreserveJSDoc, m as StripEmptyObjects, n as DeepPartial, o as LiteralString, p as RequiredKeysOf, r as Expand, s as LiteralUnion, t as Awaitable, u as Prettify } from "../helper-DFzV6jvx.mjs";
import "../plugins-Dmw3tk5H.mjs";
import { a as Role, c as Subset, i as AccessControl, n as createAccessControl, o as Statements, r as role, s as SubArray, t as AuthorizeResponse } from "../index-rnNTrfA7.mjs";
import { BetterAuthClientOptions as BetterAuthClientOptions$1, BetterAuthClientPlugin as BetterAuthClientPlugin$1, BetterAuthOptions, BetterAuthPlugin, ClientFetchOption } from "@better-auth/core";
import { Primitive } from "@better-auth/core/db";
import { BASE_ERROR_CODES } from "@better-auth/core/error";
import * as nanostores5 from "nanostores";
import { Atom, PreinitializedWritableAtom, WritableAtom } from "nanostores";
import * as _better_fetch_fetch95 from "@better-fetch/fetch";
import { BetterFetch, BetterFetchError } from "@better-fetch/fetch";
export * from "@better-auth/core/db";
export * from "nanostores";
export * from "@better-fetch/fetch";

//#region src/client/broadcast-channel.d.ts
interface BroadcastMessage {
  event?: "session" | undefined;
  data?: {
    trigger?: "signout" | "getSession" | "updateUser";
  } | undefined;
  clientId: string;
  timestamp: number;
}
type BroadcastListener = (message: BroadcastMessage) => void;
declare const kBroadcastChannel: unique symbol;
interface BroadcastChannel {
  post(message: Record<string, unknown>): void;
  subscribe(listener: BroadcastListener): () => void;
  setup(): () => void;
}
declare function getGlobalBroadcastChannel(name?: string): BroadcastChannel;
//#endregion
//#region src/client/focus-manager.d.ts
type FocusListener = (focused: boolean) => void;
declare const kFocusManager: unique symbol;
interface FocusManager {
  setFocused(focused: boolean): void;
  subscribe(listener: FocusListener): () => void;
  setup(): () => void;
}
//#endregion
//#region src/client/online-manager.d.ts
type OnlineListener = (online: boolean) => void;
declare const kOnlineManager: unique symbol;
interface OnlineManager {
  setOnline(online: boolean): void;
  isOnline: boolean;
  subscribe(listener: OnlineListener): () => void;
  setup(): () => void;
}
//#endregion
//#region src/client/query.d.ts
declare const useAuthQuery: <T>(initializedAtom: PreinitializedWritableAtom<any> | PreinitializedWritableAtom<any>[], path: string, $fetch: BetterFetch, options?: (((value: {
  data: null | T;
  error: null | BetterFetchError;
  isPending: boolean;
}) => ClientFetchOption) | ClientFetchOption) | undefined) => PreinitializedWritableAtom<{
  data: null | T;
  error: null | BetterFetchError;
  isPending: boolean;
  isRefetching: boolean;
  refetch: (queryParams?: {
    query?: SessionQueryParams;
  } | undefined) => Promise<void>;
}> & object;
//#endregion
//#region src/client/session-refresh.d.ts
interface SessionRefreshOptions {
  sessionAtom: WritableAtom<any>;
  sessionSignal: WritableAtom<boolean>;
  $fetch: BetterFetch;
  options?: BetterAuthClientOptions$1 | undefined;
}
declare function createSessionRefreshManager(opts: SessionRefreshOptions): {
  init: () => void;
  cleanup: () => void;
  triggerRefetch: (event?: {
    event?: "poll" | "visibilitychange" | "storage";
  } | undefined) => void;
  broadcastSessionUpdate: (trigger: "signout" | "getSession" | "updateUser") => void;
};
//#endregion
//#region src/client/vanilla.d.ts
type InferResolvedHooks<O extends BetterAuthClientOptions$1> = O extends {
  plugins: Array<infer Plugin>;
} ? UnionToIntersection<Plugin extends BetterAuthClientPlugin$1 ? Plugin["getAtoms"] extends ((fetch: any) => infer Atoms) ? Atoms extends Record<string, any> ? { [key in keyof Atoms as IsSignal<key> extends true ? never : key extends string ? `use${Capitalize<key>}` : never]: Atoms[key] } : {} : {} : {}> : {};
declare function createAuthClient<Option extends BetterAuthClientOptions$1>(options?: Option | undefined): UnionToIntersection<InferResolvedHooks<Option>> & InferClientAPI<Option> & InferActions<Option> & {
  useSession: Atom<{
    data: InferClientAPI<Option> extends {
      getSession: () => Promise<infer Res>;
    } ? Res extends {
      data: null;
      error: {
        message?: string | undefined;
        status: number;
        statusText: string;
      };
    } | {
      data: infer S;
      error: null;
    } ? S : Res extends Record<string, any> ? Res : never : never;
    error: BetterFetchError | null;
    isPending: boolean;
  }>;
  $fetch: _better_fetch_fetch95.BetterFetch<{
    plugins: (_better_fetch_fetch95.BetterFetchPlugin | {
      id: string;
      name: string;
      hooks: {
        onSuccess(context: _better_fetch_fetch95.SuccessContext<any>): void;
      };
    } | {
      id: string;
      name: string;
      hooks: {
        onSuccess: ((context: _better_fetch_fetch95.SuccessContext<any>) => Promise<void> | void) | undefined;
        onError: ((context: _better_fetch_fetch95.ErrorContext) => Promise<void> | void) | undefined;
        onRequest: (<T extends Record<string, any>>(context: _better_fetch_fetch95.RequestContext<T>) => Promise<_better_fetch_fetch95.RequestContext | void> | _better_fetch_fetch95.RequestContext | void) | undefined;
        onResponse: ((context: _better_fetch_fetch95.ResponseContext) => Promise<Response | void | _better_fetch_fetch95.ResponseContext> | Response | _better_fetch_fetch95.ResponseContext | void) | undefined;
      };
    })[];
    cache?: RequestCache | undefined;
    method: string;
    headers?: (HeadersInit & (HeadersInit | {
      accept: "application/json" | "text/plain" | "application/octet-stream";
      "content-type": "application/json" | "text/plain" | "application/x-www-form-urlencoded" | "multipart/form-data" | "application/octet-stream";
      authorization: "Bearer" | "Basic";
    })) | undefined;
    redirect?: RequestRedirect | undefined;
    credentials?: RequestCredentials;
    integrity?: string | undefined;
    keepalive?: boolean | undefined;
    mode?: RequestMode | undefined;
    priority?: RequestPriority | undefined;
    referrer?: string | undefined;
    referrerPolicy?: ReferrerPolicy | undefined;
    signal?: (AbortSignal | null) | undefined;
    window?: null | undefined;
    onRetry?: ((response: _better_fetch_fetch95.ResponseContext) => Promise<void> | void) | undefined;
    hookOptions?: {
      cloneResponse?: boolean;
    } | undefined;
    timeout?: number | undefined;
    customFetchImpl: _better_fetch_fetch95.FetchEsque;
    baseURL: string;
    throw?: boolean | undefined;
    auth?: ({
      type: "Bearer";
      token: string | Promise<string | undefined> | (() => string | Promise<string | undefined> | undefined) | undefined;
    } | {
      type: "Basic";
      username: string | (() => string | undefined) | undefined;
      password: string | (() => string | undefined) | undefined;
    } | {
      type: "Custom";
      prefix: string | (() => string | undefined) | undefined;
      value: string | (() => string | undefined) | undefined;
    }) | undefined;
    body?: any;
    query?: any;
    params?: any;
    duplex?: "full" | "half" | undefined;
    jsonParser: (text: string) => Promise<any> | any;
    retry?: _better_fetch_fetch95.RetryOptions | undefined;
    retryAttempt?: number | undefined;
    output?: (_better_fetch_fetch95.StandardSchemaV1 | typeof Blob | typeof File) | undefined;
    errorSchema?: _better_fetch_fetch95.StandardSchemaV1 | undefined;
    disableValidation?: boolean | undefined;
    disableSignal?: boolean | undefined;
  }, unknown, unknown, {}>;
  $store: {
    notify: (signal?: (Omit<string, "$sessionSignal"> | "$sessionSignal") | undefined) => void;
    listen: (signal: Omit<string, "$sessionSignal"> | "$sessionSignal", listener: (value: boolean, oldValue?: boolean | undefined) => void) => void;
    atoms: Record<string, nanostores5.WritableAtom<any>>;
  };
  $Infer: {
    Session: NonNullable<InferClientAPI<Option> extends {
      getSession: () => Promise<infer Res>;
    } ? Res extends {
      data: null;
      error: {
        message?: string | undefined;
        status: number;
        statusText: string;
      };
    } | {
      data: infer S;
      error: null;
    } ? S : Res extends Record<string, any> ? Res : never : never>;
  };
  $ERROR_CODES: PrettifyDeep<InferErrorCodes<Option> & typeof BASE_ERROR_CODES>;
};
type AuthClient<Option extends BetterAuthClientOptions$1> = ReturnType<typeof createAuthClient<Option>>;
//#endregion
//#region src/client/index.d.ts
declare const InferPlugin: <T extends BetterAuthPlugin>() => {
  id: "infer-server-plugin";
  $InferServerPlugin: T;
};
declare function InferAuth<O extends {
  options: BetterAuthOptions;
}>(): O["options"];
//#endregion
export { AccessControl, AtomListener, AuthClient, AuthorizeResponse, Awaitable, BetterAuthClientOptions, BetterAuthClientPlugin, BroadcastChannel, BroadcastListener, BroadcastMessage, ClientAtomListener, ClientOptions, ClientStore, DeepPartial, Expand, type FocusListener, type FocusManager, HasRequiredKeys, InferActions, InferAdditionalFromClient, InferAuth, InferClientAPI, InferErrorCodes, InferPlugin, InferPluginsFromClient, InferSessionFromClient, InferUserFromClient, IsSignal, LiteralNumber, LiteralString, LiteralUnion, OmitId, type OnlineListener, type OnlineManager, PreserveJSDoc, Prettify, PrettifyDeep, type Primitive, RequiredKeysOf, Role, SessionQueryParams, SessionRefreshOptions, Statements, Store, StripEmptyObjects, SubArray, Subset, type UnionToIntersection, WithoutEmpty, createAccessControl, createAuthClient, createSessionRefreshManager, getGlobalBroadcastChannel, kBroadcastChannel, kFocusManager, kOnlineManager, role, useAuthQuery };