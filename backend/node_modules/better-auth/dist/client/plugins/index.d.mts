import { r as BetterAuthOptions$1, tn as SessionQueryParams, wn as InferAdditionalFieldsFromPluginOptions } from "../../index-COnelCGa.mjs";
import "../../types-CEepZ-RG.mjs";
import { a as LiteralNumber, c as OmitId, d as PrettifyDeep, f as Primitive, g as WithoutEmpty, h as UnionToIntersection, i as HasRequiredKeys, l as PreserveJSDoc, m as StripEmptyObjects, n as DeepPartial, o as LiteralString, p as RequiredKeysOf, r as Expand, s as LiteralUnion, t as Awaitable, u as Prettify } from "../../helper-DFzV6jvx.mjs";
import "../../plugins-Dmw3tk5H.mjs";
import { a as Role, i as AccessControl, o as Statements } from "../../index-rnNTrfA7.mjs";
import { B as OrganizationOptions, C as Organization, D as Team, T as OrganizationRole, _ as InferTeam, _t as admin, at as OAuthAccessToken, bt as SessionWithImpersonatedBy, c as OrganizationPlugin, ct as TokenBody, f as InferInvitation, gt as ApiKeyOptions, ht as ApiKey, it as CodeVerificationValue, k as TeamMember, m as InferOrganization, nt as AuthorizationQuery, ot as OIDCMetadata, p as InferMember, pt as apiKey, rt as Client, st as OIDCOptions, tt as oidcProvider, v as Invitation, vt as AdminOptions, x as Member, xt as UserWithRole, yt as InferAdminRolesFromOption } from "../../index-F8qM450o.mjs";
import { i as schema, n as AnonymousOptions, r as UserWithAnonymous, t as anonymous } from "../../index-CMfXEnyA.mjs";
import "../../index-B64ouu9a.mjs";
import "../../index-CWcnHbjn.mjs";
import "../../index-DUa0239y.mjs";
import { n as deviceAuthorization } from "../../index-D2CJIZCI.mjs";
import { n as emailOTP } from "../../index-cgrDi8bR.mjs";
import { _ as Auth0Options, a as SlackOptions, c as okta, d as LineOptions, f as line, g as hubspot, h as HubSpotOptions, i as genericOAuth, l as MicrosoftEntraIdOptions, m as keycloak, n as GenericOAuthConfig, o as slack, p as KeycloakOptions, r as GenericOAuthOptions, s as OktaOptions, t as BaseOAuthProviderOptions, u as microsoftEntraId, v as auth0 } from "../../index-DLNLFTLS.mjs";
import "../../index-Bv2nWTrZ.mjs";
import { c as Jwk, l as JwtOptions, o as JWKOptions, s as JWSAlgorithms, t as jwt } from "../../index-0IRW2A1b.mjs";
import { n as magicLink } from "../../index-BMrJpdkv.mjs";
import { n as multiSession, t as MultiSessionConfig } from "../../index-t-jeKL1G.mjs";
import "../../index-DSwTw61P.mjs";
import "../../index-DoUxtZgO.mjs";
import { n as oneTimeToken, t as OneTimeTokenOptions } from "../../index-CmcoO98i.mjs";
import "../../index-BaZax_le.mjs";
import { n as PhoneNumberOptions, r as UserWithPhoneNumber, t as phoneNumber } from "../../index-alnYeOxT.mjs";
import { n as siwe } from "../../index-Dk2q2Dnr.mjs";
import { a as TwoFactorProvider, c as TOTPOptions, d as otp2fa, f as BackupCodeOptions, g as verifyBackupCode, h as getBackupCodes, i as TwoFactorOptions, l as totp2fa, m as generateBackupCodes, n as twoFactorClient, o as TwoFactorTable, p as backupCode2fa, s as UserWithTwoFactor, u as OTPOptions } from "../../index-Drqkvf7w.mjs";
import { n as username } from "../../index-CCnWzVSV.mjs";
import * as _better_auth_core28 from "@better-auth/core";
import { BetterAuthOptions, BetterAuthPlugin, ClientFetchOption } from "@better-auth/core";
import { DBFieldAttribute } from "@better-auth/core/db";
import { JSONWebKeySet } from "jose";
import * as nanostores6 from "nanostores";
import * as _better_fetch_fetch111 from "@better-fetch/fetch";
import { Auth } from "better-auth";

//#region src/plugins/additional-fields/client.d.ts
declare const inferAdditionalFields: <T$1, S$1 extends {
  user?: {
    [key: string]: DBFieldAttribute;
  } | undefined;
  session?: {
    [key: string]: DBFieldAttribute;
  } | undefined;
} = {}>(schema?: S$1 | undefined) => {
  id: "additional-fields-client";
  $InferServerPlugin: ((T$1 extends BetterAuthOptions ? T$1 : T$1 extends {
    options: BetterAuthOptions;
  } ? T$1["options"] : never) extends never ? S$1 extends {
    user?: {
      [key: string]: DBFieldAttribute;
    } | undefined;
    session?: {
      [key: string]: DBFieldAttribute;
    } | undefined;
  } ? {
    id: "additional-fields-client";
    schema: {
      user: {
        fields: S$1["user"] extends object ? S$1["user"] : {};
      };
      session: {
        fields: S$1["session"] extends object ? S$1["session"] : {};
      };
    };
  } : never : (T$1 extends BetterAuthOptions ? T$1 : T$1 extends {
    options: BetterAuthOptions;
  } ? T$1["options"] : never) extends BetterAuthOptions ? {
    id: "additional-fields";
    schema: {
      user: {
        fields: (T$1 extends BetterAuthOptions ? T$1 : T$1 extends {
          options: BetterAuthOptions;
        } ? T$1["options"] : never)["user"] extends {
          additionalFields: infer U;
        } ? U : {};
      };
      session: {
        fields: (T$1 extends BetterAuthOptions ? T$1 : T$1 extends {
          options: BetterAuthOptions;
        } ? T$1["options"] : never)["session"] extends {
          additionalFields: infer U;
        } ? U : {};
      };
    };
  } : never) extends BetterAuthPlugin ? (T$1 extends BetterAuthOptions ? T$1 : T$1 extends {
    options: BetterAuthOptions;
  } ? T$1["options"] : never) extends never ? S$1 extends {
    user?: {
      [key: string]: DBFieldAttribute;
    } | undefined;
    session?: {
      [key: string]: DBFieldAttribute;
    } | undefined;
  } ? {
    id: "additional-fields-client";
    schema: {
      user: {
        fields: S$1["user"] extends object ? S$1["user"] : {};
      };
      session: {
        fields: S$1["session"] extends object ? S$1["session"] : {};
      };
    };
  } : never : (T$1 extends BetterAuthOptions ? T$1 : T$1 extends {
    options: BetterAuthOptions;
  } ? T$1["options"] : never) extends BetterAuthOptions ? {
    id: "additional-fields";
    schema: {
      user: {
        fields: (T$1 extends BetterAuthOptions ? T$1 : T$1 extends {
          options: BetterAuthOptions;
        } ? T$1["options"] : never)["user"] extends {
          additionalFields: infer U;
        } ? U : {};
      };
      session: {
        fields: (T$1 extends BetterAuthOptions ? T$1 : T$1 extends {
          options: BetterAuthOptions;
        } ? T$1["options"] : never)["session"] extends {
          additionalFields: infer U;
        } ? U : {};
      };
    };
  } : never : undefined;
};
//#endregion
//#region src/plugins/admin/client.d.ts
interface AdminClientOptions {
  ac?: AccessControl | undefined;
  roles?: { [key in string]: Role } | undefined;
}
declare const adminClient: <O$1 extends AdminClientOptions>(options?: O$1 | undefined) => {
  id: "admin-client";
  $InferServerPlugin: ReturnType<typeof admin<{
    ac: O$1["ac"] extends AccessControl ? O$1["ac"] : AccessControl<{
      readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
      readonly session: readonly ["list", "revoke", "delete"];
    }>;
    roles: O$1["roles"] extends Record<string, Role> ? O$1["roles"] : {
      admin: Role;
      user: Role;
    };
  }>>;
  getActions: () => {
    admin: {
      checkRolePermission: <R extends (O$1 extends {
        roles: any;
      } ? keyof O$1["roles"] : "admin" | "user")>(data: ({
        /**
         * @deprecated Use `permissions` instead
         */
        permission: { [key in keyof (O$1["ac"] extends AccessControl<infer S extends Statements> ? S : {
          readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
          readonly session: readonly ["list", "revoke", "delete"];
        })]?: ((O$1["ac"] extends AccessControl<infer S extends Statements> ? S : {
          readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
          readonly session: readonly ["list", "revoke", "delete"];
        })[key] extends readonly unknown[] ? (O$1["ac"] extends AccessControl<infer S extends Statements> ? S : {
          readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
          readonly session: readonly ["list", "revoke", "delete"];
        })[key][number] : never)[] | undefined };
        permissions?: never | undefined;
      } | {
        permissions: { [key in keyof (O$1["ac"] extends AccessControl<infer S extends Statements> ? S : {
          readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
          readonly session: readonly ["list", "revoke", "delete"];
        })]?: ((O$1["ac"] extends AccessControl<infer S extends Statements> ? S : {
          readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
          readonly session: readonly ["list", "revoke", "delete"];
        })[key] extends readonly unknown[] ? (O$1["ac"] extends AccessControl<infer S extends Statements> ? S : {
          readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
          readonly session: readonly ["list", "revoke", "delete"];
        })[key][number] : never)[] | undefined };
        permission?: never | undefined;
      }) & {
        role: R;
      }) => boolean;
    };
  };
  pathMethods: {
    "/admin/list-users": "GET";
    "/admin/stop-impersonating": "POST";
  };
};
//#endregion
//#region src/plugins/anonymous/client.d.ts
declare const anonymousClient: () => {
  id: "anonymous";
  $InferServerPlugin: ReturnType<typeof anonymous>;
  pathMethods: {
    "/sign-in/anonymous": "POST";
  };
  atomListeners: {
    matcher: (path: string) => path is "/sign-in/anonymous";
    signal: "$sessionSignal";
  }[];
};
//#endregion
//#region src/plugins/api-key/client.d.ts
declare const apiKeyClient: () => {
  id: "api-key";
  $InferServerPlugin: ReturnType<typeof apiKey>;
  pathMethods: {
    "/api-key/create": "POST";
    "/api-key/delete": "POST";
    "/api-key/delete-all-expired-api-keys": "POST";
  };
};
type ApiKeyClientPlugin = ReturnType<typeof apiKeyClient>;
//#endregion
//#region src/plugins/custom-session/client.d.ts
declare const customSessionClient: <A extends Auth | {
  options: BetterAuthOptions$1;
}>() => {
  id: "infer-server-plugin";
  $InferServerPlugin: (A extends {
    options: infer O;
  } ? O : A)["plugins"] extends (infer P)[] ? P extends {
    id: "custom-session";
  } ? P : never : never;
};
//#endregion
//#region src/plugins/device-authorization/client.d.ts
declare const deviceAuthorizationClient: () => {
  id: "device-authorization";
  $InferServerPlugin: ReturnType<typeof deviceAuthorization>;
  pathMethods: {
    "/device/code": "POST";
    "/device/token": "POST";
    "/device": "GET";
    "/device/approve": "POST";
    "/device/deny": "POST";
  };
};
//#endregion
//#region src/plugins/email-otp/client.d.ts
declare const emailOTPClient: () => {
  id: "email-otp";
  $InferServerPlugin: ReturnType<typeof emailOTP>;
  atomListeners: {
    matcher: (path: string) => path is "/email-otp/verify-email" | "/sign-in/email-otp";
    signal: "$sessionSignal";
  }[];
};
//#endregion
//#region src/plugins/generic-oauth/client.d.ts
declare const genericOAuthClient: () => {
  id: "generic-oauth-client";
  $InferServerPlugin: ReturnType<typeof genericOAuth>;
};
//#endregion
//#region src/plugins/jwt/client.d.ts
interface JwtClientOptions {
  jwks?: {
    /**
     * The path of the endpoint exposing the JWKS.
     * Must match the server configuration.
     *
     * @default /jwks
     */
    jwksPath?: string;
  };
}
declare const jwtClient: (options?: JwtClientOptions) => {
  id: "better-auth-client";
  $InferServerPlugin: ReturnType<typeof jwt>;
  pathMethods: {
    [x: string]: "GET";
  };
  getActions: ($fetch: _better_fetch_fetch111.BetterFetch) => {
    jwks: (fetchOptions?: any) => Promise<{
      data: null;
      error: {
        message?: string | undefined;
        status: number;
        statusText: string;
      };
    } | {
      data: JSONWebKeySet;
      error: null;
    }>;
  };
};
//#endregion
//#region src/plugins/last-login-method/client.d.ts
/**
 * Configuration for the client-side last login method plugin
 */
interface LastLoginMethodClientConfig {
  /**
   * Name of the cookie to read the last login method from
   * @default "better-auth.last_used_login_method"
   */
  cookieName?: string | undefined;
}
/**
 * Client-side plugin to retrieve the last used login method
 */
declare const lastLoginMethodClient: (config?: LastLoginMethodClientConfig) => {
  id: "last-login-method-client";
  getActions(): {
    /**
     * Get the last used login method from cookies
     * @returns The last used login method or null if not found
     */
    getLastUsedLoginMethod: () => string | null;
    /**
     * Clear the last used login method cookie
     * This sets the cookie with an expiration date in the past
     */
    clearLastUsedLoginMethod: () => void;
    /**
     * Check if a specific login method was the last used
     * @param method The method to check
     * @returns True if the method was the last used, false otherwise
     */
    isLastUsedLoginMethod: (method: string) => boolean;
  };
};
//#endregion
//#region src/plugins/magic-link/client.d.ts
declare const magicLinkClient: () => {
  id: "magic-link";
  $InferServerPlugin: ReturnType<typeof magicLink>;
};
//#endregion
//#region src/plugins/multi-session/client.d.ts
declare const multiSessionClient: () => {
  id: "multi-session";
  $InferServerPlugin: ReturnType<typeof multiSession>;
  atomListeners: {
    matcher(path: string): path is "/multi-session/set-active";
    signal: "$sessionSignal";
  }[];
};
//#endregion
//#region src/plugins/oidc-provider/client.d.ts
declare const oidcClient: () => {
  id: "oidc-client";
  $InferServerPlugin: ReturnType<typeof oidcProvider>;
};
type OidcClientPlugin = ReturnType<typeof oidcClient>;
//#endregion
//#region src/plugins/one-tap/client.d.ts
declare global {
  interface Window {
    google?: {
      accounts: {
        id: {
          initialize: (config: any) => void;
          prompt: (callback?: (notification: any) => void) => void;
        };
      };
    } | undefined;
    googleScriptInitialized?: boolean | undefined;
  }
}
interface GoogleOneTapOptions {
  /**
   * Google client ID
   */
  clientId: string;
  /**
   * Auto select the account if the user is already signed in
   */
  autoSelect?: boolean | undefined;
  /**
   * Cancel the flow when the user taps outside the prompt
   */
  cancelOnTapOutside?: boolean | undefined;
  /**
   * The mode to use for the Google One Tap flow
   *
   * popup: Use a popup window
   * redirect: Redirect the user to the Google One Tap flow
   *
   * @default "popup"
   */
  uxMode?: ("popup" | "redirect") | undefined;
  /**
   * The context to use for the Google One Tap flow.
   *
   * @see {@link https://developers.google.com/identity/gsi/web/reference/js-reference}
   * @default "signin"
   */
  context?: ("signin" | "signup" | "use") | undefined;
  /**
   * Additional configuration options to pass to the Google One Tap API.
   */
  additionalOptions?: Record<string, any> | undefined;
  /**
   * Configuration options for the prompt and exponential backoff behavior.
   */
  promptOptions?: {
    /**
     * Base delay (in milliseconds) for exponential backoff.
     * @default 1000
     */
    baseDelay?: number;
    /**
     * Maximum number of prompt attempts before calling onPromptNotification.
     * @default 5
     */
    maxAttempts?: number;
    /**
     * Whether to support FedCM (Federated Credential Management) support.
     *
     * @see {@link https://developer.chrome.com/docs/identity/fedcm/overview}
     * @default true
     */
    fedCM?: boolean | undefined;
  } | undefined;
}
interface GoogleOneTapActionOptions extends Omit<GoogleOneTapOptions, "clientId" | "promptOptions"> {
  fetchOptions?: ClientFetchOption | undefined;
  /**
   * Callback URL.
   */
  callbackURL?: string | undefined;
  /**
   * Optional callback that receives the prompt notification if (or when) the prompt is dismissed or skipped.
   * This lets you render an alternative UI (e.g. a Google Sign-In button) to restart the process.
   */
  onPromptNotification?: ((notification?: any | undefined) => void) | undefined;
  nonce?: string | undefined;
}
declare const oneTapClient: (options: GoogleOneTapOptions) => {
  id: "one-tap";
  fetchPlugins: {
    id: string;
    name: string;
    hooks: {
      onResponse(ctx: _better_fetch_fetch111.ResponseContext): Promise<void>;
    };
  }[];
  getActions: ($fetch: _better_fetch_fetch111.BetterFetch, _: _better_auth_core28.ClientStore) => {
    oneTap: (opts?: GoogleOneTapActionOptions | undefined, fetchOptions?: ClientFetchOption | undefined) => Promise<void>;
  };
  getAtoms($fetch: _better_fetch_fetch111.BetterFetch): {};
};
//#endregion
//#region src/plugins/one-time-token/client.d.ts
declare const oneTimeTokenClient: () => {
  id: "one-time-token";
  $InferServerPlugin: ReturnType<typeof oneTimeToken>;
};
//#endregion
//#region src/plugins/organization/permission.d.ts
type PermissionExclusive = {
  /**
   * @deprecated Use `permissions` instead
   */
  permission: {
    [key: string]: string[];
  };
  permissions?: never | undefined;
} | {
  permissions: {
    [key: string]: string[];
  };
  permission?: never | undefined;
};
type HasPermissionBaseInput = {
  role: string;
  options: OrganizationOptions;
  allowCreatorAllPermissions?: boolean | undefined;
} & PermissionExclusive;
//#endregion
//#region src/plugins/organization/client.d.ts
/**
 * Using the same `hasPermissionFn` function, but without the need for a `ctx` parameter or the `organizationId` parameter.
 */
declare const clientSideHasPermission: (input: HasPermissionBaseInput) => boolean;
interface OrganizationClientOptions {
  ac?: AccessControl | undefined;
  roles?: { [key in string]: Role } | undefined;
  teams?: {
    enabled: boolean;
  } | undefined;
  schema?: {
    organization?: {
      additionalFields?: {
        [key: string]: DBFieldAttribute;
      };
    };
    member?: {
      additionalFields?: {
        [key: string]: DBFieldAttribute;
      };
    };
    invitation?: {
      additionalFields?: {
        [key: string]: DBFieldAttribute;
      };
    };
    team?: {
      additionalFields?: {
        [key: string]: DBFieldAttribute;
      };
    };
    organizationRole?: {
      additionalFields?: {
        [key: string]: DBFieldAttribute;
      };
    };
  } | undefined;
  dynamicAccessControl?: {
    enabled: boolean;
  } | undefined;
}
declare const organizationClient: <CO extends OrganizationClientOptions>(options?: CO | undefined) => {
  id: "organization";
  $InferServerPlugin: OrganizationPlugin<{
    ac: CO["ac"] extends AccessControl ? CO["ac"] : AccessControl<{
      readonly organization: readonly ["update", "delete"];
      readonly member: readonly ["create", "update", "delete"];
      readonly invitation: readonly ["create", "cancel"];
      readonly team: readonly ["create", "update", "delete"];
      readonly ac: readonly ["create", "read", "update", "delete"];
    }>;
    roles: CO["roles"] extends Record<string, Role> ? CO["roles"] : {
      admin: Role;
      member: Role;
      owner: Role;
    };
    teams: {
      enabled: CO["teams"] extends {
        enabled: true;
      } ? true : false;
    };
    schema: CO["schema"];
    dynamicAccessControl: {
      enabled: CO["dynamicAccessControl"] extends {
        enabled: true;
      } ? true : false;
    };
  }>;
  getActions: ($fetch: _better_fetch_fetch111.BetterFetch, _$store: _better_auth_core28.ClientStore, co: _better_auth_core28.BetterAuthClientOptions | undefined) => {
    $Infer: {
      ActiveOrganization: CO["teams"] extends {
        enabled: true;
      } ? {
        members: InferMember<CO, false>[];
        invitations: InferInvitation<CO>[];
        teams: InferTeam<CO, false>[];
      } & ({
        id: string;
        name: string;
        slug: string;
        createdAt: Date;
        logo?: string | null | undefined;
        metadata?: any;
      } & InferAdditionalFieldsFromPluginOptions<"organization", CO, false> extends infer T ? { [K in keyof T]: T[K] } : never) : {
        members: InferMember<CO, false>[];
        invitations: InferInvitation<CO, false>[];
      } & ({
        id: string;
        name: string;
        slug: string;
        createdAt: Date;
        logo?: string | null | undefined;
        metadata?: any;
      } & InferAdditionalFieldsFromPluginOptions<"organization", CO, false> extends infer T_1 ? { [K in keyof T_1]: T_1[K] } : never);
      Organization: InferOrganization<CO, false>;
      Invitation: InferInvitation<CO, false>;
      Member: InferMember<CO, false>;
      Team: InferTeam<CO, false>;
    };
    organization: {
      checkRolePermission: <R extends (CO extends {
        roles: any;
      } ? keyof CO["roles"] : "admin" | "member" | "owner")>(data: ({
        /**
         * @deprecated Use `permissions` instead
         */
        permission: { [key in keyof (CO["ac"] extends AccessControl<infer S extends Statements> ? S : {
          readonly organization: readonly ["update", "delete"];
          readonly member: readonly ["create", "update", "delete"];
          readonly invitation: readonly ["create", "cancel"];
          readonly team: readonly ["create", "update", "delete"];
          readonly ac: readonly ["create", "read", "update", "delete"];
        })]?: ((CO["ac"] extends AccessControl<infer S extends Statements> ? S : {
          readonly organization: readonly ["update", "delete"];
          readonly member: readonly ["create", "update", "delete"];
          readonly invitation: readonly ["create", "cancel"];
          readonly team: readonly ["create", "update", "delete"];
          readonly ac: readonly ["create", "read", "update", "delete"];
        })[key] extends readonly unknown[] ? (CO["ac"] extends AccessControl<infer S extends Statements> ? S : {
          readonly organization: readonly ["update", "delete"];
          readonly member: readonly ["create", "update", "delete"];
          readonly invitation: readonly ["create", "cancel"];
          readonly team: readonly ["create", "update", "delete"];
          readonly ac: readonly ["create", "read", "update", "delete"];
        })[key][number] : never)[] | undefined };
        permissions?: never | undefined;
      } | {
        permissions: { [key in keyof (CO["ac"] extends AccessControl<infer S extends Statements> ? S : {
          readonly organization: readonly ["update", "delete"];
          readonly member: readonly ["create", "update", "delete"];
          readonly invitation: readonly ["create", "cancel"];
          readonly team: readonly ["create", "update", "delete"];
          readonly ac: readonly ["create", "read", "update", "delete"];
        })]?: ((CO["ac"] extends AccessControl<infer S extends Statements> ? S : {
          readonly organization: readonly ["update", "delete"];
          readonly member: readonly ["create", "update", "delete"];
          readonly invitation: readonly ["create", "cancel"];
          readonly team: readonly ["create", "update", "delete"];
          readonly ac: readonly ["create", "read", "update", "delete"];
        })[key] extends readonly unknown[] ? (CO["ac"] extends AccessControl<infer S extends Statements> ? S : {
          readonly organization: readonly ["update", "delete"];
          readonly member: readonly ["create", "update", "delete"];
          readonly invitation: readonly ["create", "cancel"];
          readonly team: readonly ["create", "update", "delete"];
          readonly ac: readonly ["create", "read", "update", "delete"];
        })[key][number] : never)[] | undefined };
        permission?: never | undefined;
      }) & {
        role: R;
      }) => boolean;
    };
  };
  getAtoms: ($fetch: _better_fetch_fetch111.BetterFetch) => {
    $listOrg: nanostores6.PreinitializedWritableAtom<boolean> & object;
    $activeOrgSignal: nanostores6.PreinitializedWritableAtom<boolean> & object;
    $activeMemberSignal: nanostores6.PreinitializedWritableAtom<boolean> & object;
    $activeMemberRoleSignal: nanostores6.PreinitializedWritableAtom<boolean> & object;
    activeOrganization: nanostores6.PreinitializedWritableAtom<{
      data: Prettify<({
        id: string;
        name: string;
        slug: string;
        createdAt: Date;
        logo?: string | null | undefined;
        metadata?: any;
      } & InferAdditionalFieldsFromPluginOptions<"organization", CO, false> extends infer T ? { [K in keyof T]: T[K] } : never) & {
        members: InferMember<CO, false>[];
        invitations: InferInvitation<CO, false>[];
      }> | null;
      error: null | _better_fetch_fetch111.BetterFetchError;
      isPending: boolean;
      isRefetching: boolean;
      refetch: (queryParams?: {
        query?: SessionQueryParams;
      } | undefined) => Promise<void>;
    }> & object;
    listOrganizations: nanostores6.PreinitializedWritableAtom<{
      data: ({
        id: string;
        name: string;
        slug: string;
        createdAt: Date;
        logo?: string | null | undefined;
        metadata?: any;
      } & InferAdditionalFieldsFromPluginOptions<"organization", CO, false> extends infer T_1 ? { [K in keyof T_1]: T_1[K] } : never)[] | null;
      error: null | _better_fetch_fetch111.BetterFetchError;
      isPending: boolean;
      isRefetching: boolean;
      refetch: (queryParams?: {
        query?: SessionQueryParams;
      } | undefined) => Promise<void>;
    }> & object;
    activeMember: nanostores6.PreinitializedWritableAtom<{
      data: {
        id: string;
        organizationId: string;
        userId: string;
        role: string;
        createdAt: Date;
      } | null;
      error: null | _better_fetch_fetch111.BetterFetchError;
      isPending: boolean;
      isRefetching: boolean;
      refetch: (queryParams?: {
        query?: SessionQueryParams;
      } | undefined) => Promise<void>;
    }> & object;
    activeMemberRole: nanostores6.PreinitializedWritableAtom<{
      data: {
        role: string;
      } | null;
      error: null | _better_fetch_fetch111.BetterFetchError;
      isPending: boolean;
      isRefetching: boolean;
      refetch: (queryParams?: {
        query?: SessionQueryParams;
      } | undefined) => Promise<void>;
    }> & object;
  };
  pathMethods: {
    "/organization/get-full-organization": "GET";
    "/organization/list-user-teams": "GET";
  };
  atomListeners: ({
    matcher(path: string): path is "/organization/create" | "/organization/update" | "/organization/delete";
    signal: "$listOrg";
  } | {
    matcher(path: string): boolean;
    signal: "$activeOrgSignal";
  } | {
    matcher(path: string): boolean;
    signal: "$sessionSignal";
  } | {
    matcher(path: string): boolean;
    signal: "$activeMemberSignal";
  } | {
    matcher(path: string): boolean;
    signal: "$activeMemberRoleSignal";
  })[];
};
declare const inferOrgAdditionalFields: <O$1 extends {
  options: BetterAuthOptions$1;
}, S$1 extends OrganizationOptions["schema"] = undefined>(schema?: S$1 | undefined) => undefined extends S$1 ? O$1 extends Object ? O$1 extends {
  session?: {
    fields?: {
      activeOrganizationId?: string;
      activeTeamId?: string;
    };
  };
  organization?: {
    modelName?: string;
    fields?: { [key in keyof Omit<Organization, "id">]?: string };
    additionalFields?: { [key in string]: DBFieldAttribute };
  };
  member?: {
    modelName?: string;
    fields?: { [key in keyof Omit<Member, "id">]?: string };
    additionalFields?: { [key in string]: DBFieldAttribute };
  };
  invitation?: {
    modelName?: string;
    fields?: { [key in keyof Omit<Invitation, "id">]?: string };
    additionalFields?: { [key in string]: DBFieldAttribute };
  };
  team?: {
    modelName?: string;
    fields?: { [key in keyof Omit<Team, "id">]?: string };
    additionalFields?: { [key in string]: DBFieldAttribute };
  };
  teamMember?: {
    modelName?: string;
    fields?: { [key in keyof Omit<TeamMember, "id">]?: string };
  };
  organizationRole?: {
    modelName?: string;
    fields?: { [key in keyof Omit<OrganizationRole, "id">]?: string };
    additionalFields?: { [key in string]: DBFieldAttribute };
  };
} ? O$1 : ((O$1 extends {
  options: any;
} ? O$1 : {
  options: {
    plugins: [];
  };
})["options"]["plugins"][number] extends infer T ? T extends (O$1 extends {
  options: any;
} ? O$1 : {
  options: {
    plugins: [];
  };
})["options"]["plugins"][number] ? T extends {
  id: "organization";
} ? T : never : never : never) extends {
  options: {
    schema: infer S_1;
  };
} ? S_1 extends {
  session?: {
    fields?: {
      activeOrganizationId?: string;
      activeTeamId?: string;
    };
  };
  organization?: {
    modelName?: string;
    fields?: { [key in keyof Omit<Organization, "id">]?: string };
    additionalFields?: { [key in string]: DBFieldAttribute };
  };
  member?: {
    modelName?: string;
    fields?: { [key in keyof Omit<Member, "id">]?: string };
    additionalFields?: { [key in string]: DBFieldAttribute };
  };
  invitation?: {
    modelName?: string;
    fields?: { [key in keyof Omit<Invitation, "id">]?: string };
    additionalFields?: { [key in string]: DBFieldAttribute };
  };
  team?: {
    modelName?: string;
    fields?: { [key in keyof Omit<Team, "id">]?: string };
    additionalFields?: { [key in string]: DBFieldAttribute };
  };
  teamMember?: {
    modelName?: string;
    fields?: { [key in keyof Omit<TeamMember, "id">]?: string };
  };
  organizationRole?: {
    modelName?: string;
    fields?: { [key in keyof Omit<OrganizationRole, "id">]?: string };
    additionalFields?: { [key in string]: DBFieldAttribute };
  };
} | undefined ? { [K in keyof S_1]: S_1[K] extends {
  additionalFields: infer AF;
} ? S_1[K] : undefined } : undefined : undefined : undefined : S$1;
//#endregion
//#region src/plugins/phone-number/client.d.ts
declare const phoneNumberClient: () => {
  id: "phoneNumber";
  $InferServerPlugin: ReturnType<typeof phoneNumber>;
  atomListeners: {
    matcher(path: string): path is "/phone-number/verify" | "/sign-in/phone-number" | "/phone-number/update";
    signal: "$sessionSignal";
  }[];
};
//#endregion
//#region src/plugins/siwe/client.d.ts
declare const siweClient: () => {
  id: "siwe";
  $InferServerPlugin: ReturnType<typeof siwe>;
};
//#endregion
//#region src/plugins/username/client.d.ts
declare const usernameClient: () => {
  id: "username";
  $InferServerPlugin: ReturnType<typeof username>;
  atomListeners: {
    matcher: (path: string) => path is "/sign-in/username";
    signal: "$sessionSignal";
  }[];
};
//#endregion
//#region src/client/plugins/infer-plugin.d.ts
declare const InferServerPlugin: <AuthOrOption extends BetterAuthOptions | {
  options: BetterAuthOptions;
}, ID extends string>() => {
  id: "infer-server-plugin";
  $InferServerPlugin: (AuthOrOption extends {
    options: infer O;
  } ? O : AuthOrOption)["plugins"] extends (infer P)[] ? P extends {
    id: ID;
  } ? P : never : never;
};
//#endregion
export { AdminOptions, AnonymousOptions, ApiKey, ApiKeyClientPlugin, ApiKeyOptions, Auth0Options, AuthorizationQuery, Awaitable, BackupCodeOptions, type BaseOAuthProviderOptions, Client, CodeVerificationValue, DeepPartial, Expand, type GenericOAuthConfig, type GenericOAuthOptions, GoogleOneTapActionOptions, GoogleOneTapOptions, HasRequiredKeys, HubSpotOptions, InferAdminRolesFromOption, InferServerPlugin, JWKOptions, JWSAlgorithms, Jwk, JwtOptions, KeycloakOptions, LastLoginMethodClientConfig, LineOptions, LiteralNumber, LiteralString, LiteralUnion, MicrosoftEntraIdOptions, type MultiSessionConfig, OAuthAccessToken, OIDCMetadata, OIDCOptions, OTPOptions, OidcClientPlugin, OktaOptions, OmitId, type OneTimeTokenOptions, PhoneNumberOptions, PreserveJSDoc, Prettify, PrettifyDeep, Primitive, RequiredKeysOf, SessionWithImpersonatedBy, SlackOptions, StripEmptyObjects, TOTPOptions, TokenBody, TwoFactorOptions, TwoFactorProvider, TwoFactorTable, UnionToIntersection, UserWithAnonymous, UserWithPhoneNumber, UserWithRole, UserWithTwoFactor, WithoutEmpty, adminClient, anonymousClient, apiKeyClient, auth0, backupCode2fa, clientSideHasPermission, customSessionClient, deviceAuthorizationClient, emailOTPClient, generateBackupCodes, genericOAuthClient, getBackupCodes, hubspot, inferAdditionalFields, inferOrgAdditionalFields, jwtClient, keycloak, lastLoginMethodClient, line, magicLinkClient, microsoftEntraId, multiSessionClient, oidcClient, okta, oneTapClient, oneTimeTokenClient, organizationClient, otp2fa, phoneNumberClient, schema, siweClient, slack, totp2fa, twoFactorClient, usernameClient, verifyBackupCode };