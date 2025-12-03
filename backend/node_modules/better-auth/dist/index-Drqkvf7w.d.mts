import { _ as User } from "./index-COnelCGa.mjs";
import { o as LiteralString } from "./helper-DFzV6jvx.mjs";
import { t as InferOptionSchema } from "./plugins-Dmw3tk5H.mjs";
import * as _better_auth_core20 from "@better-auth/core";
import { BetterAuthPlugin, GenericEndpointContext } from "@better-auth/core";
import * as better_call688 from "better-call";
import * as z from "zod";
import * as _better_fetch_fetch94 from "@better-fetch/fetch";

//#region src/plugins/two-factor/backup-codes/index.d.ts
interface BackupCodeOptions {
  /**
   * The amount of backup codes to generate
   *
   * @default 10
   */
  amount?: number | undefined;
  /**
   * The length of the backup codes
   *
   * @default 10
   */
  length?: number | undefined;
  /**
   * An optional custom function to generate backup codes
   */
  customBackupCodesGenerate?: (() => string[]) | undefined;
  /**
   * How to store the backup codes in the database, whether encrypted or plain.
   */
  storeBackupCodes?: ("plain" | "encrypted" | {
    encrypt: (token: string) => Promise<string>;
    decrypt: (token: string) => Promise<string>;
  }) | undefined;
}
declare function generateBackupCodes(secret: string, options?: BackupCodeOptions | undefined): Promise<{
  backupCodes: string[];
  encryptedBackupCodes: string;
}>;
declare function verifyBackupCode(data: {
  backupCodes: string;
  code: string;
}, key: string, options?: BackupCodeOptions | undefined): Promise<{
  status: boolean;
  updated: null;
} | {
  status: boolean;
  updated: string[];
}>;
declare function getBackupCodes(backupCodes: string, key: string, options?: BackupCodeOptions | undefined): Promise<string[] | null>;
declare const backupCode2fa: (opts: BackupCodeOptions) => {
  id: "backup_code";
  endpoints: {
    /**
     * ### Endpoint
     *
     * POST `/two-factor/verify-backup-code`
     *
     * ### API Methods
     *
     * **server:**
     * `auth.api.verifyBackupCode`
     *
     * **client:**
     * `authClient.twoFactor.verifyBackupCode`
     *
     * @see [Read our docs to learn more.](https://better-auth.com/docs/plugins/2fa#api-method-two-factor-verify-backup-code)
     */
    verifyBackupCode: better_call688.StrictEndpoint<"/two-factor/verify-backup-code", {
      method: "POST";
      body: z.ZodObject<{
        code: z.ZodString;
        disableSession: z.ZodOptional<z.ZodBoolean>;
        trustDevice: z.ZodOptional<z.ZodBoolean>;
      }, z.core.$strip>;
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      user: {
                        type: string;
                        properties: {
                          id: {
                            type: string;
                            description: string;
                          };
                          email: {
                            type: string;
                            format: string;
                            nullable: boolean;
                            description: string;
                          };
                          emailVerified: {
                            type: string;
                            nullable: boolean;
                            description: string;
                          };
                          name: {
                            type: string;
                            nullable: boolean;
                            description: string;
                          };
                          image: {
                            type: string;
                            format: string;
                            nullable: boolean;
                            description: string;
                          };
                          twoFactorEnabled: {
                            type: string;
                            description: string;
                          };
                          createdAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                          updatedAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                        };
                        required: string[];
                        description: string;
                      };
                      session: {
                        type: string;
                        properties: {
                          token: {
                            type: string;
                            description: string;
                          };
                          userId: {
                            type: string;
                            description: string;
                          };
                          createdAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                          expiresAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                        };
                        required: string[];
                        description: string;
                      };
                    };
                    required: string[];
                  };
                };
              };
            };
          };
        };
      };
    } & {
      use: any[];
    }, {
      token: string | undefined;
      user: {
        id: string;
        email: string;
        emailVerified: boolean;
        name: string;
        image: string | null | undefined;
        createdAt: Date;
        updatedAt: Date;
      };
    }>;
    /**
     * ### Endpoint
     *
     * POST `/two-factor/generate-backup-codes`
     *
     * ### API Methods
     *
     * **server:**
     * `auth.api.generateBackupCodes`
     *
     * **client:**
     * `authClient.twoFactor.generateBackupCodes`
     *
     * @see [Read our docs to learn more.](https://better-auth.com/docs/plugins/2fa#api-method-two-factor-generate-backup-codes)
     */
    generateBackupCodes: better_call688.StrictEndpoint<"/two-factor/generate-backup-codes", {
      method: "POST";
      body: z.ZodObject<{
        password: z.ZodString;
      }, z.core.$strip>;
      use: ((inputContext: better_call688.MiddlewareInputContext<better_call688.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      status: {
                        type: string;
                        description: string;
                        enum: boolean[];
                      };
                      backupCodes: {
                        type: string;
                        items: {
                          type: string;
                        };
                        description: string;
                      };
                    };
                    required: string[];
                  };
                };
              };
            };
          };
        };
      };
    } & {
      use: any[];
    }, {
      status: boolean;
      backupCodes: string[];
    }>;
    /**
     * ### Endpoint
     *
     * POST `/two-factor/view-backup-codes`
     *
     * ### API Methods
     *
     * **server:**
     * `auth.api.viewBackupCodes`
     *
     * @see [Read our docs to learn more.](https://better-auth.com/docs/plugins/2fa#api-method-two-factor-view-backup-codes)
     */
    viewBackupCodes: better_call688.StrictEndpoint<"/two-factor/view-backup-codes", {
      method: "POST";
      body: z.ZodObject<{
        userId: z.ZodCoercedString<unknown>;
      }, z.core.$strip>;
      metadata: {
        SERVER_ONLY: true;
      };
    } & {
      use: any[];
    }, {
      status: boolean;
      backupCodes: string[];
    }>;
  };
};
//#endregion
//#region src/plugins/two-factor/otp/index.d.ts
interface OTPOptions {
  /**
   * How long the opt will be valid for in
   * minutes
   *
   * @default "3 mins"
   */
  period?: number | undefined;
  /**
   * Number of digits for the OTP code
   *
   * @default 6
   */
  digits?: number | undefined;
  /**
   * Send the otp to the user
   *
   * @param user - The user to send the otp to
   * @param otp - The otp to send
   * @param request - The request object
   * @returns void | Promise<void>
   */
  sendOTP?: ((
  /**
   * The user to send the otp to
   * @type UserWithTwoFactor
   * @default UserWithTwoFactors
   */
  data: {
    user: UserWithTwoFactor;
    otp: string;
  },
  /**
   * The request object
   */
  ctx?: GenericEndpointContext) => Promise<void> | void) | undefined;
  /**
   * The number of allowed attempts for the OTP
   *
   * @default 5
   */
  allowedAttempts?: number | undefined;
  storeOTP?: ("plain" | "encrypted" | "hashed" | {
    hash: (token: string) => Promise<string>;
  } | {
    encrypt: (token: string) => Promise<string>;
    decrypt: (token: string) => Promise<string>;
  }) | undefined;
}
/**
 * The otp adapter is created from the totp adapter.
 */
declare const otp2fa: (options?: OTPOptions | undefined) => {
  id: "otp";
  endpoints: {
    /**
     * ### Endpoint
     *
     * POST `/two-factor/send-otp`
     *
     * ### API Methods
     *
     * **server:**
     * `auth.api.send2FaOTP`
     *
     * **client:**
     * `authClient.twoFactor.sendOtp`
     *
     * @see [Read our docs to learn more.](https://better-auth.com/docs/plugins/2fa#api-method-two-factor-send-otp)
     */
    sendTwoFactorOTP: better_call688.StrictEndpoint<"/two-factor/send-otp", {
      method: "POST";
      body: z.ZodOptional<z.ZodObject<{
        trustDevice: z.ZodOptional<z.ZodBoolean>;
      }, z.core.$strip>>;
      metadata: {
        openapi: {
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      status: {
                        type: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    } & {
      use: any[];
    }, {
      status: boolean;
    }>;
    /**
     * ### Endpoint
     *
     * POST `/two-factor/verify-otp`
     *
     * ### API Methods
     *
     * **server:**
     * `auth.api.verifyOTP`
     *
     * **client:**
     * `authClient.twoFactor.verifyOtp`
     *
     * @see [Read our docs to learn more.](https://better-auth.com/docs/plugins/2fa#api-method-two-factor-verify-otp)
     */
    verifyTwoFactorOTP: better_call688.StrictEndpoint<"/two-factor/verify-otp", {
      method: "POST";
      body: z.ZodObject<{
        code: z.ZodString;
        trustDevice: z.ZodOptional<z.ZodBoolean>;
      }, z.core.$strip>;
      metadata: {
        openapi: {
          summary: string;
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      token: {
                        type: string;
                        description: string;
                      };
                      user: {
                        type: string;
                        properties: {
                          id: {
                            type: string;
                            description: string;
                          };
                          email: {
                            type: string;
                            format: string;
                            nullable: boolean;
                            description: string;
                          };
                          emailVerified: {
                            type: string;
                            nullable: boolean;
                            description: string;
                          };
                          name: {
                            type: string;
                            nullable: boolean;
                            description: string;
                          };
                          image: {
                            type: string;
                            format: string;
                            nullable: boolean;
                            description: string;
                          };
                          createdAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                          updatedAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                        };
                        required: string[];
                        description: string;
                      };
                    };
                    required: string[];
                  };
                };
              };
            };
          };
        };
      };
    } & {
      use: any[];
    }, {
      token: string;
      user: {
        id: string;
        email: string;
        emailVerified: boolean;
        name: string;
        image: string | null | undefined;
        createdAt: Date;
        updatedAt: Date;
      };
    }>;
  };
};
//#endregion
//#region src/plugins/two-factor/schema.d.ts
declare const schema: {
  user: {
    fields: {
      twoFactorEnabled: {
        type: "boolean";
        required: false;
        defaultValue: false;
        input: false;
      };
    };
  };
  twoFactor: {
    fields: {
      secret: {
        type: "string";
        required: true;
        returned: false;
        index: true;
      };
      backupCodes: {
        type: "string";
        required: true;
        returned: false;
      };
      userId: {
        type: "string";
        required: true;
        returned: false;
        references: {
          model: string;
          field: string;
        };
        index: true;
      };
    };
  };
};
//#endregion
//#region src/plugins/two-factor/totp/index.d.ts
type TOTPOptions = {
  /**
   * Issuer
   */
  issuer?: string | undefined;
  /**
   * How many digits the otp to be
   *
   * @default 6
   */
  digits?: (6 | 8) | undefined;
  /**
   * Period for otp in seconds.
   * @default 30
   */
  period?: number | undefined;
  /**
   * Backup codes configuration
   */
  backupCodes?: BackupCodeOptions | undefined;
  /**
   * Disable totp
   */
  disable?: boolean | undefined;
};
declare const totp2fa: (options?: TOTPOptions | undefined) => {
  id: "totp";
  endpoints: {
    /**
     * ### Endpoint
     *
     * POST `/totp/generate`
     *
     * ### API Methods
     *
     * **server:**
     * `auth.api.generateTOTP`
     *
     * @see [Read our docs to learn more.](https://better-auth.com/docs/plugins/2fa#totp)
     */
    generateTOTP: better_call688.StrictEndpoint<"/totp/generate", {
      method: "POST";
      body: z.ZodObject<{
        secret: z.ZodString;
      }, z.core.$strip>;
      metadata: {
        openapi: {
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      code: {
                        type: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
        SERVER_ONLY: true;
      };
    } & {
      use: any[];
    }, {
      code: string;
    }>;
    /**
     * ### Endpoint
     *
     * POST `/two-factor/get-totp-uri`
     *
     * ### API Methods
     *
     * **server:**
     * `auth.api.getTOTPURI`
     *
     * **client:**
     * `authClient.twoFactor.getTotpUri`
     *
     * @see [Read our docs to learn more.](https://better-auth.com/docs/plugins/2fa#getting-totp-uri)
     */
    getTOTPURI: better_call688.StrictEndpoint<"/two-factor/get-totp-uri", {
      method: "POST";
      use: ((inputContext: better_call688.MiddlewareInputContext<better_call688.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      body: z.ZodObject<{
        password: z.ZodString;
      }, z.core.$strip>;
      metadata: {
        openapi: {
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      totpURI: {
                        type: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    } & {
      use: any[];
    }, {
      totpURI: string;
    }>;
    /**
     * ### Endpoint
     *
     * POST `/two-factor/verify-totp`
     *
     * ### API Methods
     *
     * **server:**
     * `auth.api.verifyTOTP`
     *
     * **client:**
     * `authClient.twoFactor.verifyTotp`
     *
     * @see [Read our docs to learn more.](https://better-auth.com/docs/plugins/2fa#verifying-totp)
     */
    verifyTOTP: better_call688.StrictEndpoint<"/two-factor/verify-totp", {
      method: "POST";
      body: z.ZodObject<{
        code: z.ZodString;
        trustDevice: z.ZodOptional<z.ZodBoolean>;
      }, z.core.$strip>;
      metadata: {
        openapi: {
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      status: {
                        type: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    } & {
      use: any[];
    }, {
      token: string;
      user: {
        id: string;
        email: string;
        emailVerified: boolean;
        name: string;
        image: string | null | undefined;
        createdAt: Date;
        updatedAt: Date;
      };
    }>;
  };
};
//#endregion
//#region src/plugins/two-factor/types.d.ts
interface TwoFactorOptions {
  /**
   * Application Name
   */
  issuer?: string | undefined;
  /**
   * TOTP OPtions
   */
  totpOptions?: Omit<TOTPOptions, "issuer"> | undefined;
  /**
   * OTP Options
   */
  otpOptions?: OTPOptions | undefined;
  /**
   * Backup code options
   */
  backupCodeOptions?: BackupCodeOptions | undefined;
  /**
   * Skip verification on enabling two factor authentication.
   * @default false
   */
  skipVerificationOnEnable?: boolean | undefined;
  /**
   * Custom schema for the two factor plugin
   */
  schema?: InferOptionSchema<typeof schema> | undefined;
}
interface UserWithTwoFactor extends User {
  /**
   * If the user has enabled two factor authentication.
   */
  twoFactorEnabled: boolean;
}
interface TwoFactorProvider {
  id: LiteralString;
  endpoints?: BetterAuthPlugin["endpoints"] | undefined;
}
interface TwoFactorTable {
  userId: string;
  secret: string;
  backupCodes: string;
  enabled: boolean;
}
//#endregion
//#region src/plugins/two-factor/error-code.d.ts
declare const TWO_FACTOR_ERROR_CODES: {
  readonly OTP_NOT_ENABLED: "OTP not enabled";
  readonly OTP_HAS_EXPIRED: "OTP has expired";
  readonly TOTP_NOT_ENABLED: "TOTP not enabled";
  readonly TWO_FACTOR_NOT_ENABLED: "Two factor isn't enabled";
  readonly BACKUP_CODES_NOT_ENABLED: "Backup codes aren't enabled";
  readonly INVALID_BACKUP_CODE: "Invalid backup code";
  readonly INVALID_CODE: "Invalid code";
  readonly TOO_MANY_ATTEMPTS_REQUEST_NEW_CODE: "Too many attempts. Please request a new code.";
  readonly INVALID_TWO_FACTOR_COOKIE: "Invalid two factor cookie";
};
//#endregion
//#region src/plugins/two-factor/client.d.ts
declare const twoFactorClient: (options?: {
  /**
   * a redirect function to call if a user needs to verify
   * their two factor
   */
  onTwoFactorRedirect?: () => void | Promise<void>;
} | undefined) => {
  id: "two-factor";
  $InferServerPlugin: ReturnType<typeof twoFactor>;
  atomListeners: {
    matcher: (path: string) => boolean;
    signal: "$sessionSignal";
  }[];
  pathMethods: {
    "/two-factor/disable": "POST";
    "/two-factor/enable": "POST";
    "/two-factor/send-otp": "POST";
    "/two-factor/generate-backup-codes": "POST";
  };
  fetchPlugins: {
    id: string;
    name: string;
    hooks: {
      onSuccess(context: _better_fetch_fetch94.SuccessContext<any>): Promise<void>;
    };
  }[];
};
//#endregion
//#region src/plugins/two-factor/index.d.ts
declare const twoFactor: (options?: TwoFactorOptions | undefined) => {
  id: "two-factor";
  endpoints: {
    /**
     * ### Endpoint
     *
     * POST `/two-factor/enable`
     *
     * ### API Methods
     *
     * **server:**
     * `auth.api.enableTwoFactor`
     *
     * **client:**
     * `authClient.twoFactor.enable`
     *
     * @see [Read our docs to learn more.](https://better-auth.com/docs/plugins/2fa#api-method-two-factor-enable)
     */
    enableTwoFactor: better_call688.StrictEndpoint<"/two-factor/enable", {
      method: "POST";
      body: z.ZodObject<{
        password: z.ZodString;
        issuer: z.ZodOptional<z.ZodString>;
      }, z.core.$strip>;
      use: ((inputContext: better_call688.MiddlewareInputContext<better_call688.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      totpURI: {
                        type: string;
                        description: string;
                      };
                      backupCodes: {
                        type: string;
                        items: {
                          type: string;
                        };
                        description: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    } & {
      use: any[];
    }, {
      totpURI: string;
      backupCodes: string[];
    }>;
    /**
     * ### Endpoint
     *
     * POST `/two-factor/disable`
     *
     * ### API Methods
     *
     * **server:**
     * `auth.api.disableTwoFactor`
     *
     * **client:**
     * `authClient.twoFactor.disable`
     *
     * @see [Read our docs to learn more.](https://better-auth.com/docs/plugins/2fa#api-method-two-factor-disable)
     */
    disableTwoFactor: better_call688.StrictEndpoint<"/two-factor/disable", {
      method: "POST";
      body: z.ZodObject<{
        password: z.ZodString;
      }, z.core.$strip>;
      use: ((inputContext: better_call688.MiddlewareInputContext<better_call688.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      status: {
                        type: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    } & {
      use: any[];
    }, {
      status: boolean;
    }>;
    verifyBackupCode: better_call688.StrictEndpoint<"/two-factor/verify-backup-code", {
      method: "POST";
      body: z.ZodObject<{
        code: z.ZodString;
        disableSession: z.ZodOptional<z.ZodBoolean>;
        trustDevice: z.ZodOptional<z.ZodBoolean>;
      }, z.core.$strip>;
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      user: {
                        type: string;
                        properties: {
                          id: {
                            type: string;
                            description: string;
                          };
                          email: {
                            type: string;
                            format: string;
                            nullable: boolean;
                            description: string;
                          };
                          emailVerified: {
                            type: string;
                            nullable: boolean;
                            description: string;
                          };
                          name: {
                            type: string;
                            nullable: boolean;
                            description: string;
                          };
                          image: {
                            type: string;
                            format: string;
                            nullable: boolean;
                            description: string;
                          };
                          twoFactorEnabled: {
                            type: string;
                            description: string;
                          };
                          createdAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                          updatedAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                        };
                        required: string[];
                        description: string;
                      };
                      session: {
                        type: string;
                        properties: {
                          token: {
                            type: string;
                            description: string;
                          };
                          userId: {
                            type: string;
                            description: string;
                          };
                          createdAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                          expiresAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                        };
                        required: string[];
                        description: string;
                      };
                    };
                    required: string[];
                  };
                };
              };
            };
          };
        };
      };
    } & {
      use: any[];
    }, {
      token: string | undefined;
      user: {
        id: string;
        email: string;
        emailVerified: boolean;
        name: string;
        image: string | null | undefined;
        createdAt: Date;
        updatedAt: Date;
      };
    }>;
    generateBackupCodes: better_call688.StrictEndpoint<"/two-factor/generate-backup-codes", {
      method: "POST";
      body: z.ZodObject<{
        password: z.ZodString;
      }, z.core.$strip>;
      use: ((inputContext: better_call688.MiddlewareInputContext<better_call688.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      metadata: {
        openapi: {
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      status: {
                        type: string;
                        description: string;
                        enum: boolean[];
                      };
                      backupCodes: {
                        type: string;
                        items: {
                          type: string;
                        };
                        description: string;
                      };
                    };
                    required: string[];
                  };
                };
              };
            };
          };
        };
      };
    } & {
      use: any[];
    }, {
      status: boolean;
      backupCodes: string[];
    }>;
    viewBackupCodes: better_call688.StrictEndpoint<"/two-factor/view-backup-codes", {
      method: "POST";
      body: z.ZodObject<{
        userId: z.ZodCoercedString<unknown>;
      }, z.core.$strip>;
      metadata: {
        SERVER_ONLY: true;
      };
    } & {
      use: any[];
    }, {
      status: boolean;
      backupCodes: string[];
    }>;
    sendTwoFactorOTP: better_call688.StrictEndpoint<"/two-factor/send-otp", {
      method: "POST";
      body: z.ZodOptional<z.ZodObject<{
        trustDevice: z.ZodOptional<z.ZodBoolean>;
      }, z.core.$strip>>;
      metadata: {
        openapi: {
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      status: {
                        type: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    } & {
      use: any[];
    }, {
      status: boolean;
    }>;
    verifyTwoFactorOTP: better_call688.StrictEndpoint<"/two-factor/verify-otp", {
      method: "POST";
      body: z.ZodObject<{
        code: z.ZodString;
        trustDevice: z.ZodOptional<z.ZodBoolean>;
      }, z.core.$strip>;
      metadata: {
        openapi: {
          summary: string;
          description: string;
          responses: {
            "200": {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      token: {
                        type: string;
                        description: string;
                      };
                      user: {
                        type: string;
                        properties: {
                          id: {
                            type: string;
                            description: string;
                          };
                          email: {
                            type: string;
                            format: string;
                            nullable: boolean;
                            description: string;
                          };
                          emailVerified: {
                            type: string;
                            nullable: boolean;
                            description: string;
                          };
                          name: {
                            type: string;
                            nullable: boolean;
                            description: string;
                          };
                          image: {
                            type: string;
                            format: string;
                            nullable: boolean;
                            description: string;
                          };
                          createdAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                          updatedAt: {
                            type: string;
                            format: string;
                            description: string;
                          };
                        };
                        required: string[];
                        description: string;
                      };
                    };
                    required: string[];
                  };
                };
              };
            };
          };
        };
      };
    } & {
      use: any[];
    }, {
      token: string;
      user: {
        id: string;
        email: string;
        emailVerified: boolean;
        name: string;
        image: string | null | undefined;
        createdAt: Date;
        updatedAt: Date;
      };
    }>;
    generateTOTP: better_call688.StrictEndpoint<"/totp/generate", {
      method: "POST";
      body: z.ZodObject<{
        secret: z.ZodString;
      }, z.core.$strip>;
      metadata: {
        openapi: {
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      code: {
                        type: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
        SERVER_ONLY: true;
      };
    } & {
      use: any[];
    }, {
      code: string;
    }>;
    getTOTPURI: better_call688.StrictEndpoint<"/two-factor/get-totp-uri", {
      method: "POST";
      use: ((inputContext: better_call688.MiddlewareInputContext<better_call688.MiddlewareOptions>) => Promise<{
        session: {
          session: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            userId: string;
            expiresAt: Date;
            token: string;
            ipAddress?: string | null | undefined;
            userAgent?: string | null | undefined;
          };
          user: Record<string, any> & {
            id: string;
            createdAt: Date;
            updatedAt: Date;
            email: string;
            emailVerified: boolean;
            name: string;
            image?: string | null | undefined;
          };
        };
      }>)[];
      body: z.ZodObject<{
        password: z.ZodString;
      }, z.core.$strip>;
      metadata: {
        openapi: {
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      totpURI: {
                        type: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    } & {
      use: any[];
    }, {
      totpURI: string;
    }>;
    verifyTOTP: better_call688.StrictEndpoint<"/two-factor/verify-totp", {
      method: "POST";
      body: z.ZodObject<{
        code: z.ZodString;
        trustDevice: z.ZodOptional<z.ZodBoolean>;
      }, z.core.$strip>;
      metadata: {
        openapi: {
          summary: string;
          description: string;
          responses: {
            200: {
              description: string;
              content: {
                "application/json": {
                  schema: {
                    type: "object";
                    properties: {
                      status: {
                        type: string;
                      };
                    };
                  };
                };
              };
            };
          };
        };
      };
    } & {
      use: any[];
    }, {
      token: string;
      user: {
        id: string;
        email: string;
        emailVerified: boolean;
        name: string;
        image: string | null | undefined;
        createdAt: Date;
        updatedAt: Date;
      };
    }>;
  };
  options: TwoFactorOptions | undefined;
  hooks: {
    after: {
      matcher(context: _better_auth_core20.HookEndpointContext): boolean;
      handler: (inputContext: better_call688.MiddlewareInputContext<better_call688.MiddlewareOptions>) => Promise<{
        twoFactorRedirect: boolean;
      } | undefined>;
    }[];
  };
  schema: {
    user: {
      fields: {
        twoFactorEnabled: {
          type: "boolean";
          required: false;
          defaultValue: false;
          input: false;
        };
      };
    };
    twoFactor: {
      fields: {
        secret: {
          type: "string";
          required: true;
          returned: false;
          index: true;
        };
        backupCodes: {
          type: "string";
          required: true;
          returned: false;
        };
        userId: {
          type: "string";
          required: true;
          returned: false;
          references: {
            model: string;
            field: string;
          };
          index: true;
        };
      };
    };
  };
  rateLimit: {
    pathMatcher(path: string): boolean;
    window: number;
    max: number;
  }[];
  $ERROR_CODES: {
    readonly OTP_NOT_ENABLED: "OTP not enabled";
    readonly OTP_HAS_EXPIRED: "OTP has expired";
    readonly TOTP_NOT_ENABLED: "TOTP not enabled";
    readonly TWO_FACTOR_NOT_ENABLED: "Two factor isn't enabled";
    readonly BACKUP_CODES_NOT_ENABLED: "Backup codes aren't enabled";
    readonly INVALID_BACKUP_CODE: "Invalid backup code";
    readonly INVALID_CODE: "Invalid code";
    readonly TOO_MANY_ATTEMPTS_REQUEST_NEW_CODE: "Too many attempts. Please request a new code.";
    readonly INVALID_TWO_FACTOR_COOKIE: "Invalid two factor cookie";
  };
};
//#endregion
export { TwoFactorProvider as a, TOTPOptions as c, otp2fa as d, BackupCodeOptions as f, verifyBackupCode as g, getBackupCodes as h, TwoFactorOptions as i, totp2fa as l, generateBackupCodes as m, twoFactorClient as n, TwoFactorTable as o, backupCode2fa as p, TWO_FACTOR_ERROR_CODES as r, UserWithTwoFactor as s, twoFactor as t, OTPOptions as u };