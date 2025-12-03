import "../../../index-COnelCGa.mjs";
import "../../../types-CEepZ-RG.mjs";
import "../../../helper-DFzV6jvx.mjs";
import "../../../plugins-Dmw3tk5H.mjs";
import { c as Subset, t as AuthorizeResponse } from "../../../index-rnNTrfA7.mjs";
import "../../../index-F8qM450o.mjs";
import "../../../index-CMfXEnyA.mjs";
import "../../../index-B64ouu9a.mjs";
import "../../../index-CWcnHbjn.mjs";
import "../../../index-DUa0239y.mjs";
import "../../../index-D2CJIZCI.mjs";
import "../../../index-cgrDi8bR.mjs";
import "../../../index-DLNLFTLS.mjs";
import "../../../index-Bv2nWTrZ.mjs";
import "../../../index-0IRW2A1b.mjs";
import "../../../index-BMrJpdkv.mjs";
import "../../../index-t-jeKL1G.mjs";
import "../../../index-DSwTw61P.mjs";
import "../../../index-DoUxtZgO.mjs";
import "../../../index-CmcoO98i.mjs";
import "../../../index-BaZax_le.mjs";
import "../../../index-alnYeOxT.mjs";
import "../../../index-Dk2q2Dnr.mjs";
import "../../../index-Drqkvf7w.mjs";
import "../../../index-CCnWzVSV.mjs";

//#region src/plugins/admin/access/statement.d.ts
declare const defaultStatements: {
  readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
  readonly session: readonly ["list", "revoke", "delete"];
};
declare const defaultAc: {
  newRole<K extends "session" | "user">(statements: Subset<K, {
    readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
    readonly session: readonly ["list", "revoke", "delete"];
  }>): {
    authorize<K_1 extends K>(request: K_1 extends infer T extends keyof Subset<K, {
      readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
      readonly session: readonly ["list", "revoke", "delete"];
    }> ? { [key in T]?: Subset<K, {
      readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
      readonly session: readonly ["list", "revoke", "delete"];
    }>[key] | {
      actions: Subset<K, {
        readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
        readonly session: readonly ["list", "revoke", "delete"];
      }>[key];
      connector: "OR" | "AND";
    } | undefined } : never, connector?: "OR" | "AND"): AuthorizeResponse;
    statements: Subset<K, {
      readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
      readonly session: readonly ["list", "revoke", "delete"];
    }>;
  };
  statements: {
    readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
    readonly session: readonly ["list", "revoke", "delete"];
  };
};
declare const adminAc: {
  authorize<K extends "session" | "user">(request: K extends infer T extends keyof Subset<"session" | "user", {
    readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
    readonly session: readonly ["list", "revoke", "delete"];
  }> ? { [key in T]?: Subset<"session" | "user", {
    readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
    readonly session: readonly ["list", "revoke", "delete"];
  }>[key] | {
    actions: Subset<"session" | "user", {
      readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
      readonly session: readonly ["list", "revoke", "delete"];
    }>[key];
    connector: "OR" | "AND";
  } | undefined } : never, connector?: "OR" | "AND"): AuthorizeResponse;
  statements: Subset<"session" | "user", {
    readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
    readonly session: readonly ["list", "revoke", "delete"];
  }>;
};
declare const userAc: {
  authorize<K extends "session" | "user">(request: K extends infer T extends keyof Subset<"session" | "user", {
    readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
    readonly session: readonly ["list", "revoke", "delete"];
  }> ? { [key in T]?: Subset<"session" | "user", {
    readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
    readonly session: readonly ["list", "revoke", "delete"];
  }>[key] | {
    actions: Subset<"session" | "user", {
      readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
      readonly session: readonly ["list", "revoke", "delete"];
    }>[key];
    connector: "OR" | "AND";
  } | undefined } : never, connector?: "OR" | "AND"): AuthorizeResponse;
  statements: Subset<"session" | "user", {
    readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
    readonly session: readonly ["list", "revoke", "delete"];
  }>;
};
declare const defaultRoles: {
  admin: {
    authorize<K extends "session" | "user">(request: K extends infer T extends keyof Subset<"session" | "user", {
      readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
      readonly session: readonly ["list", "revoke", "delete"];
    }> ? { [key in T]?: Subset<"session" | "user", {
      readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
      readonly session: readonly ["list", "revoke", "delete"];
    }>[key] | {
      actions: Subset<"session" | "user", {
        readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
        readonly session: readonly ["list", "revoke", "delete"];
      }>[key];
      connector: "OR" | "AND";
    } | undefined } : never, connector?: "OR" | "AND"): AuthorizeResponse;
    statements: Subset<"session" | "user", {
      readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
      readonly session: readonly ["list", "revoke", "delete"];
    }>;
  };
  user: {
    authorize<K extends "session" | "user">(request: K extends infer T extends keyof Subset<"session" | "user", {
      readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
      readonly session: readonly ["list", "revoke", "delete"];
    }> ? { [key in T]?: Subset<"session" | "user", {
      readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
      readonly session: readonly ["list", "revoke", "delete"];
    }>[key] | {
      actions: Subset<"session" | "user", {
        readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
        readonly session: readonly ["list", "revoke", "delete"];
      }>[key];
      connector: "OR" | "AND";
    } | undefined } : never, connector?: "OR" | "AND"): AuthorizeResponse;
    statements: Subset<"session" | "user", {
      readonly user: readonly ["create", "list", "set-role", "ban", "impersonate", "delete", "set-password", "get", "update"];
      readonly session: readonly ["list", "revoke", "delete"];
    }>;
  };
};
//#endregion
export { adminAc, defaultAc, defaultRoles, defaultStatements, userAc };