import { betterAuth } from 'better-auth';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { db } from '../db';
import * as schema from '../db/schema';

export const auth = betterAuth({
    database: drizzleAdapter(db, {
        provider: 'pg',
        schema: {
            user: schema.users,
            session: schema.sessions,
        },
    }),
    emailAndPassword: {
        enabled: true,
        requireEmailVerification: false, // Set to true in production
    },
    session: {
        expiresIn: 60 * 60 * 24 * 7, // 7 days
        updateAge: 60 * 60 * 24, // 1 day
    },
    socialProviders: {
        // Optional: Configure OAuth providers
        google: {
            clientId: process.env.GOOGLE_CLIENT_ID || '',
            clientSecret: process.env.GOOGLE_CLIENT_SECRET || '',
            enabled: !!process.env.GOOGLE_CLIENT_ID,
        },
        github: {
            clientId: process.env.GITHUB_CLIENT_ID || '',
            clientSecret: process.env.GITHUB_CLIENT_SECRET || '',
            enabled: !!process.env.GITHUB_CLIENT_ID,
        },
    },
});

export type Session = typeof auth.$Infer.Session;
export type User = typeof auth.$Infer.User;
