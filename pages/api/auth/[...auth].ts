/**
 * Better-Auth API Handler
 * 
 * This file creates the authentication API routes using better-auth.
 * Following @authentication_agent patterns and @neon-database skill guidelines.
 * 
 * Routes created:
 * - POST /api/auth/signup/email - Create new user
 * - POST /api/auth/signin/email - Login user
 * - POST /api/auth/signout - Logout user
 * - GET /api/auth/session - Get current session
 * - POST /api/auth/verify-email - Verify email
 * - POST /api/auth/reset-password - Reset password
 */

import { auth } from '@/lib/auth';
import type { NextApiRequest, NextApiResponse } from 'next';

export default async function handler(
    req: NextApiRequest,
    res: NextApiResponse
) {
    return auth.handler(req, res);
}

export const config = {
    api: {
        bodyParser: false, // better-auth handles body parsing
    },
};
