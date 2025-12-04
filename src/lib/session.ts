/**
 * Session Utilities
 * 
 * Helper functions for session management.
 * Based on @authentication_agent session management patterns.
 */

import { auth } from './auth';
import type { NextApiRequest } from 'next';

/**
 * Get current session from request
 * @param req - Next.js API request
 * @returns Session object or null
 */
export async function getSession(req: NextApiRequest) {
    try {
        const session = await auth.api.getSession({
            headers: req.headers as any,
        });
        return session;
    } catch (error) {
        console.error('Session error:', error);
        return null;
    }
}

/**
 * Require authentication middleware
 * @param req - Next.js API request
 * @throws Error if not authenticated
 */
export async function requireAuth(req: NextApiRequest) {
    const session = await getSession(req);

    if (!session || !session.user) {
        throw new Error('Unauthorized');
    }

    return session;
}

/**
 * Get user ID from session
 * @param req - Next.js API request
 * @returns User ID or null
 */
export async function getUserId(req: NextApiRequest): Promise<string | null> {
    const session = await getSession(req);
    return session?.user?.id || null;
}
