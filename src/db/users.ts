/**
 * User Database Operations
 * 
 * CRUD operations for user management.
 * Following @neon-database skill query patterns and @database_agent best practices.
 */

import { db } from './index';
import { users } from './schema';
import { eq } from 'drizzle-orm';

/**
 * Find user by email
 * Reference: @neon-database/references/query-patterns.md
 */
export async function getUserByEmail(email: string) {
    const [user] = await db
        .select()
        .from(users)
        .where(eq(users.email, email));

    return user;
}

/**
 * Find user by ID
 */
export async function getUserById(id: string) {
    const [user] = await db
        .select()
        .from(users)
        .where(eq(users.id, id));

    return user;
}

/**
 * Create new user
 * Reference: @neon-database/references/query-patterns.md
 */
export async function createUser(data: {
    id: string;
    email: string;
    name?: string;
    passwordHash?: string;
}) {
    const [user] = await db
        .insert(users)
        .values({
            ...data,
            emailVerified: false,
            createdAt: new Date(),
            updatedAt: new Date(),
        })
        .returning();

    return user;
}

/**
 * Update user email verification status
 */
export async function verifyUserEmail(userId: string) {
    const [user] = await db
        .update(users)
        .set({
            emailVerified: true,
            updatedAt: new Date(),
        })
        .where(eq(users.id, userId))
        .returning();

    return user;
}

/**
 * Update user password
 */
export async function updateUserPassword(userId: string, passwordHash: string) {
    const [user] = await db
        .update(users)
        .set({
            passwordHash,
            updatedAt: new Date(),
        })
        .where(eq(users.id, userId))
        .returning();

    return user;
}

/**
 * Delete user (cascade deletes sessions and chat history)
 */
export async function deleteUser(userId: string) {
    await db
        .delete(users)
        .where(eq(users.id, userId));
}
