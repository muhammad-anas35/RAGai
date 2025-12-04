import { pgTable, text, timestamp, boolean } from 'drizzle-orm/pg-core';

// Users table for authentication
export const users = pgTable('users', {
    id: text('id').primaryKey(),
    email: text('email').notNull().unique(),
    name: text('name'),
    passwordHash: text('password_hash'),
    emailVerified: boolean('email_verified').default(false),
    createdAt: timestamp('created_at').defaultNow(),
    updatedAt: timestamp('updated_at').defaultNow(),
});

// Sessions table for better-auth
export const sessions = pgTable('sessions', {
    id: text('id').primaryKey(),
    userId: text('user_id').references(() => users.id, { onDelete: 'cascade' }).notNull(),
    expiresAt: timestamp('expires_at').notNull(),
    createdAt: timestamp('created_at').defaultNow(),
});

// Chat history for RAG conversations
export const chatHistory = pgTable('chat_history', {
    id: text('id').primaryKey(),
    userId: text('user_id').references(() => users.id, { onDelete: 'cascade' }).notNull(),
    message: text('message').notNull(),
    response: text('response').notNull(),
    createdAt: timestamp('created_at').defaultNow(),
});

// Refresh tokens table for JWT
export const refreshTokens = pgTable('refresh_tokens', {
    id: text('id').primaryKey(),
    userId: text('user_id').references(() => users.id, { onDelete: 'cascade' }).notNull(),
    token: text('token').notNull().unique(),
    expiresAt: timestamp('expires_at').notNull(),
    issuedAt: timestamp('issued_at').defaultNow(),
    revoked: boolean('revoked').default(false),
});
