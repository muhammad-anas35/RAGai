import { pgTable, uuid, varchar, text, timestamp, integer, jsonb, boolean } from 'drizzle-orm/pg-core';
import { users } from './users';

/**
 * Chat-related database schema
 * Stores conversations and messages
 */

export const conversations = pgTable('conversations', {
    id: uuid('id').primaryKey().defaultRandom(),
    userId: uuid('user_id').notNull().references(() => users.id, { onDelete: 'cascade' }),
    title: varchar('title', { length: 255 }),
    createdAt: timestamp('created_at').defaultNow().notNull(),
    updatedAt: timestamp('updated_at').defaultNow().notNull(),
});

export const chatMessages = pgTable('chat_messages', {
    id: uuid('id').primaryKey().defaultRandom(),
    conversationId: uuid('conversation_id').notNull().references(() => conversations.id, { onDelete: 'cascade' }),
    role: varchar('role', { length: 20 }).notNull(), // 'user' or 'assistant'
    content: text('content').notNull(),
    sources: jsonb('sources'), // Array of source citations
    createdAt: timestamp('created_at').defaultNow().notNull(),
});

export const userPreferences = pgTable('user_preferences', {
    id: uuid('id').primaryKey().defaultRandom(),
    userId: uuid('user_id').notNull().unique().references(() => users.id, { onDelete: 'cascade' }),
    theme: varchar('theme', { length: 20 }).default('light'), // 'light', 'dark', 'auto'
    language: varchar('language', { length: 10 }).default('en'),
    chatSettings: jsonb('chat_settings').default('{}'), // JSON object for chat preferences
    createdAt: timestamp('created_at').defaultNow().notNull(),
    updatedAt: timestamp('updated_at').defaultNow().notNull(),
});

// Type exports for TypeScript
export type Conversation = typeof conversations.$inferSelect;
export type NewConversation = typeof conversations.$inferInsert;
export type ChatMessage = typeof chatMessages.$inferSelect;
export type NewChatMessage = typeof chatMessages.$inferInsert;
export type UserPreferences = typeof userPreferences.$inferSelect;
export type NewUserPreferences = typeof userPreferences.$inferInsert;
