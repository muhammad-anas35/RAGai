# Book RAG Data Model

## Overview
This document defines the database schema and data model for the Book RAG project using Drizzle ORM with Neon PostgreSQL.

---

## Entities

### 1. User
Stores user account information and authentication details.

**Table**: `users`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Unique user identifier |
| `email` | VARCHAR(255) | UNIQUE, NOT NULL | User email address |
| `name` | VARCHAR(255) | NULL | User display name |
| `emailVerified` | BOOLEAN | DEFAULT FALSE | Email verification status |
| `image` | TEXT | NULL | Profile image URL |
| `createdAt` | TIMESTAMP | DEFAULT NOW() | Account creation timestamp |
| `updatedAt` | TIMESTAMP | DEFAULT NOW() | Last update timestamp |

**Indexes**:
- `idx_users_email` on `email` (unique)

**Relationships**:
- One-to-many with `Session`
- One-to-many with `Account`
- One-to-many with `Conversation`
- One-to-one with `UserPreferences`

---

### 2. Account
Stores OAuth provider information for users.

**Table**: `accounts`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Unique account identifier |
| `userId` | UUID | FOREIGN KEY → users(id), NOT NULL | Reference to user |
| `accountId` | VARCHAR(255) | NOT NULL | Provider account ID |
| `providerId` | VARCHAR(255) | NOT NULL | OAuth provider (google, github) |
| `accessToken` | TEXT | NULL | OAuth access token |
| `refreshToken` | TEXT | NULL | OAuth refresh token |
| `expiresAt` | TIMESTAMP | NULL | Token expiration time |
| `createdAt` | TIMESTAMP | DEFAULT NOW() | Account creation timestamp |

**Indexes**:
- `idx_accounts_userId` on `userId`
- `idx_accounts_provider` on `providerId, accountId` (unique)

**Relationships**:
- Many-to-one with `User`

---

### 3. Session
Stores user session tokens for authentication.

**Table**: `sessions`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Unique session identifier |
| `userId` | UUID | FOREIGN KEY → users(id), NOT NULL | Reference to user |
| `sessionToken` | VARCHAR(255) | UNIQUE, NOT NULL | Session token |
| `expiresAt` | TIMESTAMP | NOT NULL | Session expiration time |
| `ipAddress` | VARCHAR(45) | NULL | IP address of session |
| `userAgent` | TEXT | NULL | User agent string |
| `createdAt` | TIMESTAMP | DEFAULT NOW() | Session creation timestamp |

**Indexes**:
- `idx_sessions_token` on `sessionToken` (unique)
- `idx_sessions_userId` on `userId`
- `idx_sessions_expiresAt` on `expiresAt`

**Relationships**:
- Many-to-one with `User`

---

### 4. Conversation
Stores chat conversations between users and the AI.

**Table**: `conversations`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Unique conversation identifier |
| `userId` | UUID | FOREIGN KEY → users(id), NOT NULL | Reference to user |
| `title` | VARCHAR(255) | NULL | Conversation title (auto-generated) |
| `createdAt` | TIMESTAMP | DEFAULT NOW() | Conversation creation timestamp |
| `updatedAt` | TIMESTAMP | DEFAULT NOW() | Last message timestamp |

**Indexes**:
- `idx_conversations_userId` on `userId`
- `idx_conversations_updatedAt` on `updatedAt`

**Relationships**:
- Many-to-one with `User`
- One-to-many with `ChatMessage`

---

### 5. ChatMessage
Stores individual messages within conversations.

**Table**: `chat_messages`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Unique message identifier |
| `conversationId` | UUID | FOREIGN KEY → conversations(id), NOT NULL | Reference to conversation |
| `role` | VARCHAR(20) | NOT NULL, CHECK IN ('user', 'assistant') | Message sender role |
| `content` | TEXT | NOT NULL | Message content |
| `sources` | JSONB | NULL | Source citations (for AI responses) |
| `createdAt` | TIMESTAMP | DEFAULT NOW() | Message creation timestamp |

**Indexes**:
- `idx_messages_conversationId` on `conversationId`
- `idx_messages_createdAt` on `createdAt`

**Relationships**:
- Many-to-one with `Conversation`

**JSONB Structure for `sources`**:
```json
[
  {
    "chunkId": "uuid",
    "documentPath": "docs/chapter1/foundations.md",
    "content": "excerpt from source",
    "score": 0.85
  }
]
```

---

### 6. Document
Stores metadata about ingested documents.

**Table**: `documents`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Unique document identifier |
| `path` | TEXT | UNIQUE, NOT NULL | Document file path |
| `title` | VARCHAR(255) | NOT NULL | Document title |
| `chapter` | VARCHAR(100) | NULL | Chapter identifier |
| `section` | VARCHAR(100) | NULL | Section identifier |
| `wordCount` | INTEGER | DEFAULT 0 | Total word count |
| `chunkCount` | INTEGER | DEFAULT 0 | Number of chunks |
| `ingestedAt` | TIMESTAMP | DEFAULT NOW() | Ingestion timestamp |
| `updatedAt` | TIMESTAMP | DEFAULT NOW() | Last update timestamp |

**Indexes**:
- `idx_documents_path` on `path` (unique)
- `idx_documents_chapter` on `chapter`

**Relationships**:
- One-to-many with `DocumentChunk`

---

### 7. DocumentChunk
Stores text chunks from documents for RAG retrieval.

**Table**: `document_chunks`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Unique chunk identifier |
| `documentId` | UUID | FOREIGN KEY → documents(id), NOT NULL | Reference to document |
| `chunkIndex` | INTEGER | NOT NULL | Chunk position in document |
| `content` | TEXT | NOT NULL | Chunk text content |
| `wordCount` | INTEGER | DEFAULT 0 | Chunk word count |
| `vectorId` | VARCHAR(255) | UNIQUE, NULL | Qdrant vector ID |
| `createdAt` | TIMESTAMP | DEFAULT NOW() | Chunk creation timestamp |

**Indexes**:
- `idx_chunks_documentId` on `documentId`
- `idx_chunks_vectorId` on `vectorId` (unique)

**Relationships**:
- Many-to-one with `Document`

**Note**: The actual vector embeddings are stored in Qdrant, referenced by `vectorId`.

---

### 8. UserPreferences
Stores user-specific preferences and settings.

**Table**: `user_preferences`

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Unique preferences identifier |
| `userId` | UUID | FOREIGN KEY → users(id), UNIQUE, NOT NULL | Reference to user |
| `theme` | VARCHAR(20) | DEFAULT 'light' | UI theme (light, dark, auto) |
| `language` | VARCHAR(10) | DEFAULT 'en' | Preferred language |
| `chatSettings` | JSONB | DEFAULT '{}' | Chat-specific settings |
| `createdAt` | TIMESTAMP | DEFAULT NOW() | Preferences creation timestamp |
| `updatedAt` | TIMESTAMP | DEFAULT NOW() | Last update timestamp |

**Indexes**:
- `idx_preferences_userId` on `userId` (unique)

**Relationships**:
- One-to-one with `User`

**JSONB Structure for `chatSettings`**:
```json
{
  "temperature": 0.7,
  "maxTokens": 1000,
  "showSources": true,
  "autoSave": true
}
```

---

## Relationships Diagram

```
User (1) ──────< (M) Session
  │
  ├──────< (M) Account
  │
  ├──────< (M) Conversation ──────< (M) ChatMessage
  │
  └──────< (1) UserPreferences

Document (1) ──────< (M) DocumentChunk
```

---

## Data Constraints

### Business Rules
1. **Email Uniqueness**: Each email can only be associated with one user account
2. **Session Expiry**: Sessions expire after 24 hours of inactivity
3. **Conversation Ownership**: Users can only access their own conversations
4. **Document Integrity**: Deleting a document cascades to all its chunks
5. **User Deletion**: Deleting a user cascades to sessions, conversations, and preferences

### Validation Rules
1. **Email Format**: Must be valid email format (validated by Better Auth)
2. **Password**: Minimum 8 characters (validated by Better Auth)
3. **Role**: Must be 'user' or 'assistant'
4. **Theme**: Must be 'light', 'dark', or 'auto'
5. **Chunk Index**: Must be >= 0

---

## Indexes Strategy

### Performance Optimization
- **User lookups**: Index on `email` for fast authentication
- **Session validation**: Index on `sessionToken` for quick session checks
- **Conversation retrieval**: Index on `userId` and `updatedAt` for efficient history queries
- **Message ordering**: Index on `conversationId` and `createdAt` for chronological display
- **Document search**: Index on `path` and `chapter` for content organization

---

## Migration Strategy

### Phase 1: Core Authentication (✅ Complete)
- Create `users`, `accounts`, `sessions` tables
- Setup Better Auth integration

### Phase 2: Chat System (Planned - Phase 5)
- Create `conversations`, `chat_messages` tables
- Add foreign key constraints

### Phase 3: Document Ingestion (Planned - Phase 3)
- Create `documents`, `document_chunks` tables
- Setup Qdrant integration

### Phase 4: User Preferences (Planned - Phase 5)
- Create `user_preferences` table
- Add default preferences for existing users

---

## Data Retention Policy

### User Data
- **Active accounts**: Retained indefinitely
- **Inactive accounts**: Flagged after 1 year, deleted after 2 years
- **Deleted accounts**: All associated data deleted within 30 days

### Chat History
- **Recent conversations**: Retained for 1 year
- **Archived conversations**: User can manually delete
- **Orphaned messages**: Deleted with conversation

### Document Data
- **Published content**: Retained indefinitely
- **Deprecated content**: Archived but not deleted
- **Vector embeddings**: Synced with document lifecycle

---

## Security Considerations

### Sensitive Data
- **Passwords**: Never stored (handled by Better Auth with bcrypt)
- **OAuth tokens**: Encrypted at rest
- **Session tokens**: Cryptographically secure, httpOnly cookies
- **PII**: Email and name only, minimal data collection

### Access Control
- **Row-level security**: Users can only access their own data
- **API authorization**: All endpoints check user authentication
- **Database user**: Least privilege principle

---

## Drizzle Schema Files

### Location
- `src/db/schema/users.ts` - User, Account, Session
- `src/db/schema/chat.ts` - Conversation, ChatMessage
- `src/db/schema/documents.ts` - Document, DocumentChunk
- `src/db/schema/preferences.ts` - UserPreferences

### Example Schema (User)
```typescript
import { pgTable, uuid, varchar, boolean, timestamp } from 'drizzle-orm/pg-core';

export const users = pgTable('users', {
  id: uuid('id').primaryKey().defaultRandom(),
  email: varchar('email', { length: 255 }).notNull().unique(),
  name: varchar('name', { length: 255 }),
  emailVerified: boolean('email_verified').default(false),
  image: text('image'),
  createdAt: timestamp('created_at').defaultNow(),
  updatedAt: timestamp('updated_at').defaultNow(),
});
```

---

## Next Steps

1. **Phase 3**: Create `documents` and `document_chunks` tables
2. **Phase 5**: Create `conversations`, `chat_messages`, and `user_preferences` tables
3. **Testing**: Write migration tests for all schema changes
4. **Documentation**: Update API documentation with data models
