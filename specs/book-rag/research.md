# Research and Technical Decisions - Book RAG Project

## Overview
This document captures technical research, decisions, and rationale for the Book RAG project implementation.

---

## Technology Selection

### 1. Frontend Framework: Docusaurus

**Decision**: Use Docusaurus 3.9.2 for the frontend

**Rationale**:
- **Purpose-built for documentation**: Designed specifically for content-heavy sites
- **MDX support**: Allows embedding React components in markdown
- **Built-in features**: Search, versioning, i18n, dark mode
- **React-based**: Easy to add custom components (chat widget)
- **Performance**: Static site generation for fast loading
- **SEO-friendly**: Server-side rendering support

**Alternatives Considered**:
- **Next.js**: More flexible but overkill for documentation
- **VuePress**: Similar features but smaller ecosystem
- **GitBook**: Less customizable, vendor lock-in

**Trade-offs**:
- ✅ Excellent for documentation and books
- ✅ Large plugin ecosystem
- ❌ Less flexible than general-purpose frameworks
- ❌ Opinionated structure

---

### 2. AI Model: Google Gemini

**Decision**: Use Gemini family models (gemini-pro for chat, embedding-001 for embeddings)

**Rationale**:
- **Cost-effective**: Competitive pricing vs OpenAI
- **Performance**: State-of-the-art quality for chat and embeddings
- **Context window**: Large context window (up to 1M tokens for Gemini 1.5)
- **Multimodal**: Future support for images, videos (if needed)
- **API simplicity**: Easy-to-use SDK (@google/generative-ai)

**Alternatives Considered**:
- **OpenAI GPT-4**: Higher cost, similar quality
- **Anthropic Claude**: Good quality but less embedding support
- **Open-source models**: Lower cost but requires hosting

**Trade-offs**:
- ✅ Excellent price/performance ratio
- ✅ Large context window for RAG
- ❌ Less mature than OpenAI ecosystem
- ❌ Fewer third-party integrations

**Configuration**:
```typescript
{
  model: "gemini-pro",
  temperature: 0.7,
  maxOutputTokens: 1000,
  topP: 0.9,
  topK: 40
}
```

---

### 3. Vector Database: Qdrant

**Decision**: Use Qdrant for vector storage and similarity search

**Rationale**:
- **Performance**: Fast similarity search with HNSW algorithm
- **Scalability**: Handles millions of vectors efficiently
- **Filtering**: Metadata filtering for precise retrieval
- **Open-source**: Self-hostable with cloud option
- **Developer experience**: Good documentation and SDKs

**Alternatives Considered**:
- **Pinecone**: Easier setup but vendor lock-in, higher cost
- **Weaviate**: Good features but more complex
- **Chroma**: Simpler but less performant at scale
- **pgvector**: PostgreSQL extension, simpler but slower

**Trade-offs**:
- ✅ Best performance for similarity search
- ✅ Flexible deployment (local, cloud, self-hosted)
- ❌ Requires separate service
- ❌ More complex setup than pgvector

**Configuration**:
```typescript
{
  collection: "book_chunks",
  vectorSize: 768,  // Gemini embedding-001 dimension
  distance: "Cosine",
  onDiskPayload: true
}
```

---

### 4. Primary Database: Neon DB

**Decision**: Use Neon serverless PostgreSQL for relational data

**Rationale**:
- **Serverless**: Auto-scaling, pay-per-use
- **PostgreSQL**: Full SQL support, ACID compliance
- **Developer experience**: Instant provisioning, branching
- **Performance**: Fast queries with connection pooling
- **Cost**: Free tier for development, affordable scaling

**Alternatives Considered**:
- **Supabase**: Similar features but more opinionated
- **PlanetScale**: MySQL-based, different ecosystem
- **MongoDB**: NoSQL, less suitable for relational data
- **Traditional PostgreSQL**: Requires server management

**Trade-offs**:
- ✅ Serverless simplicity
- ✅ PostgreSQL compatibility
- ✅ Excellent free tier
- ❌ Vendor lock-in (but standard PostgreSQL)
- ❌ Cold start latency (minimal)

---

### 5. ORM: Drizzle ORM

**Decision**: Use Drizzle ORM for database access

**Rationale**:
- **Type safety**: Full TypeScript support
- **Performance**: Minimal overhead, close to raw SQL
- **Developer experience**: Intuitive API, great autocomplete
- **Migrations**: Built-in migration system
- **Lightweight**: Small bundle size

**Alternatives Considered**:
- **Prisma**: More features but heavier, slower
- **TypeORM**: Mature but less type-safe
- **Kysely**: Similar philosophy but smaller ecosystem
- **Raw SQL**: Maximum performance but no type safety

**Trade-offs**:
- ✅ Excellent TypeScript support
- ✅ Fast and lightweight
- ✅ SQL-like syntax
- ❌ Smaller ecosystem than Prisma
- ❌ Fewer built-in features

---

### 6. Authentication: Better Auth

**Decision**: Use Better Auth for authentication

**Rationale**:
- **Modern**: Built for modern TypeScript/Node.js apps
- **Flexible**: Supports email/password, OAuth, magic links
- **Type-safe**: Full TypeScript support
- **Lightweight**: Minimal dependencies
- **Customizable**: Easy to extend and customize

**Alternatives Considered**:
- **NextAuth.js**: More mature but Next.js-focused
- **Lucia Auth**: Similar philosophy, smaller ecosystem
- **Auth0**: Feature-rich but expensive, vendor lock-in
- **Custom solution**: Maximum control but time-consuming

**Trade-offs**:
- ✅ Modern, type-safe API
- ✅ Framework-agnostic
- ✅ Good documentation
- ❌ Newer, smaller community
- ❌ Fewer third-party integrations

---

## Architecture Decisions

### 1. Chunking Strategy

**Decision**: Use 500-1000 token chunks with 100-token overlap

**Rationale**:
- **Context preservation**: Overlap prevents information loss at boundaries
- **Retrieval quality**: Optimal size for semantic similarity
- **Token limits**: Fits well within Gemini context window
- **Performance**: Balance between granularity and speed

**Implementation**:
```typescript
{
  chunkSize: 750,        // tokens
  chunkOverlap: 100,     // tokens
  separator: "\n\n",     // paragraph breaks
  keepSeparator: true
}
```

**Alternatives Considered**:
- **Fixed character length**: Simpler but breaks semantic units
- **Sentence-based**: More semantic but variable size
- **Paragraph-based**: Too large for some content

---

### 2. RAG Pipeline Flow

**Decision**: Hybrid search with re-ranking

**Flow**:
1. User query → Embed query with Gemini embedding-001
2. Vector search in Qdrant → Top 20 candidates
3. Metadata filtering (chapter, section) → Narrow results
4. Re-rank by relevance score → Top 5 chunks
5. Construct prompt with context → Send to Gemini gemini-pro
6. Generate answer → Return with source citations

**Rationale**:
- **Recall**: Vector search finds semantically similar content
- **Precision**: Re-ranking improves relevance
- **Context**: Multiple chunks provide comprehensive context
- **Citations**: Source tracking for transparency

**Configuration**:
```typescript
{
  topK: 20,              // Initial retrieval
  rerankTopK: 5,         // Final context
  minScore: 0.7,         // Relevance threshold
  maxContextTokens: 3000 // Gemini context limit
}
```

---

### 3. Session Management

**Decision**: Cookie-based sessions with 24-hour expiry

**Rationale**:
- **Security**: httpOnly cookies prevent XSS attacks
- **Simplicity**: No need for client-side token management
- **Compatibility**: Works across all browsers
- **Expiry**: 24 hours balances security and UX

**Configuration**:
```typescript
{
  cookieName: "better-auth.session_token",
  httpOnly: true,
  secure: true,          // HTTPS only in production
  sameSite: "lax",       // CSRF protection
  maxAge: 86400          // 24 hours
}
```

**Alternatives Considered**:
- **JWT tokens**: More flexible but client-side storage risks
- **Longer expiry**: Better UX but security risk
- **Refresh tokens**: More complex, not needed for this use case

---

### 4. Chat History Storage

**Decision**: Store all messages in PostgreSQL, not in vector DB

**Rationale**:
- **Relational data**: Conversations have clear relationships (user, messages)
- **ACID compliance**: Ensure data consistency
- **Query flexibility**: Complex queries (filters, pagination)
- **Cost**: PostgreSQL storage cheaper than vector DB

**Schema**:
- `conversations` table: Conversation metadata
- `chat_messages` table: Individual messages with sources

**Alternatives Considered**:
- **Vector DB**: Simpler but not designed for relational data
- **Separate NoSQL DB**: More complex architecture
- **Local storage**: No persistence, no cross-device sync

---

## Performance Optimizations

### 1. Content Ingestion

**Strategy**: Batch processing with progress tracking

**Implementation**:
- Process 10 documents at a time
- Generate embeddings in parallel (5 concurrent requests)
- Batch insert to Qdrant (100 vectors per batch)
- Progress bar for user feedback

**Expected Performance**:
- ~50 documents/minute
- ~500 chunks/minute
- Total ingestion time: ~10 minutes for full book

---

### 2. Vector Search

**Optimization**: HNSW index with metadata filtering

**Configuration**:
```typescript
{
  m: 16,                 // HNSW connections
  efConstruct: 200,      // Build-time accuracy
  ef: 128,               // Search-time accuracy
  quantization: "scalar" // Reduce memory usage
}
```

**Expected Performance**:
- Search latency: < 100ms (p95)
- Recall@5: > 90%
- Memory usage: ~500MB for 10K vectors

---

### 3. Response Caching

**Strategy**: Cache common queries with Redis (optional)

**Implementation**:
- Cache key: Hash of user query
- TTL: 1 hour
- Invalidation: On content updates

**Expected Impact**:
- 50% reduction in Gemini API calls
- 80% faster response for cached queries

---

## Security Considerations

### 1. API Rate Limiting

**Implementation**: Token bucket algorithm

**Limits**:
- Auth endpoints: 5 req/min per IP
- Chat endpoints: 20 req/min per user
- History endpoints: 60 req/min per user

**Rationale**:
- Prevent brute-force attacks
- Protect against DDoS
- Manage API costs

---

### 2. Input Sanitization

**Strategy**: Validate and sanitize all user inputs

**Implementation**:
- Email: Regex validation + normalization
- Password: Length + complexity checks
- Chat messages: Max length + XSS prevention
- SQL: Parameterized queries (Drizzle ORM)

---

### 3. Data Encryption

**Strategy**: Encryption at rest and in transit

**Implementation**:
- **In transit**: HTTPS/TLS 1.2+
- **At rest**: Database encryption (Neon default)
- **Passwords**: bcrypt hashing (10 rounds)
- **Tokens**: Cryptographically secure random

---

## Deployment Strategy

### 1. Platform: Vercel

**Decision**: Deploy to Vercel for both frontend and backend

**Rationale**:
- **Serverless**: Auto-scaling, pay-per-use
- **Edge network**: Fast global delivery
- **Integration**: Seamless with Next.js/Docusaurus
- **Developer experience**: Easy deployment, preview URLs

**Configuration**:
- Frontend: Static site generation
- Backend: Serverless functions (Node.js runtime)
- Environment variables: Vercel dashboard

---

### 2. CI/CD Pipeline

**Strategy**: GitHub Actions for automated testing and deployment

**Workflow**:
1. Push to `main` → Run tests
2. Tests pass → Build application
3. Build succeeds → Deploy to Vercel
4. Deployment complete → Run smoke tests

**Expected Benefits**:
- Faster deployment cycles
- Reduced human error
- Consistent quality

---

## Future Enhancements

### 1. Multimodal RAG (Phase 7)

**Idea**: Support images and diagrams in RAG retrieval

**Implementation**:
- Use Gemini vision model for image embeddings
- Store image vectors in Qdrant
- Return relevant images with text responses

---

### 2. Conversational Memory (Phase 7)

**Idea**: Maintain conversation context across messages

**Implementation**:
- Store conversation history in prompt
- Use sliding window (last 5 messages)
- Implement conversation summarization

---

### 3. Personalized Recommendations (Phase 8)

**Idea**: Recommend relevant chapters based on user history

**Implementation**:
- Track user reading patterns
- Use collaborative filtering
- Display personalized suggestions

---

## References

- [Docusaurus Documentation](https://docusaurus.io)
- [Google Gemini API](https://ai.google.dev)
- [Qdrant Documentation](https://qdrant.tech/documentation)
- [Neon Documentation](https://neon.tech/docs)
- [Drizzle ORM](https://orm.drizzle.team)
- [Better Auth](https://better-auth.vercel.app)
- [RAG Best Practices](https://www.pinecone.io/learn/retrieval-augmented-generation)
