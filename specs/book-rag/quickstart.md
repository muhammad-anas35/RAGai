# Quick Start Guide - Book RAG Project

## For Developers

### Prerequisites
- Node.js >= 20.0
- npm or yarn
- Git
- Neon DB account (free tier)
- Google AI API key (for Gemini)
- Qdrant instance (local or cloud)

---

## Local Development Setup

### 1. Clone Repository
```bash
git clone https://github.com/yourusername/book-rag.git
cd book-rag
```

### 2. Install Dependencies
```bash
# Install root dependencies
npm install

# Install backend dependencies
cd backend
npm install
cd ..
```

### 3. Environment Configuration

Create `.env` file in root:
```env
# Neon Database
DATABASE_URL=postgresql://user:password@host/database

# Google Gemini API
GEMINI_API_KEY=your_gemini_api_key_here

# Qdrant Vector Database
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your_qdrant_api_key_here
```

Create `backend/.env` file:
```env
# Database
DATABASE_URL=postgresql://user:password@host/database

# Authentication
BETTER_AUTH_SECRET=your_32_character_hex_secret_here
BETTER_AUTH_URL=http://localhost:4000

# Server
PORT=4000

# AI Services
GEMINI_API_KEY=your_gemini_api_key_here

# Vector Database
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your_qdrant_api_key_here
```

### 4. Database Setup
```bash
# Generate database schema
npm run db:generate

# Push schema to Neon
npm run db:push

# (Optional) Open Drizzle Studio
npm run db:studio
```

### 5. Start Development Servers

**Option A: Run separately**
```bash
# Terminal 1: Frontend (Docusaurus)
npm start
# Runs on http://localhost:3000

# Terminal 2: Backend (Express)
cd backend
npm run dev
# Runs on http://localhost:4000
```

**Option B: Run together**
```bash
npm run dev:all
# Frontend: http://localhost:3000
# Backend: http://localhost:4000
```

### 6. Verify Setup
```bash
# Check backend health
curl http://localhost:4000/api/health

# Expected response:
# {"status":"ok","timestamp":"2025-12-04T..."}
```

---

## Quick Testing

### Test Authentication
```bash
# Signup
curl -X POST http://localhost:4000/api/auth/signup/email \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"Test1234!","name":"Test User"}'

# Login
curl -X POST http://localhost:4000/api/auth/signin/email \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"Test1234!"}'

# Get session
curl http://localhost:4000/api/auth/session \
  -H "Cookie: better-auth.session_token=YOUR_TOKEN_HERE"
```

---

## Content Ingestion (Phase 3)

### 1. Prepare Content
Ensure all book content is in `docs/physical-ai-book/` directory as `.md` files.

### 2. Run Ingestion Script
```bash
cd backend
npm run ingest

# Expected output:
# ✓ Reading documents...
# ✓ Chunking content...
# ✓ Generating embeddings...
# ✓ Storing in Qdrant...
# ✓ Complete! Processed 50 documents, 500 chunks
```

### 3. Verify Ingestion
```bash
# Check Qdrant collection
curl http://localhost:6333/collections/book_chunks

# Expected response:
# {"result":{"vectors_count":500,...}}
```

---

## Common Tasks

### Add New Content
1. Create `.md` file in `docs/physical-ai-book/`
2. Update `sidebars.ts` if needed
3. Run ingestion script to update vector DB
4. Restart frontend to see changes

### Update Database Schema
```bash
# 1. Edit schema files in src/db/schema/
# 2. Generate migration
npm run db:generate

# 3. Review migration in drizzle/ directory
# 4. Apply migration
npm run db:push
```

### Run Tests
```bash
# Backend tests
cd backend
npm test

# Frontend tests
npm test

# All tests
npm run test:all
```

### Build for Production
```bash
# Build frontend
npm run build

# Build backend
cd backend
npm run build

# Test production build
npm run serve
```

---

## Troubleshooting

### Port Already in Use
```bash
# Kill process on port 3000
npx kill-port 3000

# Kill process on port 4000
npx kill-port 4000
```

### Database Connection Error
1. Check `DATABASE_URL` in `.env`
2. Verify Neon DB is accessible
3. Check firewall/network settings
4. Test connection with `psql` or database client

### Qdrant Connection Error
1. Check if Qdrant is running: `curl http://localhost:6333`
2. Start local Qdrant: `docker run -p 6333:6333 qdrant/qdrant`
3. Or use Qdrant Cloud and update `QDRANT_URL`

### Gemini API Error
1. Verify `GEMINI_API_KEY` is correct
2. Check API quota/limits in Google AI Studio
3. Ensure billing is enabled (if required)

### Build Errors
```bash
# Clear cache and rebuild
npm run clear
npm install
npm run build
```

---

## Integration Scenarios

### Scenario 1: Add OAuth Provider

**Goal**: Add Google OAuth login

**Steps**:
1. Create OAuth app in Google Cloud Console
2. Get client ID and secret
3. Add to `backend/.env`:
   ```env
   GOOGLE_CLIENT_ID=your_client_id
   GOOGLE_CLIENT_SECRET=your_client_secret
   ```
4. Update Better Auth configuration in `backend/src/server.ts`
5. Add OAuth button to frontend login page
6. Test OAuth flow

**Expected Result**: Users can login with Google account

---

### Scenario 2: Customize Chat UI

**Goal**: Change chat widget appearance

**Steps**:
1. Edit `src/components/Chat/ChatWidget.tsx`
2. Update styles in `src/css/chat.css`
3. Modify colors, fonts, layout as needed
4. Test responsiveness on mobile/tablet
5. Verify accessibility (keyboard navigation, screen readers)

**Expected Result**: Chat widget matches brand design

---

### Scenario 3: Add New API Endpoint

**Goal**: Create endpoint to get user statistics

**Steps**:
1. Create route file: `backend/src/routes/stats.ts`
2. Define endpoint logic (query database)
3. Add authentication middleware
4. Register route in `backend/src/server.ts`
5. Add TypeScript types for request/response
6. Write unit tests
7. Update API documentation in `specs/book-rag/contracts/`

**Expected Result**: `/api/stats` returns user statistics

---

### Scenario 4: Deploy to Production

**Goal**: Deploy application to Vercel

**Steps**:
1. Create Vercel account and link GitHub repo
2. Configure environment variables in Vercel dashboard
3. Set build command: `npm run build`
4. Set output directory: `build`
5. Deploy backend as serverless functions
6. Update `BETTER_AUTH_URL` to production URL
7. Test deployment with smoke tests
8. Monitor logs and errors

**Expected Result**: Application accessible at `https://your-app.vercel.app`

---

## Development Workflow

### Feature Development
1. Create feature branch: `git checkout -b feature/chat-history`
2. Implement feature following tasks.md
3. Write tests (unit + integration)
4. Run linter: `npm run lint`
5. Run tests: `npm test`
6. Commit changes: `git commit -m "feat: add chat history"`
7. Push and create PR: `git push origin feature/chat-history`
8. Wait for CI/CD checks to pass
9. Request code review
10. Merge to main after approval

### Bug Fixes
1. Create bug branch: `git checkout -b fix/login-error`
2. Write failing test that reproduces bug
3. Fix the bug
4. Verify test passes
5. Commit and push
6. Create PR with bug description

---

## Useful Commands

### Database
```bash
npm run db:generate    # Generate migrations
npm run db:push        # Apply migrations
npm run db:studio      # Open Drizzle Studio
```

### Development
```bash
npm start              # Start frontend
npm run dev            # Start with hot reload
npm run build          # Build for production
npm run serve          # Serve production build
npm run clear          # Clear cache
```

### Backend
```bash
cd backend
npm run dev            # Start with hot reload
npm run build          # Compile TypeScript
npm start              # Start production server
npm test               # Run tests
npm run type-check     # Check TypeScript types
```

### Code Quality
```bash
npm run lint           # Run ESLint
npm run format         # Run Prettier
npm test               # Run all tests
npm run test:watch     # Run tests in watch mode
```

---

## Next Steps

1. **Phase 3**: Implement content ingestion pipeline
2. **Phase 4**: Build RAG chat functionality
3. **Phase 5**: Add user preferences and chat history
4. **Phase 6**: Deploy to production

Refer to `specs/book-rag/tasks.md` for detailed task breakdown.

---

## Resources

- [Project Documentation](./README.md)
- [API Contracts](./specs/book-rag/contracts/api.md)
- [Data Model](./specs/book-rag/data-model.md)
- [Implementation Plan](./specs/book-rag/plan.md)
- [Docusaurus Docs](https://docusaurus.io)
- [Better Auth Docs](https://better-auth.vercel.app)
- [Drizzle ORM Docs](https://orm.drizzle.team)

---

## Support

- **Issues**: Create GitHub issue with bug/feature request
- **Questions**: Check documentation or ask in discussions
- **Security**: Email security@example.com for security issues
