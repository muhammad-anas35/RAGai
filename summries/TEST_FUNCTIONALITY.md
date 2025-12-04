# Authentication System - Testing Guide

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Frontend (Port 3000)                  │
│              Docusaurus + React + TypeScript            │
│  ┌─────────────────────────────────────────────────────┐
│  │ Base URL (/) → Redirects to /auth/login             │
│  │ /auth/login   → Login page                          │
│  │ /auth/signup  → Sign up page                        │
│  │ /docs/*       → Protected content (requires login)  │
│  └─────────────────────────────────────────────────────┘
│                         ↕️ (HTTP Requests + Cookies)
└─────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────┐
│                  Backend (Port 4000)                     │
│              Express.js + PostgreSQL + Drizzle         │
│  ┌─────────────────────────────────────────────────────┐
│  │ POST   /api/auth/signup/email   → Create account   │
│  │ POST   /api/auth/signin/email   → Login            │
│  │ POST   /api/auth/signout        → Logout           │
│  │ GET    /api/auth/check          → Check auth       │
│  │ GET    /api/health              → Server health    │
│  │ GET    /api/dashboard (🔐)      → Protected        │
│  │ POST   /api/chat (🔐)           → Protected        │
│  └─────────────────────────────────────────────────────┘
│                         ↕️ (Database Operations)
└─────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────┐
│              Neon PostgreSQL Database                    │
│  ┌─────────────────────────────────────────────────────┐
│  │ TABLE: users                                        │
│  │   - id (UUID)                                       │
│  │   - email (unique)                                  │
│  │   - passwordHash (hashed)                           │
│  │   - name                                            │
│  │                                                      │
│  │ TABLE: sessions                                     │
│  │   - id (token)                                      │
│  │   - userId (FK → users)                             │
│  │   - expiresAt (7 days)                              │
│  │                                                      │
│  │ TABLE: chat_history                                 │
│  │   - id                                              │
│  │   - userId (FK → users)                             │
│  │   - message                                         │
│  │   - response                                        │
│  └─────────────────────────────────────────────────────┘
└─────────────────────────────────────────────────────────┘
```

---

## Prerequisites

### 1. Environment Setup

Create `.env.local` in the root directory with:

```env
# Neon Database - UPDATE with your actual DATABASE_URL
DATABASE_URL="postgresql://user:password@host/dbname?sslmode=require&channel_binding=require"

# Better-Auth Secret - Generate with: openssl rand -hex 32
BETTER_AUTH_SECRET="your-generated-secret-here"

# Frontend URL (for CORS)
BETTER_AUTH_URL="http://localhost:3000"

# Optional - API Keys for future features
GEMINI_API_KEY="your-gemini-key"
QDRANT_URL="your-qdrant-url"
QDRANT_API_KEY="your-qdrant-key"
```

### 2. Generate BETTER_AUTH_SECRET

Run in terminal:
```bash
openssl rand -hex 32
```

Copy the output and paste into `.env.local`

### 3. Database Setup

```bash
# Install dependencies if not done
npm install
cd backend && npm install && cd ..

# Push schema to Neon
npm run db:push
```

---

## Test Procedure

### Phase 1: Start Services

**Terminal 1 - Start Backend:**
```bash
cd backend
npm run dev
```

Expected output:
```
╔═══════════════════════════════════════╗
║    📚 Book RAG Backend Server         ║
║         (Authentication API)          ║
╚═══════════════════════════════════════╝

🚀 Server running at: http://localhost:4000
📡 Frontend expects: http://localhost:3000
🗄️  Database: ✅ Connected

Available Endpoints:
  PUBLIC:
  ✅ GET    /api/health               - Health check
  ✅ GET    /api/auth/check           - Check auth status
  ✅ POST   /api/auth/signup/email    - Create account
  ✅ POST   /api/auth/signin/email    - Login
  
  PROTECTED (require login):
  🔐 POST   /api/auth/signout         - Logout
  🔐 GET    /api/auth/session         - Get session info
  🔐 GET    /api/auth/me              - Get user info
  🔐 GET    /api/dashboard            - User dashboard
  🔐 POST   /api/chat                 - Send message
  🔐 GET    /api/chat/history         - Get chat history
```

**Terminal 2 - Start Frontend:**
```bash
npm start
```

Expected output:
```
[SUCCESS] The site is running at http://localhost:3000
```

---

### Phase 2: Test Sign Up Flow

1. **Navigate to:** http://localhost:3000
   - ✅ Should automatically redirect to `/auth/login`

2. **Click "Sign up" link**
   - ✅ Should navigate to `/auth/signup`

3. **Fill signup form:**
   - Name: `Test User`
   - Email: `test@example.com`
   - Password: `TestPassword123`
   - Confirm Password: `TestPassword123`

4. **Submit form**
   - ✅ Should show "Creating account..." loading state
   - ✅ Should create user in database
   - ✅ Should redirect to `/auth/login?registered=true`
   - ❌ If error: Check DATABASE_URL and backend logs

---

### Phase 3: Test Login Flow

1. **On login page, enter:**
   - Email: `test@example.com`
   - Password: `TestPassword123`

2. **Click "Login"**
   - ✅ Should show "Logging in..." loading state
   - ✅ Should validate credentials against database
   - ✅ Should create session token
   - ✅ Should set HTTPOnly cookie: `better-auth.session_token`
   - ✅ Should redirect to home page (`/`)

3. **After successful login:**
   - ✅ Cookie should be visible in DevTools → Application → Cookies
   - ✅ Cookie name: `better-auth.session_token`
   - ✅ HTTPOnly flag: ✓ (secure)
   - ✅ Path: `/`

---

### Phase 4: Test Protected Routes

1. **After logging in, try to access:**
   - http://localhost:3000/docs/intro
   - ✅ Should load successfully (user is authenticated)

2. **In another private window/tab (not logged in):**
   - Visit: http://localhost:3000/docs/intro
   - ✅ Should redirect to `/auth/login` (unauthenticated access blocked)

---

### Phase 5: Test API Endpoints

**Test 1 - Health Check (Public)**
```bash
curl http://localhost:4000/api/health
```
Expected: `{ "status": "ok" }`

**Test 2 - Check Auth Status (Public, no login needed)**
```bash
curl http://localhost:4000/api/auth/check
```
Expected: `{ "authenticated": false }`

**Test 3 - Get User Info (Protected, requires login)**
```bash
# Won't work without session cookie
curl http://localhost:4000/api/auth/me
```
Expected: `{ "error": "Unauthorized", "message": "Please login to access this resource" }`

**Test 4 - Dashboard Access (Protected)**
```bash
curl http://localhost:4000/api/dashboard
```
Expected: `{ "error": "Unauthorized", "message": "Please login to access this resource" }`

---

### Phase 6: Test Logout

1. **After login, logout from the UI (when implemented)**
   - ✅ Should clear session cookie
   - ✅ Should redirect to `/auth/login`

2. **Verify cookie is cleared:**
   - DevTools → Application → Cookies
   - ✅ `better-auth.session_token` should be gone

---

## Testing Checklist

### Frontend Pages

- [ ] Base URL (`/`) redirects to `/auth/login`
- [ ] `/auth/login` page displays correctly
- [ ] `/auth/signup` page displays correctly
- [ ] Form styling is professional (dark mode + light mode)
- [ ] Error messages display properly

### Authentication Flow

- [ ] Signup creates user in database
- [ ] Signup redirects to login after success
- [ ] Login validates credentials
- [ ] Login creates session and sets cookie
- [ ] Successful login redirects to home
- [ ] Failed login shows error message

### Security

- [ ] Session cookie is HTTPOnly (immune to XSS)
- [ ] Session cookie has Secure flag (HTTPS in production)
- [ ] CORS properly configured (frontend can reach backend)
- [ ] Protected routes reject unauthenticated requests (401)
- [ ] Password hashing works (check database, passwords aren't plain text)

### Database

- [ ] Users table has data after signup
- [ ] Sessions table has entries after login
- [ ] Password field is hashed (not plain text)
- [ ] Session expires after 7 days
- [ ] Foreign keys working (sessions.userId → users.id)

### API Endpoints

- [ ] `GET /api/health` returns 200
- [ ] `GET /api/auth/check` returns auth status
- [ ] `POST /api/auth/signup/email` creates user
- [ ] `POST /api/auth/signin/email` authenticates
- [ ] `GET /api/dashboard` requires auth (401 without login)
- [ ] Protected routes work after login

---

## Debugging Guide

### Issue: "Database connection failed"
- Check `DATABASE_URL` in `.env.local`
- Verify Neon database is running
- Run: `npm run db:push` to create tables

### Issue: "Signup creates user but won't login"
- Check password hashing logic
- Verify password is being hashed in signup
- Check if passwords match in login verification

### Issue: "CORS errors in browser console"
- Check `BETTER_AUTH_URL` matches frontend URL
- Verify backend CORS middleware has `credentials: true`
- Verify frontend fetch includes `credentials: 'include'`

### Issue: "Session cookie not being set"
- Check backend response includes `Set-Cookie` header
- Verify cookies aren't being cleared by browser security policy
- Check if frontend properly handles cookie-based sessions

### Issue: "Can't access protected pages after login"
- Verify session cookie is present in DevTools
- Check cookie name: `better-auth.session_token`
- Verify backend `requireAuth` middleware is working
- Check if middleware correctly reads cookie

### Issue: "TypeScript errors in backend"
- Run: `npm run type-check` in backend folder
- Ensure all types are properly imported
- Check if database schema matches Drizzle types

---

## Success Criteria

✅ **Full Functionality Achieved When:**

1. ✅ Frontend loads and redirects to login
2. ✅ User can sign up with email/password
3. ✅ User can login with credentials
4. ✅ Session cookie is set after successful login
5. ✅ Protected pages require login (redirect if not authenticated)
6. ✅ User can access content after login
7. ✅ Logout clears session and redirects to login
8. ✅ All API endpoints respond correctly
9. ✅ Database stores users and sessions properly
10. ✅ No XSS vulnerabilities (HTTPOnly cookies)

---

## Next Steps (After Core Auth Works)

1. **Add logout button** to homepage/dashboard
2. **Add password reset** functionality
3. **Implement OAuth** (Google, GitHub)
4. **Add email verification**
5. **Implement RAG chatbot** with Gemini + Qdrant
6. **Add user profile page**
7. **Deploy to production** (update BETTER_AUTH_URL)

---

**Last Updated:** December 4, 2025  
**Status:** ✅ Ready for testing
