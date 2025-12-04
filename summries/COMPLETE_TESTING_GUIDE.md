# Complete Setup & Testing Guide

## 🚀 Quick Start (5 Minutes)

### Step 1: Setup Environment

**Windows (Command Prompt):**
```cmd
REM Generate auth secret
openssl rand -hex 32
REM Copy the output

REM Create .env.local in root folder
notepad .env.local
```

**Add this to `.env.local`:**
```env
# Neon Database - Get this from https://console.neon.tech
DATABASE_URL="postgresql://user:password@host/dbname?sslmode=require&channel_binding=require"

# Auth Secret - Use generated value above
BETTER_AUTH_SECRET="your-generated-32-character-hex-secret"

# Frontend URL
BETTER_AUTH_URL="http://localhost:3000"

# Optional
GEMINI_API_KEY="your-key-if-you-have"
```

---

### Step 2: Database Setup

```bash
# Create database tables
npm run db:push
```

Expected output:
```
✓ Creating schema
✓ Creating "public"."users" table
✓ Creating "public"."sessions" table
✓ Creating "public"."chat_history" table
```

---

### Step 3: Start Backend (Terminal 1)

```bash
cd backend
npm run dev
```

Expected output:
```
✅ Environment variables loaded
✅ Database connection verified
🚀 Server running at: http://localhost:4000
```

---

### Step 4: Start Frontend (Terminal 2)

```bash
npm start
```

Expected output:
```
[SUCCESS] The site is running at http://localhost:3000
```

---

### Step 5: Test Authentication

1. **Open browser:** http://localhost:3000
   - ✅ Should auto-redirect to `/auth/login`

2. **Click "Sign up"**

3. **Fill form:**
   - Name: `Test User`
   - Email: `test@example.com`
   - Password: `TestPass123`
   - Confirm: `TestPass123`

4. **Click "Create Account"**
   - ✅ Should create user
   - ✅ Should redirect to login

5. **Login with same credentials**
   - ✅ Should set session cookie
   - ✅ Should redirect to home

---

## 🔍 Deep Functionality Check

### Test 1: Database Connection

```bash
curl http://localhost:4000/api/health/db
```

**Success response:**
```json
{"status":"ok","database":"connected","timestamp":"2025-12-04T10:00:00.000Z"}
```

**Failure response:**
```json
{
  "status":"error",
  "database":"disconnected",
  "error":"connect ECONNREFUSED",
  "message":"Cannot connect to database. Check DATABASE_URL in .env.local"
}
```

---

### Test 2: Server Health

```bash
curl http://localhost:4000/api/health
```

**Response:**
```json
{
  "status":"ok",
  "timestamp":"2025-12-04T10:00:00.000Z",
  "authenticated":false
}
```

---

### Test 3: Signup via API

**Using cURL:**
```bash
curl -X POST http://localhost:4000/api/auth/signup/email \
  -H "Content-Type: application/json" \
  -d '{
    "email":"api-test@example.com",
    "password":"TestPass123",
    "name":"API Test User"
  }'
```

**Success response (201):**
```json
{
  "success":true,
  "message":"Account created successfully",
  "user":{
    "id":"abc123...",
    "email":"api-test@example.com",
    "name":"API Test User"
  },
  "redirect":"/"
}
```

**Failure responses:**

- **400 - Missing fields:**
```json
{"error":"Email and password required"}
```

- **409 - User exists:**
```json
{"error":"User already exists"}
```

- **500 - Database error:**
```json
{
  "error":"Signup failed. Please try again.",
  "details":"relation 'users' does not exist"
}
```

---

### Test 4: Login via API

```bash
curl -X POST http://localhost:4000/api/auth/signin/email \
  -H "Content-Type: application/json" \
  -c cookies.txt \
  -d '{
    "email":"api-test@example.com",
    "password":"TestPass123"
  }'
```

**Success response (200):**
```json
{
  "success":true,
  "message":"Logged in successfully",
  "user":{
    "id":"abc123...",
    "email":"api-test@example.com",
    "name":"API Test User"
  },
  "redirect":"/"
}
```

**Check set-cookie header:**
```
Set-Cookie: better-auth.session_token=...; Path=/; HttpOnly; SameSite=Lax; Max-Age=604800
```

---

### Test 5: Protected Route (Dashboard)

**Without login:**
```bash
curl http://localhost:4000/api/dashboard
```

**Response (401):**
```json
{
  "error":"Unauthorized",
  "message":"Please login to access this resource"
}
```

**With session cookie:**
```bash
curl -b "better-auth.session_token=your-token" \
     http://localhost:4000/api/dashboard
```

**Response (200):**
```json
{
  "success":true,
  "message":"Dashboard data",
  "data":{}
}
```

---

### Test 6: Check Authentication Status

```bash
curl http://localhost:4000/api/auth/check
```

**Response (when not logged in):**
```json
{"authenticated":false}
```

**Response (when logged in):**
```json
{"authenticated":true}
```

---

## 📊 Backend Logs - What to Look For

### Successful Signup Log:

```
[SIGNUP] Request received: { email: 'user@example.com', name: 'User', passwordLength: 10 }
[SIGNUP] Checking if user exists: user@example.com
[SIGNUP] Hashing password
[SIGNUP] Creating user with ID: abc123def456
[SIGNUP] User created successfully
[SIGNUP] Creating session
[SIGNUP] Session created
[SIGNUP] Cookie set, sending success response
```

### Successful Login Log:

```
[LOGIN] Request received: { email: 'user@example.com' }
[LOGIN] Finding user: user@example.com
[LOGIN] User found, verifying password
[LOGIN] Password verified, creating session
[LOGIN] Session created, setting cookie
[LOGIN] Success for: user@example.com
```

### Signup Error - User Exists:

```
[SIGNUP] Request received: { email: 'user@example.com', name: 'User', passwordLength: 10 }
[SIGNUP] Checking if user exists: user@example.com
[SIGNUP] User already exists: user@example.com
```

### Signup Error - Database Issue:

```
[SIGNUP ERROR] {
  message: 'relation "users" does not exist',
  code: '42P01',
  stack: '...'
}
```
**Fix:** Run `npm run db:push`

### Signup Error - Invalid Email:

```
[SIGNUP] Invalid email format
```

### Signup Error - Password Too Short:

```
[SIGNUP] Password too short
```

---

## 🔐 Security Verification

### Check 1: Session Cookie is HTTPOnly

1. **Open DevTools** (F12)
2. Go to **Application** tab
3. Click **Cookies** → `localhost:3000`
4. Find `better-auth.session_token`
5. ✅ **HttpOnly** checkbox should be checked
6. ✅ **Secure** checkbox (checked in production)

### Check 2: CORS Working

1. **Open DevTools** Console
2. Attempt login
3. Should **NOT** see CORS errors
4. If you see `Access to XMLHttpRequest blocked by CORS policy`, check:
   - Backend `BETTER_AUTH_URL` setting
   - Frontend fetch has `credentials: 'include'`
   - Backend CORS middleware

### Check 3: Password Hashing

1. **Login to database** (via Neon console)
2. **Query users table:**
   ```sql
   SELECT email, password_hash FROM users LIMIT 1;
   ```
3. ✅ Password should be 64-character hex string (SHA256)
4. ❌ Should NOT see plain text password

---

## ❌ Common Errors & Fixes

| Error | Cause | Fix |
|-------|-------|-----|
| "Signup failed. Please try again." | Frontend can't reach backend | Check DevTools Network tab, verify backend is running |
| "relation 'users' does not exist" | Tables not created | Run `npm run db:push` |
| "DATABASE_URL not set" | Missing .env.local | Create .env.local with DATABASE_URL |
| "BETTER_AUTH_SECRET not set" | Missing .env.local | Run `openssl rand -hex 32` and add to .env.local |
| CORS error in browser | Backend CORS not configured | Check BETTER_AUTH_URL in .env.local matches frontend URL |
| "Invalid email or password" | Wrong credentials | Verify user exists and password is correct |
| Cookie not being set | `credentials: 'include'` missing | Check signup.tsx and login.tsx have fetch with credentials |
| Session expires immediately | Max age misconfigured | Check backend server.ts maxAge setting (should be 7 days) |

---

## ✅ Final Verification Checklist

- [ ] `.env.local` created in root with all 3 required variables
- [ ] `npm run db:push` executed successfully
- [ ] Backend starts without errors on port 4000
- [ ] Database health check passes: `curl http://localhost:4000/api/health/db`
- [ ] Frontend starts without errors on port 3000
- [ ] Base URL redirects to login: `http://localhost:3000` → `http://localhost:3000/auth/login`
- [ ] Signup form displays correctly
- [ ] Signup creates user in database
- [ ] Login accepts correct credentials
- [ ] Session cookie is set after login
- [ ] Cookie is HTTPOnly (checked in DevTools)
- [ ] Protected routes redirect to login when not authenticated
- [ ] No CORS errors in DevTools console
- [ ] Backend logs show [SIGNUP] and [LOGIN] messages during testing

---

## 📞 Need Help?

1. **Check DEBUG_SIGNUP.md** - Detailed diagnosis guide
2. **Look at backend logs** - They now show exactly what fails
3. **Use verification script:** `verify-setup.bat` (Windows) or `verify-setup.sh` (Mac/Linux)
4. **Test each endpoint** with cURL to isolate issues
5. **Check browser DevTools** - Network and Console tabs

---

**Last Updated:** December 4, 2025  
**System Status:** ✅ Ready for Deep Testing
