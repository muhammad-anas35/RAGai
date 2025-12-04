# Signup Failure - Debugging Guide

## Common Issues & Solutions

### Issue 1: Database Connection Failed

**Symptoms:**
- "Signup failed. Please try again."
- Backend logs show: `[DB-CHECK] Database connection failed`

**Solutions:**

1. **Verify .env.local exists in root folder:**
   ```bash
   ls .env.local
   ```
   Should show: `.env.local`

2. **Check DATABASE_URL is valid:**
   ```bash
   grep DATABASE_URL .env.local
   ```
   Should show a Neon PostgreSQL connection string

3. **Test database connection:**
   ```bash
   # With backend running on port 4000:
   curl http://localhost:4000/api/health/db
   ```
   Expected response:
   ```json
   {"status":"ok","database":"connected","timestamp":"2025-12-04T10:00:00.000Z"}
   ```
   If it fails, check your DATABASE_URL

4. **If DATABASE_URL is wrong:**
   - Get correct URL from Neon console
   - Update `.env.local`
   - Restart backend: `cd backend && npm run dev`

---

### Issue 2: Missing Database Tables

**Symptoms:**
- Backend logs show: `relation "users" does not exist`
- Database connection works but signup still fails

**Solutions:**

1. **Push schema to database:**
   ```bash
   npm run db:push
   ```

2. **Expected output:**
   ```
   ✓ Creating schema
   ✓ Creating "public"."users" table
   ✓ Creating "public"."sessions" table
   ✓ Creating "public"."chat_history" table
   ✓ Database migration successful
   ```

3. **Verify tables created:**
   ```bash
   npm run db:studio
   ```
   This opens Drizzle Studio where you can see all tables

---

### Issue 3: BETTER_AUTH_SECRET Not Set

**Symptoms:**
- Backend fails to start
- Error: `BETTER_AUTH_SECRET not set in .env.local`

**Solutions:**

1. **Generate secret (in any terminal):**
   ```bash
   openssl rand -hex 32
   ```

2. **Copy the output and add to .env.local:**
   ```env
   BETTER_AUTH_SECRET="your-generated-secret-here"
   ```

3. **Restart backend:**
   ```bash
   cd backend && npm run dev
   ```

---

### Issue 4: Frontend Can't Reach Backend

**Symptoms:**
- Browser console shows: `CORS error` or `Failed to fetch`
- Network tab shows red/failed request to `http://localhost:4000/api/auth/signup/email`

**Solutions:**

1. **Verify backend is running:**
   ```bash
   curl http://localhost:4000/api/health
   ```
   Should return: `{"status":"ok",...}`

2. **Check frontend CORS configuration in browser:**
   - Open DevTools (F12)
   - Go to Network tab
   - Submit signup form
   - Click on failed request
   - Check Response headers for `Access-Control-Allow-Origin`

3. **Verify frontend fetch URL:**
   - Check `src/pages/auth/signup.tsx`
   - Should use: `http://localhost:4000/api/auth/signup/email`
   - Should include: `credentials: 'include'`

4. **Check .env.local has BETTER_AUTH_URL:**
   ```env
   BETTER_AUTH_URL="http://localhost:3000"
   ```

---

### Issue 5: Environment Variables Not Loading

**Symptoms:**
- Backend doesn't see DATABASE_URL despite it being in .env.local
- Error: `DATABASE_URL not set in .env.local`

**Solutions:**

1. **Verify backend loads from correct path:**
   - Backend should load from: `../.env.local` (root folder)
   - Check `backend/src/server.ts` line 12

2. **Make sure you're running from backend folder:**
   ```bash
   cd backend
   npm run dev
   ```
   NOT:
   ```bash
   npm run dev  # from root (wrong location)
   ```

3. **If still not working, manually verify:**
   ```bash
   # In backend folder
   cat ../env.local
   ```
   Should show your environment variables

---

## Step-by-Step Testing

### Step 1: Verify Environment Setup

```bash
# Check .env.local exists and has required variables
grep -E "DATABASE_URL|BETTER_AUTH_SECRET|BETTER_AUTH_URL" .env.local
```

Expected output:
```
DATABASE_URL=postgresql://...
BETTER_AUTH_SECRET=your-secret
BETTER_AUTH_URL=http://localhost:3000
```

---

### Step 2: Setup Database

```bash
# Push schema to database
npm run db:push
```

Expected output includes:
```
✓ Creating "public"."users" table
✓ Creating "public"."sessions" table
✓ Creating "public"."chat_history" table
```

---

### Step 3: Start Backend

```bash
cd backend
npm run dev
```

Expected output includes:
```
✅ Environment variables loaded
✅ Database connection verified
🚀 Server running at: http://localhost:4000
```

---

### Step 4: Test Database Connection

```bash
curl http://localhost:4000/api/health/db
```

Expected response:
```json
{"status":"ok","database":"connected","timestamp":"..."}
```

---

### Step 5: Start Frontend

```bash
# In a new terminal, from root folder
npm start
```

Expected output:
```
[SUCCESS] The site is running at http://localhost:3000
```

---

### Step 6: Test Signup Through UI

1. Navigate to: `http://localhost:3000`
2. Should redirect to: `http://localhost:3000/auth/login`
3. Click "Sign up"
4. Fill form:
   - Name: `Test User`
   - Email: `test@example.com`
   - Password: `Test@123456`
   - Confirm: `Test@123456`
5. Click "Create Account"

---

### Step 7: Monitor Backend Logs

During signup, you should see in backend terminal:

```
[SIGNUP] Request received: { email: 'test@example.com', name: 'Test User', passwordLength: 12 }
[SIGNUP] Checking if user exists: test@example.com
[SIGNUP] Hashing password
[SIGNUP] Creating user with ID: abc123def456...
[SIGNUP] User created successfully
[SIGNUP] Creating session
[SIGNUP] Session created
[SIGNUP] Cookie set, sending success response
```

---

## Complete Checklist

- [ ] `.env.local` file exists in root folder
- [ ] `DATABASE_URL` is set to valid Neon connection string
- [ ] `BETTER_AUTH_SECRET` is set (32 hex characters)
- [ ] `BETTER_AUTH_URL=http://localhost:3000` is set
- [ ] Database tables created: `npm run db:push` successful
- [ ] Backend starts without errors: `cd backend && npm run dev`
- [ ] `curl http://localhost:4000/api/health/db` returns success
- [ ] Frontend starts: `npm start`
- [ ] Frontend redirects to `/auth/login` on base URL
- [ ] Signup form displays correctly
- [ ] Frontend can reach backend (no CORS errors)
- [ ] Database receives signup data

---

## Actual Error Messages & Fixes

### "Signup failed. Please try again." (with no backend logs)
- **Cause:** Frontend can't reach backend (CORS/network issue)
- **Fix:** Check DevTools Network tab, verify `http://localhost:4000` is reachable

### "[SIGNUP ERROR] relation 'users' does not exist"
- **Cause:** Database tables not created
- **Fix:** Run `npm run db:push`

### "[SIGNUP ERROR] ECONNREFUSED 127.0.0.1:5432"
- **Cause:** Can't connect to database server
- **Fix:** Verify DATABASE_URL is correct Neon URL (not localhost)

### "[LOGIN] Invalid password" (after successful signup)
- **Cause:** Password hashing mismatch
- **Fix:** Verify `BETTER_AUTH_SECRET` is same at signup and login

### "Unexpected token < in JSON at position 0"
- **Cause:** Backend returning HTML error page instead of JSON
- **Fix:** Check backend logs for actual error, likely missing tables

---

## Need More Help?

1. **Check backend logs carefully** - they now show exactly what fails
2. **Verify DATABASE_URL works** - test with `npm run db:studio`
3. **Test each endpoint** - use curl to isolate issues:
   ```bash
   curl -X POST http://localhost:4000/api/auth/signup/email \
     -H "Content-Type: application/json" \
     -d '{"email":"test@example.com","password":"Test@123456","name":"Test"}'
   ```
4. **Check browser DevTools** - Network and Console tabs show frontend errors

---

**Updated:** December 4, 2025  
**Status:** Complete diagnostic guide included
