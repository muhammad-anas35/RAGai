# ⚡ QUICK REFERENCE - GET STARTED IN 10 MINUTES

## 🎯 Current Status: 95% Done - Just Need Setup!

---

## ✅ What's Ready
- ✅ Frontend pages (login, signup)
- ✅ Backend API (11 endpoints)
- ✅ Database schema
- ✅ Authentication logic
- ✅ All documentation

## ❌ What's Missing
- ❌ `.env.local` file
- ❌ Database tables
- ❌ Services running

---

## 🚀 3-STEP QUICK START

### Step 1: Environment (2 min)
```bash
# Generate secret
openssl rand -hex 32

# Create .env.local in root with:
DATABASE_URL="postgresql://user@host/db?sslmode=require&channel_binding=require"
BETTER_AUTH_SECRET="your-generated-32-char-hex"
BETTER_AUTH_URL="http://localhost:3000"
```

### Step 2: Database (2 min)
```bash
npm run db:push
```

### Step 3: Start Services (2 min)
```bash
# Terminal 1
cd backend && npm run dev

# Terminal 2
npm start
```

### Step 4: Test (1 min)
```
Visit: http://localhost:3000
Try signup/login
```

---

## 🔗 Important URLs

| Service | URL | Status |
|---------|-----|--------|
| Frontend | http://localhost:3000 | ⏳ Not running |
| Backend | http://localhost:4000 | ⏳ Not running |
| Database | Neon console | ✅ Ready |

---

## 📝 API Quick Reference

### Signup
```bash
curl -X POST http://localhost:4000/api/auth/signup/email \
  -H "Content-Type: application/json" \
  -d '{"email":"user@test.com","password":"Test@123","name":"User"}'
```

### Login
```bash
curl -X POST http://localhost:4000/api/auth/signin/email \
  -H "Content-Type: application/json" \
  -d '{"email":"user@test.com","password":"Test@123"}'
```

### Check Health
```bash
curl http://localhost:4000/api/health
curl http://localhost:4000/api/health/db
```

---

## 🐛 If Something Breaks

| Error | Fix |
|-------|-----|
| "DATABASE_URL not set" | Create .env.local with DATABASE_URL |
| "relation 'users' does not exist" | Run npm run db:push |
| "Can't connect to backend" | Verify backend is running on 4000 |
| "CORS error in browser" | Check BETTER_AUTH_URL in .env.local |
| "Signup failed" | Check backend logs for [SIGNUP ERROR] |

---

## 📚 Full Guides Available

- **COMPLETE_TESTING_GUIDE.md** - Full walkthrough
- **DEBUG_SIGNUP.md** - Troubleshooting
- **FUNCTIONALITY_CHECK.md** - Status report
- **STATUS_VISUAL.md** - Visual breakdown

---

## ✨ What to Expect After Setup

✅ Base URL redirects to login  
✅ Signup creates user in database  
✅ Login authenticates user  
✅ Session cookie set (HTTPOnly)  
✅ Protected pages work after login  
✅ All API endpoints respond  
✅ Proper error messages shown  
✅ Backend logs show [SIGNUP]/[LOGIN] tags  

---

## 🎓 Architecture

```
Browser (3000)  ←→  Backend (4000)  ←→  Database (Neon)
  Signup form       API Endpoints       PostgreSQL
  Login form        Auth Logic          Users table
  Protected pages   Error Handling      Sessions table
```

---

## 📊 System Components

| Component | Type | Status |
|-----------|------|--------|
| Frontend | React + Docusaurus | ✅ Ready |
| Backend | Express.js | ✅ Ready |
| Database | PostgreSQL (Neon) | ⚠️ Setup needed |
| Auth | Email/Password | ✅ Ready |
| Cookies | HTTPOnly | ✅ Ready |

---

## 🎯 Success Checklist

- [ ] `.env.local` created
- [ ] `DATABASE_URL` added
- [ ] `BETTER_AUTH_SECRET` generated
- [ ] `npm run db:push` successful
- [ ] Backend started (see [SIGNUP] in logs)
- [ ] Frontend started (http://localhost:3000 works)
- [ ] Signup page loads
- [ ] Can create account
- [ ] Can login
- [ ] Session cookie appears in DevTools

**All checked = System works!** ✅

---

## 📞 Support Resources

1. **Setup stuck?** → See COMPLETE_TESTING_GUIDE.md
2. **Signup failing?** → See DEBUG_SIGNUP.md
3. **Want details?** → See FUNCTIONALITY_CHECK.md
4. **Visual overview?** → See STATUS_VISUAL.md
5. **Quick reference?** → You're reading it!

---

## ⏱️ Time Estimate

- Setup .env.local: **2 min**
- Database setup: **2 min**
- Start services: **2 min**
- First test: **1 min**
- Full testing: **3-5 min**

**Total: ~10 minutes** ⏱️

---

## 🚀 Ready? GO!

1. Open 2 terminals
2. Create .env.local
3. Run: `npm run db:push`
4. Terminal 1: `cd backend && npm run dev`
5. Terminal 2: `npm start`
6. Visit: http://localhost:3000

**LET'S GO!** 🎉

---

**Last Updated:** December 4, 2025  
**System Status:** ✅ 95% Ready - Just Setup Needed!
