# 🚀 Quick Setup - better-auth + Neon

## 1️⃣ Neon Setup (5 min)
```
1. Visit: https://neon.tech
2. Create project: "book-rag-db"
3. Copy connection string
```

## 2️⃣ Environment (2 min)
```bash
cp .env.local.example .env.local
```

Add to `.env.local`:
```env
DATABASE_URL="postgresql://..." # From Neon
BETTER_AUTH_SECRET="..." # Run: node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"
BETTER_AUTH_URL="http://localhost:3000"
```

## 3️⃣ Database (1 min)
```bash
npm run db:push
```

## 4️⃣ Test (5 min)
```bash
npm start
```

Visit:
- Signup: http://localhost:3000/RAGai/auth/signup
- Login: http://localhost:3000/RAGai/auth/login

## ✅ Done!
Check Neon dashboard → Tables → users (should see your account)

---

**Full Guide**: See `auth_setup_guide.md`
