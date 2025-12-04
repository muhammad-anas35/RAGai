# Authentication Setup Guide

> **Using**: @authentication_agent + @neon-database skill

## Step 1: Create Neon Database

Following **@neon-database skill** workflow:

1. **Sign up at Neon**:
   ```
   https://neon.tech
   ```

2. **Create Project**:
   - Click "New Project"
   - Name: `book-rag-db`
   - Region: Choose closest to you (e.g., `us-east-2`)

3. **Copy Connection String**:
   - Go to Dashboard → Connection Details
   - Copy the connection string
   - Format: `postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require`

## Step 2: Configure Environment

Create `.env.local` file:

```bash
# Copy template
cp .env.local.example .env.local
```

Fill in values:

```env
# Neon Database (from Step 1)
DATABASE_URL="postgresql://your-connection-string-here"

# Better-Auth Secret (generate new one)
# Run: node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"
BETTER_AUTH_SECRET="your-32-byte-hex-string"

# Better-Auth URL
BETTER_AUTH_URL="http://localhost:3000"

# Optional: OAuth (leave empty for now)
GOOGLE_CLIENT_ID=""
GOOGLE_CLIENT_SECRET=""
GITHUB_CLIENT_ID=""
GITHUB_CLIENT_SECRET=""
```

## Step 3: Push Database Schema

Following **@neon-database skill** migration workflow:

```bash
# Generate migration files
npm run db:generate

# Push schema to Neon database
npm run db:push
```

This creates:
- `users` table
- `sessions` table
- `chat_history` table

## Step 4: Update Login/Signup Pages

Following **@authentication_agent** patterns, update the auth pages to connect to API:

### Update `src/pages/auth/login.tsx`:

Replace the `handleSubmit` function:

```typescript
const handleSubmit = async (e: React.FormEvent) => {
  e.preventDefault();
  setError('');
  setLoading(true);

  try {
    const res = await fetch('/api/auth/signin/email', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password }),
    });

    if (res.ok) {
      // Redirect to chat page
      window.location.href = '/chat';
    } else {
      const data = await res.json();
      setError(data.error || 'Invalid email or password');
    }
  } catch (err) {
    setError('Login failed. Please try again.');
  } finally {
    setLoading(false);
  }
};
```

### Update `src/pages/auth/signup.tsx`:

Replace the `handleSubmit` function:

```typescript
const handleSubmit = async (e: React.FormEvent) => {
  e.preventDefault();
  setError('');

  // Validation
  if (formData.password !== formData.confirmPassword) {
    setError('Passwords do not match');
    return;
  }

  if (formData.password.length < 8) {
    setError('Password must be at least 8 characters');
    return;
  }

  setLoading(true);

  try {
    const res = await fetch('/api/auth/signup/email', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        email: formData.email,
        password: formData.password,
        name: formData.name,
      }),
    });

    if (res.ok) {
      // Redirect to login
      window.location.href = '/auth/login?registered=true';
    } else {
      const data = await res.json();
      setError(data.error || 'Failed to create account');
    }
  } catch (err) {
    setError('Signup failed. Please try again.');
  } finally {
    setLoading(false);
  }
};
```

## Step 5: Test Authentication

1. **Start development server**:
   ```bash
   npm start
   ```

2. **Test Signup**:
   - Visit: `http://localhost:3000/RAGai/auth/signup`
   - Create account with email/password
   - Should redirect to login

3. **Test Login**:
   - Visit: `http://localhost:3000/RAGai/auth/login`
   - Login with credentials
   - Should create session

4. **Verify in Neon**:
   - Open Neon dashboard
   - Go to Tables → `users`
   - See your new user record
   - Check `sessions` table for active session

## Step 6: Add Protected Route Example

Create a simple protected chat page:

```typescript
// src/pages/chat/index.tsx
import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';

export default function Chat() {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const history = useHistory();

  useEffect(() => {
    // Check authentication
    fetch('/api/auth/session')
      .then(res => res.json())
      .then(data => {
        if (!data.user) {
          history.push('/auth/login');
        } else {
          setUser(data.user);
        }
      })
      .catch(() => {
        history.push('/auth/login');
      })
      .finally(() => {
        setLoading(false);
      });
  }, []);

  if (loading) {
    return <Layout><div>Loading...</div></Layout>;
  }

  return (
    <Layout title="Chat">
      <div style={{ padding: '2rem' }}>
        <h1>Welcome, {user?.name || user?.email}!</h1>
        <p>Chat interface coming soon...</p>
      </div>
    </Layout>
  );
}
```

## Troubleshooting

### Database Connection Issues

**Error**: `Connection refused`

**Solution** (from @neon-database skill):
- Verify `DATABASE_URL` in `.env.local`
- Check Neon project is active
- Ensure `?sslmode=require` is in connection string

### Authentication Errors

**Error**: `Invalid credentials`

**Solution** (from @authentication_agent):
- Check password is hashed correctly
- Verify email exists in database
- Check session cookie settings

### Migration Errors

**Error**: `Table already exists`

**Solution** (from @neon-database skill):
- Run `npm run db:push` to sync schema
- Or manually drop tables in Neon dashboard

## Next Steps

✅ Authentication working  
⏭️ Phase 2: Qdrant vector database setup  
⏭️ Phase 3: RAG pipeline implementation  
⏭️ Phase 4: OpenAI integration  

---

**Resources Used**:
- @authentication_agent - Security patterns, session management
- @neon-database skill - Database setup, query patterns, migrations
- @database_agent - Schema design, CRUD operations
