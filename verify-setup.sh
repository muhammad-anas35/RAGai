#!/bin/bash
# Verification script for Book RAG authentication system

echo "📋 Book RAG - System Verification Script"
echo "========================================"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check 1: .env.local exists
echo "1️⃣  Checking .env.local..."
if [ -f ".env.local" ]; then
    echo -e "${GREEN}✓ .env.local found${NC}"
else
    echo -e "${RED}✗ .env.local NOT found${NC}"
    echo "  → Create .env.local in root folder"
    exit 1
fi

# Check 2: Required environment variables
echo ""
echo "2️⃣  Checking environment variables..."

if grep -q "DATABASE_URL=" .env.local; then
    echo -e "${GREEN}✓ DATABASE_URL configured${NC}"
else
    echo -e "${RED}✗ DATABASE_URL missing${NC}"
    echo "  → Add DATABASE_URL from Neon to .env.local"
fi

if grep -q "BETTER_AUTH_SECRET=" .env.local; then
    echo -e "${GREEN}✓ BETTER_AUTH_SECRET configured${NC}"
else
    echo -e "${RED}✗ BETTER_AUTH_SECRET missing${NC}"
    echo "  → Generate: openssl rand -hex 32"
    echo "  → Add to .env.local"
fi

if grep -q "BETTER_AUTH_URL=" .env.local; then
    echo -e "${GREEN}✓ BETTER_AUTH_URL configured${NC}"
else
    echo -e "${YELLOW}⚠ BETTER_AUTH_URL missing (optional, defaults to http://localhost:3000)${NC}"
fi

# Check 3: Dependencies installed
echo ""
echo "3️⃣  Checking dependencies..."

if [ -d "node_modules" ]; then
    echo -e "${GREEN}✓ Root dependencies installed${NC}"
else
    echo -e "${RED}✗ Root dependencies NOT installed${NC}"
    echo "  → Run: npm install"
fi

if [ -d "backend/node_modules" ]; then
    echo -e "${GREEN}✓ Backend dependencies installed${NC}"
else
    echo -e "${RED}✗ Backend dependencies NOT installed${NC}"
    echo "  → Run: cd backend && npm install"
fi

# Check 4: Database schema file
echo ""
echo "4️⃣  Checking database schema..."

if [ -f "src/db/schema.ts" ]; then
    echo -e "${GREEN}✓ Database schema defined${NC}"
else
    echo -e "${RED}✗ Database schema NOT found${NC}"
fi

# Check 5: Authentication files
echo ""
echo "5️⃣  Checking authentication files..."

files=(
    "src/pages/auth/login.tsx"
    "src/pages/auth/signup.tsx"
    "src/pages/auth/auth.module.css"
    "backend/src/server.ts"
)

for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓ $file exists${NC}"
    else
        echo -e "${RED}✗ $file NOT found${NC}"
    fi
done

# Check 6: Configuration files
echo ""
echo "6️⃣  Checking configuration files..."

if [ -f "drizzle.config.ts" ]; then
    echo -e "${GREEN}✓ Drizzle ORM configured${NC}"
else
    echo -e "${RED}✗ drizzle.config.ts NOT found${NC}"
fi

if [ -f "backend/package.json" ]; then
    echo -e "${GREEN}✓ Backend package.json exists${NC}"
else
    echo -e "${RED}✗ Backend package.json NOT found${NC}"
fi

# Summary
echo ""
echo "========================================"
echo "✅ Verification Complete!"
echo ""
echo "📝 Next Steps:"
echo "1. Start backend:  cd backend && npm run dev"
echo "2. Start frontend: npm start"
echo "3. Visit:          http://localhost:3000"
echo ""
echo "🔧 If any checks failed:"
echo "   → See DEBUG_SIGNUP.md for detailed solutions"
echo "   → Run: npm run db:push (to create database tables)"
echo ""
