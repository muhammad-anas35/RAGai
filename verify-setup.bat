@echo off
REM Verification script for Book RAG authentication system
REM Run this from the root folder: verify-setup.bat

echo.
echo 📋 Book RAG - System Verification Script
echo ========================================
echo.

setlocal enabledelayedexpansion

REM Check 1: .env.local exists
echo 1️⃣  Checking .env.local...
if exist ".env.local" (
    echo ✓ .env.local found
) else (
    echo ✗ .env.local NOT found
    echo   → Create .env.local in root folder
    exit /b 1
)

REM Check 2: Required environment variables
echo.
echo 2️⃣  Checking environment variables...

findstr /m "DATABASE_URL=" .env.local >nul
if !errorlevel! equ 0 (
    echo ✓ DATABASE_URL configured
) else (
    echo ✗ DATABASE_URL missing
    echo   → Add DATABASE_URL from Neon to .env.local
)

findstr /m "BETTER_AUTH_SECRET=" .env.local >nul
if !errorlevel! equ 0 (
    echo ✓ BETTER_AUTH_SECRET configured
) else (
    echo ✗ BETTER_AUTH_SECRET missing
    echo   → Generate: openssl rand -hex 32
    echo   → Add to .env.local
)

findstr /m "BETTER_AUTH_URL=" .env.local >nul
if !errorlevel! equ 0 (
    echo ✓ BETTER_AUTH_URL configured
) else (
    echo ⚠ BETTER_AUTH_URL missing (optional, defaults to http://localhost:3000)
)

REM Check 3: Dependencies installed
echo.
echo 3️⃣  Checking dependencies...

if exist "node_modules" (
    echo ✓ Root dependencies installed
) else (
    echo ✗ Root dependencies NOT installed
    echo   → Run: npm install
)

if exist "backend\node_modules" (
    echo ✓ Backend dependencies installed
) else (
    echo ✗ Backend dependencies NOT installed
    echo   → Run: cd backend ^& npm install
)

REM Check 4: Database schema file
echo.
echo 4️⃣  Checking database schema...

if exist "src\db\schema.ts" (
    echo ✓ Database schema defined
) else (
    echo ✗ Database schema NOT found
)

REM Check 5: Authentication files
echo.
echo 5️⃣  Checking authentication files...

if exist "src\pages\auth\login.tsx" (
    echo ✓ src\pages\auth\login.tsx exists
) else (
    echo ✗ src\pages\auth\login.tsx NOT found
)

if exist "src\pages\auth\signup.tsx" (
    echo ✓ src\pages\auth\signup.tsx exists
) else (
    echo ✗ src\pages\auth\signup.tsx NOT found
)

if exist "src\pages\auth\auth.module.css" (
    echo ✓ src\pages\auth\auth.module.css exists
) else (
    echo ✗ src\pages\auth\auth.module.css NOT found
)

if exist "backend\src\server.ts" (
    echo ✓ backend\src\server.ts exists
) else (
    echo ✗ backend\src\server.ts NOT found
)

REM Check 6: Configuration files
echo.
echo 6️⃣  Checking configuration files...

if exist "drizzle.config.ts" (
    echo ✓ Drizzle ORM configured
) else (
    echo ✗ drizzle.config.ts NOT found
)

if exist "backend\package.json" (
    echo ✓ Backend package.json exists
) else (
    echo ✗ Backend package.json NOT found
)

REM Summary
echo.
echo ========================================
echo ✅ Verification Complete!
echo.
echo 📝 Next Steps:
echo 1. Start backend:  cd backend ^& npm run dev
echo 2. Start frontend: npm start
echo 3. Visit:          http://localhost:3000
echo.
echo 🔧 If any checks failed:
echo    → See DEBUG_SIGNUP.md for detailed solutions
echo    → Run: npm run db:push (to create database tables)
echo.

pause
