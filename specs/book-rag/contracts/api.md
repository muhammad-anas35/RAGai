# API Contracts - Book RAG Project

## Overview
This document defines all API endpoints, request/response schemas, error codes, and test requirements for the Book RAG backend.

**Base URL**: `http://localhost:4000` (development)  
**Production URL**: TBD (Vercel deployment)

---

## Authentication Endpoints

### 1. Health Check

**Endpoint**: `GET /api/health`  
**Authentication**: None  
**Description**: Check API server status

#### Request
```http
GET /api/health HTTP/1.1
Host: localhost:4000
```

#### Response (200 OK)
```json
{
  "status": "ok",
  "timestamp": "2025-12-04T20:40:00.000Z"
}
```

#### Test Requirements
- [ ] Returns 200 status code
- [ ] Response contains `status` field with value "ok"
- [ ] Response contains valid ISO timestamp
- [ ] Response time < 100ms

---

### 2. Get Session

**Endpoint**: `GET /api/auth/session`  
**Authentication**: Optional (cookie-based)  
**Description**: Get current user session information

#### Request
```http
GET /api/auth/session HTTP/1.1
Host: localhost:4000
Cookie: better-auth.session_token=<token>
```

#### Response (200 OK - Authenticated)
```json
{
  "authenticated": true,
  "session": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "userId": "660e8400-e29b-41d4-a716-446655440001",
    "expiresAt": "2025-12-05T20:40:00.000Z"
  },
  "user": {
    "id": "660e8400-e29b-41d4-a716-446655440001",
    "email": "user@example.com",
    "name": "John Doe",
    "emailVerified": false
  }
}
```

#### Response (200 OK - Not Authenticated)
```json
{
  "authenticated": false,
  "session": null,
  "user": null
}
```

#### Test Requirements
- [ ] Returns 200 for valid session
- [ ] Returns 200 for no session (not 401)
- [ ] `authenticated` field is boolean
- [ ] Session data matches database
- [ ] Expired sessions return `authenticated: false`

---

### 3. User Signup

**Endpoint**: `POST /api/auth/signup/email`  
**Authentication**: None  
**Description**: Register a new user account

#### Request
```http
POST /api/auth/signup/email HTTP/1.1
Host: localhost:4000
Content-Type: application/json

{
  "email": "newuser@example.com",
  "password": "SecurePass123!",
  "name": "Jane Doe"
}
```

#### Request Schema
```typescript
{
  email: string;      // Required, valid email format
  password: string;   // Required, min 8 characters
  name?: string;      // Optional
}
```

#### Response (201 Created)
```json
{
  "success": true,
  "message": "Account created successfully",
  "user": {
    "id": "770e8400-e29b-41d4-a716-446655440002",
    "email": "newuser@example.com",
    "name": "Jane Doe"
  }
}
```

#### Response (400 Bad Request - Validation Error)
```json
{
  "success": false,
  "error": "Validation failed",
  "details": {
    "email": "Invalid email format",
    "password": "Password must be at least 8 characters"
  }
}
```

#### Response (409 Conflict - Email Exists)
```json
{
  "success": false,
  "error": "Email already registered"
}
```

#### Test Requirements
- [ ] Returns 201 for valid signup
- [ ] Creates user in database
- [ ] Password is hashed (bcrypt)
- [ ] Returns 400 for invalid email
- [ ] Returns 400 for short password
- [ ] Returns 409 for duplicate email
- [ ] Sets session cookie on success
- [ ] Email is case-insensitive

---

### 4. User Login

**Endpoint**: `POST /api/auth/signin/email`  
**Authentication**: None  
**Description**: Authenticate user and create session

#### Request
```http
POST /api/auth/signin/email HTTP/1.1
Host: localhost:4000
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "SecurePass123!"
}
```

#### Request Schema
```typescript
{
  email: string;      // Required
  password: string;   // Required
}
```

#### Response (200 OK)
```http
HTTP/1.1 200 OK
Set-Cookie: better-auth.session_token=<token>; HttpOnly; Secure; SameSite=Lax; Max-Age=86400
Content-Type: application/json

{
  "success": true,
  "message": "Logged in successfully",
  "user": {
    "id": "660e8400-e29b-41d4-a716-446655440001",
    "email": "user@example.com",
    "name": "John Doe"
  }
}
```

#### Response (401 Unauthorized - Invalid Credentials)
```json
{
  "success": false,
  "error": "Invalid email or password"
}
```

#### Response (429 Too Many Requests - Rate Limit)
```json
{
  "success": false,
  "error": "Too many login attempts. Please try again later."
}
```

#### Test Requirements
- [ ] Returns 200 for valid credentials
- [ ] Sets httpOnly session cookie
- [ ] Returns 401 for invalid email
- [ ] Returns 401 for invalid password
- [ ] Error message doesn't reveal which field is wrong
- [ ] Rate limiting after 5 failed attempts
- [ ] Session created in database
- [ ] Session expires after 24 hours

---

### 5. User Logout

**Endpoint**: `POST /api/auth/signout`  
**Authentication**: Required (cookie-based)  
**Description**: Invalidate user session

#### Request
```http
POST /api/auth/signout HTTP/1.1
Host: localhost:4000
Cookie: better-auth.session_token=<token>
```

#### Response (200 OK)
```http
HTTP/1.1 200 OK
Set-Cookie: better-auth.session_token=; HttpOnly; Secure; SameSite=Lax; Max-Age=0
Content-Type: application/json

{
  "success": true,
  "message": "Logged out successfully"
}
```

#### Test Requirements
- [ ] Returns 200 for valid session
- [ ] Clears session cookie
- [ ] Deletes session from database
- [ ] Works even if session is already invalid
- [ ] Returns 200 (not 401) if no session

---

## Chat Endpoints (Planned - Phase 4)

### 6. Send Chat Message

**Endpoint**: `POST /api/chat`  
**Authentication**: Required  
**Description**: Send a message and get AI response with RAG

#### Request
```http
POST /api/chat HTTP/1.1
Host: localhost:4000
Cookie: better-auth.session_token=<token>
Content-Type: application/json

{
  "message": "What is ROS 2?",
  "conversationId": "880e8400-e29b-41d4-a716-446655440003"
}
```

#### Request Schema
```typescript
{
  message: string;           // Required, max 2000 characters
  conversationId?: string;   // Optional, creates new if not provided
}
```

#### Response (200 OK)
```json
{
  "success": true,
  "conversationId": "880e8400-e29b-41d4-a716-446655440003",
  "message": {
    "id": "990e8400-e29b-41d4-a716-446655440004",
    "role": "assistant",
    "content": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software...",
    "sources": [
      {
        "chunkId": "aa0e8400-e29b-41d4-a716-446655440005",
        "documentPath": "docs/physical-ai-book/chapter2/architecture.md",
        "content": "ROS 2 is built on a Data Distribution Service (DDS)...",
        "score": 0.92
      }
    ],
    "createdAt": "2025-12-04T20:40:00.000Z"
  }
}
```

#### Response (401 Unauthorized)
```json
{
  "success": false,
  "error": "Authentication required"
}
```

#### Response (400 Bad Request)
```json
{
  "success": false,
  "error": "Message cannot be empty"
}
```

#### Response (500 Internal Server Error)
```json
{
  "success": false,
  "error": "Failed to generate response. Please try again."
}
```

#### Test Requirements
- [ ] Returns 401 if not authenticated
- [ ] Returns 400 for empty message
- [ ] Returns 400 for message > 2000 chars
- [ ] Creates new conversation if no conversationId
- [ ] Uses existing conversation if valid conversationId
- [ ] Returns 403 if conversationId belongs to another user
- [ ] Response time < 3 seconds (p95)
- [ ] Sources are relevant to query
- [ ] AI response is contextually accurate
- [ ] Saves both user and AI messages to database

---

### 7. Get Chat History

**Endpoint**: `GET /api/chat/history`  
**Authentication**: Required  
**Description**: Get user's conversation history

#### Request
```http
GET /api/chat/history?limit=20&offset=0 HTTP/1.1
Host: localhost:4000
Cookie: better-auth.session_token=<token>
```

#### Query Parameters
- `limit` (optional): Number of conversations to return (default: 20, max: 100)
- `offset` (optional): Pagination offset (default: 0)

#### Response (200 OK)
```json
{
  "success": true,
  "conversations": [
    {
      "id": "880e8400-e29b-41d4-a716-446655440003",
      "title": "ROS 2 Architecture Questions",
      "createdAt": "2025-12-04T18:00:00.000Z",
      "updatedAt": "2025-12-04T20:40:00.000Z",
      "messageCount": 8
    }
  ],
  "total": 15,
  "limit": 20,
  "offset": 0
}
```

#### Test Requirements
- [ ] Returns 401 if not authenticated
- [ ] Returns only user's own conversations
- [ ] Pagination works correctly
- [ ] Conversations ordered by updatedAt (newest first)
- [ ] messageCount is accurate

---

### 8. Get Conversation Messages

**Endpoint**: `GET /api/chat/conversation/:conversationId`  
**Authentication**: Required  
**Description**: Get all messages in a conversation

#### Request
```http
GET /api/chat/conversation/880e8400-e29b-41d4-a716-446655440003 HTTP/1.1
Host: localhost:4000
Cookie: better-auth.session_token=<token>
```

#### Response (200 OK)
```json
{
  "success": true,
  "conversation": {
    "id": "880e8400-e29b-41d4-a716-446655440003",
    "title": "ROS 2 Architecture Questions",
    "createdAt": "2025-12-04T18:00:00.000Z",
    "updatedAt": "2025-12-04T20:40:00.000Z"
  },
  "messages": [
    {
      "id": "990e8400-e29b-41d4-a716-446655440004",
      "role": "user",
      "content": "What is ROS 2?",
      "createdAt": "2025-12-04T18:00:00.000Z"
    },
    {
      "id": "aa0e8400-e29b-41d4-a716-446655440005",
      "role": "assistant",
      "content": "ROS 2 is...",
      "sources": [...],
      "createdAt": "2025-12-04T18:00:02.000Z"
    }
  ]
}
```

#### Response (403 Forbidden)
```json
{
  "success": false,
  "error": "Access denied"
}
```

#### Response (404 Not Found)
```json
{
  "success": false,
  "error": "Conversation not found"
}
```

#### Test Requirements
- [ ] Returns 401 if not authenticated
- [ ] Returns 403 if conversation belongs to another user
- [ ] Returns 404 if conversation doesn't exist
- [ ] Messages ordered chronologically
- [ ] Sources included for assistant messages

---

## Error Response Format

All error responses follow this structure:

```typescript
{
  success: false;
  error: string;           // Human-readable error message
  details?: object;        // Optional validation details
  code?: string;           // Optional error code
}
```

### Error Codes

| HTTP Status | Error Code | Description |
|-------------|------------|-------------|
| 400 | `VALIDATION_ERROR` | Request validation failed |
| 401 | `UNAUTHORIZED` | Authentication required |
| 403 | `FORBIDDEN` | Access denied |
| 404 | `NOT_FOUND` | Resource not found |
| 409 | `CONFLICT` | Resource already exists |
| 429 | `RATE_LIMIT` | Too many requests |
| 500 | `INTERNAL_ERROR` | Server error |
| 503 | `SERVICE_UNAVAILABLE` | External service unavailable |

---

## Rate Limiting

### Limits
- **Authentication endpoints**: 5 requests per minute per IP
- **Chat endpoints**: 20 requests per minute per user
- **History endpoints**: 60 requests per minute per user

### Headers
```http
X-RateLimit-Limit: 20
X-RateLimit-Remaining: 15
X-RateLimit-Reset: 1733342400
```

---

## CORS Configuration

### Allowed Origins
- Development: `http://localhost:3000`
- Production: `https://book-rag.vercel.app`

### Allowed Methods
- `GET`, `POST`, `PUT`, `DELETE`, `OPTIONS`

### Allowed Headers
- `Content-Type`, `Authorization`, `Cookie`

---

## Test Coverage Requirements

### Unit Tests
- [ ] All request validation functions
- [ ] All response formatting functions
- [ ] Error handling for each endpoint

### Integration Tests
- [ ] Complete authentication flow
- [ ] Complete chat flow
- [ ] Error scenarios for each endpoint
- [ ] Rate limiting behavior

### Performance Tests
- [ ] Health check < 100ms
- [ ] Auth endpoints < 500ms
- [ ] Chat endpoint < 3s (p95)
- [ ] History endpoints < 500ms

---

## Next Steps

1. **Phase 4**: Implement chat endpoints (`/api/chat`, `/api/chat/history`)
2. **Phase 5**: Add conversation management endpoints
3. **Phase 6**: Add OpenAPI/Swagger documentation
4. **Testing**: Write integration tests for all endpoints
