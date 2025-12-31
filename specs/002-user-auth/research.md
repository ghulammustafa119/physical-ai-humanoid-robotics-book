# Research: User Authentication & Background-Aware Personalization System

**Feature**: 002-user-auth | **Date**: 2025-12-30

## Research Questions

### RQ1: Better Auth Integration with Docusaurus/React

**Question**: How to integrate Better Auth with Docusaurus React frontend and FastAPI backend?

**Decision**: Use Better Auth client SDK for React with API routes in FastAPI

**Rationale**:
- Better Auth provides official React client SDK that works with any React app including Docusaurus
- FastAPI backend will serve as the authentication API endpoint
- Session tokens exchanged via HTTP-only cookies for security
- Better Auth supports email/password authentication flow out of the box

**Alternatives Considered**:
1. Custom JWT implementation - Rejected: More error-prone, no security audits
2. Auth.js/NextAuth - Rejected: Tightly coupled to Next.js ecosystem
3. Firebase Auth - Rejected: Vendor lock-in, additional cost

---

### RQ2: Session Management Architecture

**Question**: How to manage user sessions across frontend and backend with Better Auth?

**Decision**: Token-based session with HTTP-only cookies + FastAPI dependency injection

**Rationale**:
- Better Auth handles session token generation and validation
- Backend receives session token via Authorization header or cookie
- FastAPI middleware validates tokens and injects user context into requests
- Sessions persist across browser sessions with configurable expiration

**Session Flow**:
```
1. User signs up/in via Better Auth client
2. Better Auth creates session, returns session token
3. Session token stored in HTTP-only cookie (secure, same-site)
4. Each request includes cookie automatically
5. FastAPI middleware extracts and validates session
6. User ID injected into request state for downstream handlers
```

---

### RQ3: Personalization Context Injection into RAG

**Question**: How to inject user background context into RAG chatbot prompts?

**Decision**: Structured context object passed to RAG service as metadata

**Rationale**:
- PersonalizationContext aggregates user profile data
- Injected into system prompt as structured instructions
- RAG service reads context and tailors response style/content
- Graceful fallback to generic mode if user not authenticated

**Context Structure**:
```typescript
interface PersonalizationContext {
  skillLevel: 'beginner' | 'intermediate' | 'advanced';
  pythonLevel: 'none' | 'basic' | 'strong';
  aiMlLevel: 'none' | 'basic' | 'applied';
  roboticsLevel: 'none' | 'academic' | 'practical';
  systemType: 'laptop' | 'desktop' | 'cloud';
  hasGpu: boolean;
  hardwareAccess: 'none' | 'simulators' | 'real';
  simulators: string[];
}
```

**System Prompt Integration**:
```
[Base system prompt for book assistant]

[Personalization context - conditional]
User Profile:
- Skill Level: Intermediate Python Developer
- Robotics Experience: Academic (theoretical knowledge)
- Hardware: Laptop with no GPU
- Suggestion: Provide CPU-friendly code examples, include theoretical foundations

When responding to user queries:
- Adapt explanation depth based on skill level
- Provide code examples optimized for user's hardware
- Balance theoretical concepts with practical applications
```

---

### RQ4: Database Schema for User Profiles

**Question**: How to store user profiles in Neon Postgres?

**Decision**: Separate tables for users, sessions, and profiles with proper relationships

**Rationale**:
- Better Auth manages its own schema for users and sessions
- User profiles extend authentication data with personalization info
- Foreign key relationships ensure data integrity
- Indexed queries for efficient retrieval

**Schema Design**:
```sql
-- Users table (managed by Better Auth)
CREATE TABLE user (
    id TEXT PRIMARY KEY,
    email TEXT NOT NULL UNIQUE,
    password_hash TEXT,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Sessions table (managed by Better Auth)
CREATE TABLE session (
    id TEXT PRIMARY KEY,
    user_id TEXT NOT NULL REFERENCES user(id) ON DELETE CASCADE,
    token TEXT NOT NULL UNIQUE,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    ip_address TEXT,
    user_agent TEXT
);

-- User profiles table (custom for personalization)
CREATE TABLE user_profile (
    id TEXT PRIMARY KEY,
    user_id TEXT NOT NULL REFERENCES user(id) ON DELETE CASCADE,
    programming_level TEXT CHECK (programming_level IN ('beginner', 'intermediate', 'advanced')),
    python_level TEXT CHECK (python_level IN ('none', 'basic', 'strong')),
    ai_ml_level TEXT CHECK (ai_ml_level IN ('none', 'basic', 'applied')),
    robotics_level TEXT CHECK (robotics_level IN ('none', 'academic', 'practical')),
    system_type TEXT CHECK (system_type IN ('laptop', 'desktop', 'cloud')),
    gpu_availability TEXT CHECK (gpu_availability IN ('none', 'integrated', 'nvidia_cuda')),
    hardware_access TEXT CHECK (hardware_access IN ('none', 'simulators', 'real')),
    simulator_experience TEXT[], -- Array of simulator names
    profile_completeness REAL DEFAULT 0.0, -- 0.0 to 1.0
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Indexes for efficient queries
CREATE INDEX idx_user_profile_user_id ON user_profile(user_id);
CREATE INDEX idx_session_user_id ON session(user_id);
CREATE INDEX idx_session_token ON session(token);
CREATE INDEX idx_session_expires ON session(expires_at) WHERE expires_at > NOW();
```

---

### RQ5: API Contract for Profile Management

**Question**: What endpoints are needed for authentication and profile management?

**Decision**: RESTful API design aligned with Better Auth patterns

**Endpoints**:

| Method | Path | Description | Auth Required |
|--------|------|-------------|---------------|
| POST | /api/auth/signup | Create new account | No |
| POST | /api/auth/signin | Authenticate user | No |
| POST | /api/auth/signout | End session | Yes |
| GET | /api/auth/session | Get current session | Yes |
| GET | /api/auth/user | Get user profile | Yes |
| PUT | /api/auth/profile | Update profile | Yes |

**Request/Response Examples**:

Sign Up Request:
```json
{
  "email": "user@example.com",
  "password": "securePassword123",
  "profile": {
    "programming_level": "intermediate",
    "python_level": "strong",
    "ai_ml_level": "basic",
    "robotics_level": "none",
    "system_type": "laptop",
    "gpu_availability": "integrated",
    "hardware_access": "simulators",
    "simulator_experience": ["gazebo"]
  }
}
```

Sign Up Response:
```json
{
  "user": {
    "id": "usr_abc123",
    "email": "user@example.com"
  },
  "session": {
    "token": "sess_xyz789",
    "expires_at": "2026-01-06T10:00:00Z"
  },
  "profile_completeness": 1.0
}
```

---

## Technology Decisions Summary

| Category | Decision | Rationale |
|----------|----------|-----------|
| Auth Library | Better Auth | React SDK available, email/password support, session management built-in |
| Session Storage | Database + HTTP-only cookies | Secure, persistent across sessions, server-validated |
| Profile Database | Neon Postgres | Serverless, scales automatically, already in use |
| Context Injection | Structured metadata in system prompt | Extensible, testable, no hardcoded logic |
| Frontend Integration | Better Auth React client | Official SDK, Docusaurus compatible |

## Open Questions (Resolved)

1. ✅ Better Auth works with Docusaurus - Yes, via React SDK
2. ✅ Sessions persist across browser - Yes, HTTP-only cookies
3. ✅ Profile data links to auth - Yes, via user_id foreign key
4. ✅ Context injects into RAG - Yes, via personalization service

## References

- Better Auth Documentation: https://www.better-auth.com/docs
- FastAPI Dependencies: https://fastapi.tiangolo.com/tutorial/dependencies/
- Docusaurus Plugin System: https://docusaurus.io/docs/api/plugins
- Neon Serverless Postgres: https://neon.tech/docs/introduction
