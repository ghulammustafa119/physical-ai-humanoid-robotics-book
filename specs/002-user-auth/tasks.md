# Implementation Tasks: User Authentication & Background-Aware Personalization

**Feature**: 002-user-auth | **Date**: 2025-12-30 | **Plan**: [plan.md](plan.md)

---

## Phase 1: Database & Backend Foundation

### Task 1.1: Create Database Schema

**Description**: Create SQL migration scripts for users, sessions, and user_profiles tables in Neon Postgres

**Status**: 🟢 COMPLETE | **Priority**: P1 | **Est. Time**: 2h

**Acceptance Criteria**:
- [x] `CREATE TABLE` scripts exist for `user`, `session`, `user_profile` tables
- [x] All constraints (PK, FK, CHECK) match data-model.md specification
- [x] Indexes created for efficient queries on user_id, email, token, expires_at
- [x] Script is idempotent (can run multiple times safely)
- [x] Script includes DROP statements for clean teardown

**Test**: Run migration against test database, verify tables created with correct schema

**Files to Create**:
- `backend/src/db/migrations/001_create_auth_tables.sql`

---

### Task 1.2: Create Pydantic Models

**Description**: Create Python Pydantic models for User, Session, UserProfile entities

**Status**: 🟢 COMPLETE | **Priority**: P1 | **Est. Time**: 3h

**Acceptance Criteria**:
- [x] User model with id, email, password_hash, created_at, updated_at
- [x] Session model with id, user_id, token, expires_at, ip_address, user_agent
- [x] UserProfile model with all 8 background fields plus completeness score
- [x] All models inherit from BaseModel
- [x] Validation rules match API contracts (email format, enum values)
- [x] SQLAlchemy ORM models also created for database operations

**Test**: `pytest tests/unit/test_models.py` - All validation tests pass

**Files to Create**:
- `backend/src/models/user.py`
- `backend/src/models/session.py`
- `backend/src/models/profile.py`

---

### Task 1.3: Create Database Connection Layer

**Description**: Create database connection utilities for Neon Postgres

**Status**: 🟢 COMPLETE | **Priority**: P1 | **Est. Time**: 2h

**Acceptance Criteria**:
- [x] Connection to Neon Postgres via DATABASE_URL
- [x] Session factory for SQLAlchemy
- [x] Dependency injection function `get_db()` for FastAPI
- [x] Connection pooling configured appropriately
- [x] Error handling for connection failures

**Test**: `pytest tests/integration/test_db_connection.py` - Connection succeeds

**Files to Create**:
- `backend/src/db/connection.py`
- `backend/src/db/dependency.py`

---

## Phase 2: Authentication Service

### Task 2.1: Create Auth Service with Better Auth Integration

**Description**: Implement authentication service using Better Auth for session management

**Status**: 🟢 COMPLETE | **Priority**: P1 | **Est. Time**: 4h

**Acceptance Criteria**:
- [x] Password hashing using bcrypt
- [x] User creation with email validation
- [x] Session token generation
- [x] Session validation and expiration handling
- [x] Sign out functionality (session revocation)
- [x] Integration with Better Auth server SDK

**Test**: `pytest tests/unit/test_auth_service.py` - All auth operations pass

**Files to Create**:
- `backend/src/services/auth_service.py`

**Key Functions**:
```python
async def create_user(email: str, password: str) -> User
async def authenticate_user(email: str, password: str) -> Optional[Session]
async def validate_session(token: str) -> Optional[Session]
async def revoke_session(session_id: str) -> bool
async def get_user_by_id(user_id: str) -> Optional[User]
```

---

### Task 2.2: Create Auth API Endpoints

**Description**: Implement FastAPI endpoints for signup, signin, signout, session, and profile

**Status**: 🟢 COMPLETE | **Priority**: P1 | **Est. Time**: 4h

**Acceptance Criteria**:
- [x] POST /auth/signup - Returns 201 on success, 409 if email exists
- [x] POST /auth/signin - Returns session token on success, 401 on failure
- [x] POST /auth/signout - Returns 200, requires auth
- [x] GET /auth/session - Returns session info, requires auth
- [x] GET /auth/profile - Returns user profile, requires auth
- [x] PUT /auth/profile - Updates profile, requires auth
- [x] Proper error responses with meaningful messages
- [x] Input validation on all endpoints

**Test**: `pytest tests/integration/test_auth_api.py` - All endpoint tests pass

**Files to Create**:
- `backend/src/api/v1/auth.py`

---

### Task 2.3: Create Auth Middleware

**Description**: Create FastAPI middleware for session validation and user context injection

**Status**: 🟢 COMPLETE | **Priority**: P1 | **Est. Time**: 2h

**Acceptance Criteria**:
- [x] Extract session token from Authorization header or cookie
- [x] Validate session against database
- [x] Inject user_id into request state for downstream handlers
- [x] Handle expired sessions gracefully (401 response)
- [x] Pass through unauthenticated requests for public endpoints

**Test**: `pytest tests/unit/test_auth_middleware.py` - Middleware tests pass

**Files to Create**:
- `backend/src/middleware/auth.py`

---

## Phase 3: Profile Management

### Task 3.1: Create Profile Service

**Description**: Implement service for user profile CRUD operations

**Status**: 🟢 COMPLETE | **Priority**: P2 | **Est. Time**: 3h

**Acceptance Criteria**:
- [x] Create profile for new user
- [x] Get profile by user_id
- [x] Update profile fields
- [x] Calculate profile completeness score
- [x] Handle partial profile (not all fields filled)

**Test**: `pytest tests/unit/test_profile_service.py` - All profile operations pass

**Files to Create**:
- `backend/src/services/profile_service.py`

**Key Functions**:
```python
async def create_profile(user_id: str, profile_data: dict) -> UserProfile
async def get_profile(user_id: str) -> Optional[UserProfile]
async def update_profile(user_id: str, updates: dict) -> UserProfile
async def calculate_completeness(profile: UserProfile) -> float
```

---

### Task 3.2: Create Personalization Context Service

**Description**: Build service that aggregates user background data into PersonalizationContext

**Status**: 🟢 COMPLETE | **Priority**: P2 | **Est. Time**: 3h

**Acceptance Criteria**:
- [x] Fetch user profile by user_id
- [x] Transform profile data into PersonalizationContext format
- [x] Provide fallback for users without profiles
- [x] Cache context for performance (optional)
- [x] Context includes skill level, hardware info, simulator experience

**Test**: `pytest tests/unit/test_personalization.py` - Context generation passes

**Files to Create**:
- `backend/src/services/personalization_service.py`

---

## Phase 4: RAG Integration

### Task 4.1: Modify RAG Service for Personalization

**Description**: Update RAG service to accept and inject PersonalizationContext into prompts

**Status**: 🟢 COMPLETE | **Priority**: P1 | **Est. Time**: 4h

**Acceptance Criteria**:
- [x] RAG service accepts optional PersonalizationContext parameter
- [x] When context provided, injects user background into system prompt
- [x] When no context (guest), uses generic system prompt
- [x] Response quality adapted based on skill level
- [x] Code examples optimized based on hardware capabilities
- [x] RAG integrity maintained - answers still grounded in book content

**Test**: `pytest tests/integration/test_rag_personalization.py` - Personalized responses work

**Files to Modify**:
- `backend/src/services/rag_service.py`

---

## Phase 5: Frontend Components

### Task 5.1: Create Auth UI Components

**Description**: Build React components for Sign Up, Sign In, and Profile management

**Status**: 🔴 TODO | **Priority**: P1 | **Est. Time**: 6h

**Acceptance Criteria**:
- [ ] Sign Up form with email, password, and 8 profile questions
- [ ] Sign In form with email and password
- [ ] Profile view showing current settings
- [ ] Profile edit form to update background
- [ ] Form validation with error messages
- [ ] Loading states during API calls
- [ ] Responsive design

**Test**: Manual testing of all forms; E2E tests in Task 5.4

**Files to Create**:
- `frontend/physical-ai-book/src/components/auth/SignUp.tsx`
- `frontend/physical-ai-book/src/components/auth/SignIn.tsx`
- `frontend/physical-ai-book/src/components/auth/Profile.tsx`
- `frontend/physical-ai-book/src/components/auth/styles.module.css`

---

### Task 5.2: Integrate Better Auth Client

**Description**: Configure Better Auth client SDK in Docusaurus

**Status**: 🔴 TODO | **Priority**: P1 | **Est. Time**: 3h

**Acceptance Criteria**:
- [ ] Better Auth configured in docusaurus.config.ts
- [ ] API path points to backend (/api/v1/auth)
- [ ] Session persisted in HTTP-only cookies
- [ ] Sign in/sign out flows work
- [ ] Session available across page navigations

**Test**: Manual testing of session persistence

**Files to Modify**:
- `frontend/physical-ai-book/docusaurus.config.ts`
- `frontend/physical-ai-book/src/utils/auth.ts`

---

### Task 5.3: Create Auth Navigation

**Description**: Add authentication buttons and user menu to navigation bar

**Status**: 🔴 TODO | **Priority**: P2 | **Est. Time**: 2h

**Acceptance Criteria**:
- [ ] Show "Sign In" button when not logged in
- [ ] Show user avatar/menu when logged in
- [ ] Menu options: Profile, Sign Out
- [ ] Clicking profile opens profile modal/page
- [ ] Consistent with Docusaurus theme

**Test**: Manual navigation testing

**Files to Create/Modify**:
- `frontend/physical-ai-book/src/components/auth/UserMenu.tsx`
- Modify navbar component

---

### Task 5.4: Create E2E Auth Tests

**Description**: Write Playwright E2E tests for authentication flows

**Status**: 🔴 TODO | **Priority**: P2 | **Est. Time**: 4h

**Acceptance Criteria**:
- [ ] Test user can sign up with profile data
- [ ] Test user can sign in with correct credentials
- [ ] Test sign in with wrong password shows error
- [ ] Test session persists after browser refresh
- [ ] Test user can sign out
- [ ] Test profile can be updated
- [ ] Test chatbot provides personalized responses when logged in

**Test**: `npx playwright test tests/e2e/auth.spec.ts` - All E2E tests pass

**Files to Create**:
- `frontend/physical-ai-book/tests/e2e/auth.spec.ts`
- `frontend/physical-ai-book/tests/e2e/playwright.config.ts`

---

## Phase 6: Testing & Documentation

### Task 6.1: Write Unit Tests for Backend

**Description**: Create comprehensive unit tests for all backend components

**Status**: 🔴 TODO | **Priority**: P1 | **Est. Time**: 6h

**Test Files**:
- `tests/unit/test_models.py` - Model validation
- `tests/unit/test_auth_service.py` - Auth operations
- `tests/unit/test_profile_service.py` - Profile operations
- `tests/unit/test_personalization.py` - Context generation
- `tests/unit/test_auth_middleware.py` - Middleware behavior

**Acceptance Criteria**: All tests pass with >80% coverage

---

### Task 6.2: Write Integration Tests for API

**Description**: Create integration tests for API endpoints

**Status**: 🔴 TODO | **Priority**: P1 | **Est. Time**: 4h

**Test Files**:
- `tests/integration/test_auth_api.py` - All auth endpoints
- `tests/integration/test_profile_api.py` - Profile endpoints
- `tests/integration/test_db_connection.py` - Database connection
- `tests/integration/test_rag_personalization.py` - RAG integration

**Acceptance Criteria**: All tests pass

---

### Task 6.3: Update README

**Description**: Update project documentation with authentication feature

**Status**: 🔴 TODO | **Priority**: P3 | **Est. Time**: 2h

**Files to Modify**:
- `README.md` - Add authentication section
- `docs/authentication.md` - New detailed auth docs

---

## Task Dependencies

```
Phase 1 (Database)
├── Task 1.1: Create Database Schema
├── Task 1.2: Create Pydantic Models ─────┐
└── Task 1.3: Create DB Connection ───────┤
                                          │
Phase 2 (Auth Service)                    │
├── Task 2.1: Create Auth Service ◄───────┤
├── Task 2.2: Create Auth API ◄───────────┤
└── Task 2.3: Create Auth Middleware ◄────┘

Phase 3 (Profile)
├── Task 3.1: Create Profile Service ◄────┐
└── Task 3.2: Create Personalization ◄────┤
                                            │
Phase 4 (RAG Integration)                  │
└── Task 4.1: Modify RAG Service ◄─────────┘

Phase 5 (Frontend)
├── Task 5.1: Create Auth UI Components
├── Task 5.2: Integrate Better Auth Client
├── Task 5.3: Create Auth Navigation
└── Task 5.4: Create E2E Tests

Phase 6 (Testing)
├── Task 6.1: Write Unit Tests
├── Task 6.2: Write Integration Tests
└── Task 6.3: Update README
```

---

## Execution Order (Recommended)

1. **Sprint 1**: Database Foundation
   - Task 1.1, 1.2, 1.3, 2.3

2. **Sprint 2**: Auth Service
   - Task 2.1, 2.2, 3.1

3. **Sprint 3**: Personalization & RAG
   - Task 3.2, 4.1

4. **Sprint 4**: Frontend
   - Task 5.2, 5.1, 5.3, 5.4

5. **Sprint 5**: Testing & Polish
   - Task 6.1, 6.2, 6.3

---

## Success Metrics

| Task | Metric | Target |
|------|--------|--------|
| All | Unit test coverage | >80% |
| All | API test pass rate | 100% |
| Task 2.1 | Sign up time | <3 seconds |
| Task 2.2 | Sign in time | <500ms |
| Task 4.1 | Profile retrieval | <200ms |
| Task 5.4 | E2E test pass rate | 100% |

---

## Notes

- All code must follow Red-Green-Refactor TDD cycle
- Each task should have a corresponding test file
- Use type hints throughout Python code
- Use TypeScript for all frontend code
- Commit frequently with meaningful messages
