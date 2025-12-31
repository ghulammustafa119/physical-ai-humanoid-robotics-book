# Implementation Plan: User Authentication & Background-Aware Personalization System

**Branch**: `002-user-auth` | **Date**: 2025-12-30 | **Spec**: [spec.md](spec.md)

**Input**: Feature specification from `/specs/002-user-auth/spec.md`

## Summary

Implement user authentication using Better Auth with email/password credentials, collect user background data during signup, store profiles in Neon Postgres, and inject personalization context into the RAG chatbot for tailored responses.

**Primary Approach**:
- Better Auth client SDK for React/Docusaurus frontend
- FastAPI backend with Better Auth server SDK
- Neon Postgres for user sessions and profile data
- PersonalizationContext passed to RAG service as metadata

## Technical Context

**Language/Version**: Python 3.11 (backend), TypeScript 5.6 (frontend)
**Primary Dependencies**: Better Auth, FastAPI, Neon Postgres, React/Docusaurus
**Storage**: Neon Serverless Postgres (users, sessions, profiles tables)
**Testing**: pytest (backend), Jest/Vitest (frontend)
**Target Platform**: Web application (Docusaurus + FastAPI)
**Performance Goals**: Profile retrieval <200ms, auth operations <500ms
**Constraints**: HTTP-only cookies for sessions, GDPR-compliant data
**Scale/Scope**: Individual user accounts, session-based auth
**Project Type**: Web application with separate frontend/backend

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Gate | Status | Notes |
|------|--------|-------|
| Technical claims verified | ✅ PASS | Better Auth documentation reviewed |
| Code examples functional | ✅ PASS | Quickstart guide provided |
| Architecture traceable | ✅ PASS | Data model and contracts defined |
| RAG integrity maintained | ✅ PASS | Generic fallback for unauthenticated users |
| Test-first workflow | ⚠️ NOTE | Tasks phase will create tests first |

**Constitution Compliance**: FULL - No violations detected

## Project Structure

### Documentation (this feature)

```text
specs/002-user-auth/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/
│   └── openapi.yaml     # API specification
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── user.py              # User entity
│   │   ├── session.py           # Session entity
│   │   └── profile.py           # UserProfile entity
│   ├── services/
│   │   ├── auth_service.py      # Better Auth integration
│   │   └── personalization.py   # Context injection
│   ├── api/
│   │   └── v1/
│   │       └── auth.py          # Auth endpoints
│   └── db/
│       └── migrations/          # SQL migrations
└── tests/
    ├── unit/
    └── integration/

frontend/physical-ai-book/
├── src/
│   ├── components/
│   │   └── auth/                # Auth UI components
│   │       ├── SignIn.tsx
│   │       ├── SignUp.tsx
│   │       └── Profile.tsx
│   └── utils/
│       └── auth.ts              # Better Auth client
└── tests/
    └── e2e/
        └── auth.spec.ts         # Auth E2E tests
```

**Structure Decision**: Web application with separate frontend (Docusaurus plugin) and backend (FastAPI). Backend handles auth logic and data storage; frontend provides UI components and session management.

## Complexity Tracking

> No complexity violations - implementation follows standard auth patterns

## Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| Better Auth for auth | Official React SDK, session management, email/password support |
| HTTP-only cookies | Secure session storage, XSS protection |
| Separate profile table | Extensible, queryable, links to auth user |
| Context injection | Metadata-based, no hardcoded prompts |

## Research Findings

All technical questions resolved in [research.md](research.md):

1. ✅ Better Auth integrates with Docusaurus via React SDK
2. ✅ Sessions persist with HTTP-only cookies + database
3. ✅ Profile data links via user_id foreign key
4. ✅ Context injected as structured metadata in RAG system prompt

## API Endpoints

| Method | Path | Description | Auth |
|--------|------|-------------|------|
| POST | /api/v1/auth/signup | Create account | No |
| POST | /api/v1/auth/signin | Authenticate | No |
| POST | /api/v1/auth/signout | End session | Yes |
| GET | /api/v1/auth/session | Get session | Yes |
| GET | /api/v1/auth/profile | Get profile | Yes |
| PUT | /api/v1/auth/profile | Update profile | Yes |

## Deliverables

- [x] research.md - Technical decisions and alternatives
- [x] data-model.md - Database schema and TypeScript interfaces
- [x] contracts/openapi.yaml - API specification
- [x] quickstart.md - Installation and testing guide
- [ ] tasks.md - Testable tasks (via `/sp.tasks`)

## Next Phase

Run `/sp.tasks` to generate testable implementation tasks from this plan.
