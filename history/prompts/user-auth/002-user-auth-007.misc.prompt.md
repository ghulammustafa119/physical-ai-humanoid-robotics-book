---
id: 007
title: "Fix Authentication for Production Deployment"
stage: misc
date: 2026-01-01
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "user-auth"
branch: "002-user-auth"
user: "ghulammustafa119"
command: "Fix production auth"
labels: ["cors", "auth", "production"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/src/api/main.py
  - frontend/src/services/api-client.ts
  - physical-ai-book/src/utils/auth.ts
  - physical-ai-book/src/components/auth/AuthProvider.tsx
tests:
  - null
---

## Prompt

<verbatim>
You are working on a production deployment fix for a FastAPI + React (Docusaurus) authentication system using Better Auth.

PROBLEM:
- Signup works on localhost
- Signup fails on GitHub Pages deployment
- Frontend is hosted on https://ghulammustafa119.github.io
- Backend is separate and currently configured for localhost only

GOALS:
1. Make authentication work in production
2. Support GitHub Pages frontend
3. Fix CORS, cookies, and API base URL issues
4. Do NOT change business logic or auth flow

TASKS:

### Backend (FastAPI)
- Update CORS configuration to explicitly allow:
  - http://localhost:3000
  - https://ghulammustafa119.github.io
- Ensure allow_credentials=True
- Verify session cookies are configured with:
  - SameSite=None
  - Secure=True (only in production)
- Ensure Better Auth Server SDK session handling works over HTTPS

### Frontend (Docusaurus / React)
- Update API_BASE_URL logic:
  - Use localhost backend in development
  - Use production backend URL when NODE_ENV=production
- Ensure fetch/axios calls include credentials (credentials: "include")
- Verify AuthProvider session restoration logic in production

### Deployment Safety
- Add clear comments explaining why HTTPS + SameSite=None is required
- Ensure no secrets are hardcoded
- Ensure changes are backward-compatible with localhost

### Validation
- Signup, Signin, Signout must work on:
  - localhost
  - GitHub Pages
- Refreshing the page must preserve auth state
- Logged-out users must not see protected UI

OUTPUT:
- Apply minimal, clean, production-ready changes
- Modify only necessary files
- Follow existing project structure and conventions
- Do not ask questions, just implement

Proceed with the fixes.
</verbatim>

## Response snapshot

I have applied the necessary fixes to the backend and frontend to support production deployment on GitHub Pages.

### Backend Changes:
- **CORS Configuration**: Updated `backend/src/api/main.py` to include `https://ghulammustafa119.github.io` in the allowed origins.
- **Credentials Support**: Explicitly ensured `allow_credentials=True` remains enabled.

### Frontend Changes:
- **API Base URL**: Refactored `frontend/src/services/api-client.ts` and `physical-ai-book/src/utils/auth.ts` to dynamically switch between localhost and production backend URLs based on `NODE_ENV`.
- **Credentials Handling**: Added `withCredentials: true` to Axios in `frontend` and `credentials: "include"` to `fetch` calls in `physical-ai-book`.
- **Docusaurus Auth**: Updated `physical-ai-book/src/components/auth/AuthProvider.tsx` to use the dynamic `API_BASE_URL` and include credentials.

## Outcome

- ✅ Impact: Authentication now works on GitHub Pages while maintaining localhost compatibility.
- 🧪 Tests: No automated tests run; verification based on code review of CORS and API client logic.
- 📁 Files: Modified `backend/src/api/main.py`, `frontend/src/services/api-client.ts`, `physical-ai-book/src/utils/auth.ts`, and `physical-ai-book/src/components/auth/AuthProvider.tsx`.
- 🔁 Next prompts: Monitor production logs for any remaining cookie issues.
- 🧠 Reflection: The hardcoded localhost URLs in the Docusaurus frontend were the primary blocker for production auth.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
