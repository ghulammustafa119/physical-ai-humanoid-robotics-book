# Quickstart: User Authentication & Personalization

**Feature**: 002-user-auth | **Date**: 2025-12-30

## Prerequisites

- Node.js 20+ with npm
- Python 3.11+ with pip
- Neon Serverless Postgres URL (existing)
- Better Auth library

## Installation

### Backend Dependencies

```bash
cd backend
pip install better-auth pydantic
```

### Frontend Dependencies

```bash
cd physical-ai-book
npm install better-auth
```

## Environment Variables

### Backend (.env)

```bash
# Existing variables (already set)
DATABASE_URL=postgresql://...

# Add these
BETTER_AUTH_SECRET=your-secret-key-min-32-chars
BETTER_AUTH_URL=http://localhost:3000
```

### Frontend (physical-ai-book/docusaurus.config.ts)

```typescript
// Add better-auth plugin configuration
import { betterAuth } from "better-auth/react";

export default defineConfig({
  // ... existing config

  themes: {
    myTheme: {
      // ...
    }
  },

  plugins: [
    betterAuth({
      baseURL: "http://localhost:3000", // Your Docusaurus URL
      apiPath: "/api/auth",
    }),
  ],
});
```

## Database Setup

```bash
# Run migrations (Better Auth creates tables automatically)
cd backend
python -m scripts.migrate_auth
```

## Running the Application

### Development

```bash
# Terminal 1: Backend
cd backend
python main.py

# Terminal 2: Frontend
cd physical-ai-book
npm start
```

### Access

- Frontend: http://localhost:3000/physical-ai-humanoid-robotics-book/
- API Docs: http://localhost:8000/docs

## Testing Authentication

### Sign Up

```bash
curl -X POST http://localhost:8000/api/v1/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePass123",
    "profile": {
      "programmingLevel": "intermediate",
      "pythonLevel": "strong",
      "aiMlLevel": "basic",
      "roboticsLevel": "none",
      "systemType": "laptop",
      "gpuAvailability": "integrated",
      "hardwareAccess": "simulators",
      "simulatorExperience": ["gazebo"]
    }
  }'
```

### Sign In

```bash
curl -X POST http://localhost:8000/api/v1/auth/signin \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePass123"
  }'
```

### Get Profile (with session token)

```bash
curl -X GET http://localhost:8000/api/v1/auth/profile \
  -H "Authorization: Bearer YOUR_SESSION_TOKEN"
```

## Verify Personalization

1. Sign up with profile data
2. Open the book chat widget
3. Ask: "How do I create a ROS 2 node?"
4. Response should be tailored to your skill level

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Session not persisting | Check cookies are enabled |
| CORS errors | Verify ALLOWED_ORIGINS includes frontend URL |
| Database connection | Verify DATABASE_URL is correct |
| Profile not updating | Ensure you're authenticated with valid session |
