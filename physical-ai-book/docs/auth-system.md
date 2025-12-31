# User Authentication & Personalization System

This document describes the user authentication and personalization system implemented for the Physical AI and Humanoid Robotics Book Assistant.

## Overview

The system provides:
- User registration and authentication
- Profile management with background information collection
- Personalization context injection into the RAG chatbot
- Secure session management

## Architecture

### Backend Components

#### 1. Database Schema
- **User Table**: Stores user credentials (email, password hash)
- **Session Table**: Manages user sessions with tokens and expiration
- **User Profile Table**: Stores user background information

#### 2. Services
- **Auth Service**: Handles user registration, authentication, and session management
- **Profile Service**: Manages user profile CRUD operations
- **Personalization Service**: Creates context objects for RAG integration
- **RAG Service**: Modified to accept and inject personalization context

#### 3. API Endpoints
- `POST /api/v1/auth/signup` - User registration
- `POST /api/v1/auth/signin` - User authentication
- `POST /api/v1/auth/signout` - User logout
- `GET /api/v1/auth/profile` - Get user profile
- `PUT /api/v1/auth/profile` - Update user profile

### Frontend Components

#### 1. Authentication Components
- **SignUp**: Registration form with optional profile questions
- **SignIn**: Login form
- **Profile**: User profile management with edit capabilities
- **AuthProvider**: Context provider for authentication state

#### 2. Navigation
- **AuthNavigation**: Conditional navigation based on authentication status

## User Registration Flow

1. User accesses the signup page
2. User provides email and password
3. User optionally provides background information:
   - Programming experience level
   - Python experience level
   - AI/ML experience level
   - Robotics experience level
   - System type (laptop, desktop, cloud)
   - GPU availability
   - Hardware access
   - Simulator experience (Gazebo, Isaac Sim, Unity)
4. Backend creates user account and profile
5. Session token is generated and stored in localStorage

## Authentication Flow

1. User provides credentials on sign-in page
2. Backend validates credentials against stored hash
3. If valid, a new session is created
4. Session token is returned and stored in localStorage
5. User is redirected to protected content

## Profile Management

Users can view and update their profile information:
- Access profile page after authentication
- Click "Edit Profile" button
- Update any fields
- Save changes to update profile

## Personalization Context

The system collects user background information to provide personalized responses:

### Context Fields
- `programming_level`: User's programming experience
- `python_level`: User's Python experience
- `ai_ml_level`: User's AI/ML experience
- `robotics_level`: User's robotics experience
- `system_type`: User's system type
- `gpu_availability`: User's GPU availability
- `hardware_access`: User's hardware access
- `simulator_experience`: List of simulators user has experience with
- `profile_completeness`: Percentage of profile completion

### RAG Integration
The personalization context is injected into chatbot prompts to provide tailored responses based on user background.

## Security Features

- Passwords are hashed using bcrypt
- Sessions use secure tokens
- API endpoints are protected by authentication middleware
- Password requirements: minimum 8 characters

## API Usage Examples

### Register a New User
```bash
curl -X POST http://localhost:8000/api/v1/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "user@example.com",
    "password": "SecurePassword123!",
    "profile": {
      "programming_level": "intermediate",
      "python_level": "strong",
      "ai_ml_level": "applied",
      "robotics_level": "practical",
      "system_type": "desktop",
      "gpu_availability": "nvidia_cuda",
      "hardware_access": "real",
      "simulator_experience": ["gazebo", "isaac_sim"]
    }
  }'
```

### Authenticate User
```bash
curl -X POST http://localhost:8000/api/v1/auth/signin \
  -H "Content-Type: application/json" \
  -d '{
    "email": "user@example.com",
    "password": "SecurePassword123!"
  }'
```

### Get User Profile
```bash
curl -X GET http://localhost:8000/api/v1/auth/profile \
  -H "Authorization: Bearer <session_token>"
```

### Update User Profile
```bash
curl -X PUT http://localhost:8000/api/v1/auth/profile \
  -H "Authorization: Bearer <session_token>" \
  -H "Content-Type: application/json" \
  -d '{
    "programming_level": "advanced"
  }'
```

## Frontend Integration

### Using AuthProvider
```tsx
import { AuthProvider, useAuth } from './components/auth/AuthProvider';

// Wrap your app with AuthProvider
function App() {
  return (
    <AuthProvider>
      {/* Your app content */}
    </AuthProvider>
  );
}

// Use authentication context in components
function MyComponent() {
  const { user, loading, signIn, signOut } = useAuth();

  if (loading) return <div>Loading...</div>;

  return (
    <div>
      {user ? (
        <div>Welcome, {user.email}!</div>
      ) : (
        <button onClick={() => signIn('email', 'password')}>
          Sign In
        </button>
      )}
    </div>
  );
}
```

## Testing

### Unit Tests
- Backend services are tested with pytest
- API endpoints are tested with FastAPI TestClient
- Frontend components can be tested with React Testing Library

### E2E Tests
- Playwright tests for complete authentication flows
- API tests for backend endpoints

## Environment Variables

The system uses the following environment variables:

```env
# Database
DATABASE_URL=postgresql://user:password@localhost/dbname

# Authentication
SECRET_KEY=your-secret-key-here
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30

# AI Services
GOOGLE_GEMINI_API_KEY=your-gemini-api-key
```

## Database Migrations

The initial database schema is created via the migration file:
- `backend/src/db/migrations/001_create_auth_tables.sql`

## Profile Completeness Calculation

The system calculates profile completeness as the percentage of required fields filled out:
- 8 main profile fields are considered
- Completeness = (number of filled fields) / (total fields) * 100
- Used to encourage users to complete their profiles