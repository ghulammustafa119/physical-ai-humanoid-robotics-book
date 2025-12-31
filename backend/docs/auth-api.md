# Authentication API Documentation

This document describes the authentication API endpoints for the Physical AI and Humanoid Robotics Book Assistant.

## Base URL

`http://localhost:8000/api/v1/auth`

## Endpoints

### POST /signup

Register a new user account with optional profile information.

#### Request

**Content-Type:** `application/json`

**Body:**
```json
{
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
}
```

#### Profile Fields

All profile fields are optional when signing up.

- `programming_level`: (string) User's programming experience level - "beginner", "intermediate", "advanced"
- `python_level`: (string) User's Python experience level - "none", "basic", "strong"
- `ai_ml_level`: (string) User's AI/ML experience level - "none", "basic", "applied"
- `robotics_level`: (string) User's robotics experience level - "none", "academic", "practical"
- `system_type`: (string) User's system type - "laptop", "desktop", "cloud"
- `gpu_availability`: (string) User's GPU availability - "none", "integrated", "nvidia_cuda"
- `hardware_access`: (string) User's hardware access - "none", "simulators", "real"
- `simulator_experience`: (array) List of simulators user has experience with - "gazebo", "isaac_sim", "unity"

#### Response

**Success (200):**
```json
{
  "user": {
    "id": "uuid-string",
    "email": "user@example.com"
  },
  "session": {
    "token": "jwt-token-string"
  }
}
```

**Error (400, 409, 500):**
```json
{
  "detail": "Error message"
}
```

#### Errors

- `400 Bad Request`: Invalid request format or validation error
- `409 Conflict`: User with email already exists
- `500 Internal Server Error`: Server error during registration

---

### POST /signin

Authenticate an existing user.

#### Request

**Content-Type:** `application/json`

**Body:**
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123!"
}
```

#### Response

**Success (200):**
```json
{
  "user": {
    "id": "uuid-string",
    "email": "user@example.com",
    "profile": {
      "id": "uuid-string",
      "user_id": "uuid-string",
      "programming_level": "intermediate",
      "python_level": "strong",
      "ai_ml_level": "applied",
      "robotics_level": "practical",
      "system_type": "desktop",
      "gpu_availability": "nvidia_cuda",
      "hardware_access": "real",
      "simulator_experience": ["gazebo", "isaac_sim"],
      "profile_completeness": 0.875,
      "created_at": "2023-01-01T00:00:00Z",
      "updated_at": "2023-01-01T00:00:00Z"
    }
  },
  "session": {
    "token": "jwt-token-string"
  }
}
```

**Error (400, 401, 500):**
```json
{
  "detail": "Error message"
}
```

#### Errors

- `400 Bad Request`: Invalid request format
- `401 Unauthorized`: Invalid credentials
- `500 Internal Server Error`: Server error during authentication

---

### POST /signout

End the current user session.

#### Request

**Headers:**
```
Authorization: Bearer {session_token}
Content-Type: application/json
```

#### Response

**Success (200):**
```json
{
  "message": "Successfully signed out"
}
```

**Error (401, 500):**
```json
{
  "detail": "Error message"
}
```

#### Errors

- `401 Unauthorized`: Invalid or expired session token
- `500 Internal Server Error`: Server error during signout

---

### GET /profile

Get the current user's profile information.

#### Request

**Headers:**
```
Authorization: Bearer {session_token}
Content-Type: application/json
```

#### Response

**Success (200):**
```json
{
  "id": "uuid-string",
  "user_id": "uuid-string",
  "programming_level": "intermediate",
  "python_level": "strong",
  "ai_ml_level": "applied",
  "robotics_level": "practical",
  "system_type": "desktop",
  "gpu_availability": "nvidia_cuda",
  "hardware_access": "real",
  "simulator_experience": ["gazebo", "isaac_sim"],
  "profile_completeness": 0.875,
  "created_at": "2023-01-01T00:00:00Z",
  "updated_at": "2023-01-01T00:00:00Z"
}
```

**Error (401, 404, 500):**
```json
{
  "detail": "Error message"
}
```

#### Errors

- `401 Unauthorized`: Invalid or expired session token
- `404 Not Found`: User profile not found
- `500 Internal Server Error`: Server error during profile retrieval

---

### PUT /profile

Update the current user's profile information.

#### Request

**Headers:**
```
Authorization: Bearer {session_token}
Content-Type: application/json
```

**Body (all fields optional):**
```json
{
  "programming_level": "advanced",
  "python_level": "strong",
  "ai_ml_level": "applied",
  "robotics_level": "practical",
  "system_type": "desktop",
  "gpu_availability": "nvidia_cuda",
  "hardware_access": "real",
  "simulator_experience": ["gazebo", "isaac_sim", "unity"]
}
```

#### Response

**Success (200):**
```json
{
  "id": "uuid-string",
  "user_id": "uuid-string",
  "programming_level": "advanced",
  "python_level": "strong",
  "ai_ml_level": "applied",
  "robotics_level": "practical",
  "system_type": "desktop",
  "gpu_availability": "nvidia_cuda",
  "hardware_access": "real",
  "simulator_experience": ["gazebo", "isaac_sim", "unity"],
  "profile_completeness": 1.0,
  "created_at": "2023-01-01T00:00:00Z",
  "updated_at": "2023-01-02T00:00:00Z"
}
```

**Error (400, 401, 500):**
```json
{
  "detail": "Error message"
}
```

#### Errors

- `400 Bad Request`: Invalid request format or validation error
- `401 Unauthorized`: Invalid or expired session token
- `500 Internal Server Error`: Server error during profile update

## Authentication Headers

All authenticated endpoints require the following header:

```
Authorization: Bearer {session_token}
```

## Session Management

- Session tokens are JWTs with a default expiration of 24 hours
- Tokens are stored in HTTP-only cookies for security
- Use the signout endpoint to properly end sessions
- Expired tokens will result in 401 Unauthorized responses

## Error Handling

All error responses follow this format:

```json
{
  "detail": "Human-readable error message"
}
```

Common HTTP status codes:
- `200 OK`: Request successful
- `400 Bad Request`: Client error in request format
- `401 Unauthorized`: Authentication required or failed
- `404 Not Found`: Requested resource not found
- `409 Conflict`: Request conflicts with current state
- `500 Internal Server Error`: Server error

## Rate Limiting

Authentication endpoints may be subject to rate limiting to prevent abuse. Exceeding rate limits will result in `429 Too Many Requests` responses.

## Security

- Passwords must be at least 8 characters
- All passwords are hashed using bcrypt with 12 rounds
- Sessions use secure, HTTP-only tokens
- All API communication should be over HTTPS in production