from fastapi import HTTPException, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional
from sqlmodel import Session
from ..db.connection import get_session
from ..services.auth_service import AuthService
from ..models.session import Session as SessionModel
from datetime import datetime


class AuthMiddleware:
    """
    Middleware class for handling authentication and authorization
    """

    def __init__(self):
        self.security = HTTPBearer(auto_error=False)

    async def authenticate_request(self, request: Request) -> Optional[dict]:
        """
        Extract and validate authentication from request
        Returns user info if authenticated, None if not
        """
        # Try to get token from Authorization header
        auth_header = request.headers.get("Authorization")
        if not auth_header or not auth_header.startswith("Bearer "):
            return None

        token = auth_header.split(" ")[1]

        # Validate the session using the token
        with next(get_session()) as db:
            auth_service = AuthService(db)
            session_info = auth_service.validate_session(token)

            if not session_info:
                return None

            # Add user info to request state
            user_info = auth_service.get_user_by_id(session_info.user_id)
            if not user_info:
                return None

            return {
                "user_id": session_info.user_id,
                "session_id": session_info.id,
                "email": user_info.email
            }

    async def require_auth(self, request: Request) -> dict:
        """
        Require authentication, raise exception if not authenticated
        """
        user_info = await self.authenticate_request(request)

        if not user_info:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Not authenticated"
            )

        return user_info


# FastAPI dependency to extract user info from request
async def get_current_user(request: Request) -> Optional[dict]:
    """
    FastAPI dependency to get current authenticated user
    Usage in endpoints:
    ```
    @app.get("/protected")
    async def protected_endpoint(current_user: dict = Depends(get_current_user)):
        return {"message": f"Hello {current_user['email']}"}
    ```
    """
    auth_middleware = AuthMiddleware()
    return await auth_middleware.authenticate_request(request)


async def require_current_user(request: Request) -> dict:
    """
    FastAPI dependency to require authentication
    Usage in endpoints:
    ```
    @app.get("/protected")
    async def protected_endpoint(current_user: dict = Depends(require_current_user)):
        return {"message": f"Hello {current_user['email']}"}
    ```
    """
    auth_middleware = AuthMiddleware()
    return await auth_middleware.require_auth(request)


# Example of how to use the middleware in a FastAPI app
"""
from fastapi import FastAPI, Depends
from typing import Optional

app = FastAPI()

@app.get("/public")
async def public_endpoint():
    return {"message": "This is public"}

@app.get("/protected")
async def protected_endpoint(
    current_user: dict = Depends(require_current_user)
):
    return {"message": f"Hello {current_user['email']}", "user_id": current_user['user_id']}

@app.get("/optional")
async def optional_auth_endpoint(
    current_user: Optional[dict] = Depends(get_current_user)
):
    if current_user:
        return {"message": f"Hello {current_user['email']}", "authenticated": True}
    else:
        return {"message": "Hello guest", "authenticated": False}
"""


# Alternative implementation as an actual middleware class that can be added to FastAPI
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import Response


class AuthenticationMiddleware(BaseHTTPMiddleware):
    """
    Starlette middleware implementation that can be added to FastAPI
    Usage:
    ```
    app.add_middleware(AuthenticationMiddleware)
    ```
    """

    async def dispatch(self, request: Request, call_next):
        # Extract and validate authentication
        auth_header = request.headers.get("Authorization")

        user_info = None
        if auth_header and auth_header.startswith("Bearer "):
            token = auth_header.split(" ")[1]

            # Validate the session using the token
            with next(get_session()) as db:
                auth_service = AuthService(db)
                session_info = auth_service.validate_session(token)

                if session_info:
                    user_info = auth_service.get_user_by_id(session_info.user_id)
                    if user_info:
                        # Store user info in request state
                        request.state.user = {
                            "user_id": session_info.user_id,
                            "session_id": session_info.id,
                            "email": user_info.email
                        }

        # Continue with the request
        response = await call_next(request)
        return response


# Example usage in main.py:
"""
from fastapi import FastAPI
from backend.src.middleware.auth import AuthenticationMiddleware

app = FastAPI()

# Add the authentication middleware
app.add_middleware(AuthenticationMiddleware)

# Now you can access request.state.user in your endpoints
@app.get("/profile")
async def get_profile(request: Request):
    if hasattr(request.state, 'user') and request.state.user:
        return {"user": request.state.user}
    else:
        return {"message": "Not authenticated"}
"""