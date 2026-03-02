"""
Better Auth Adapter Routes

Maps better-auth client SDK's expected API paths to our existing AuthService.
better-auth client calls:
  POST /api/auth/sign-up/email
  POST /api/auth/sign-in/email
  POST /api/auth/sign-out
  GET  /api/auth/get-session
"""
import os
from fastapi import APIRouter, Depends, HTTPException, status, Request
from fastapi.responses import JSONResponse
from sqlmodel import Session
from datetime import datetime, timedelta
from pydantic import BaseModel, EmailStr
from typing import Optional
from ...db.dependency import get_db_session
from ...models.user import UserCreate, UserResponse
from ...services.auth_service import AuthService
from ...config.settings import settings

router = APIRouter()


class BetterAuthSignUpRequest(BaseModel):
    email: EmailStr
    password: str
    name: Optional[str] = None
    image: Optional[str] = None


class BetterAuthSignInRequest(BaseModel):
    email: EmailStr
    password: str
    rememberMe: Optional[bool] = True


@router.post("/sign-up/email")
async def better_auth_signup(
    request_body: BetterAuthSignUpRequest,
    request: Request,
    db: Session = Depends(get_db_session),
):
    """Sign up endpoint matching better-auth client SDK expected path"""
    try:
        auth_service = AuthService(db)
        user_create = UserCreate(email=request_body.email, password=request_body.password)
        user_response, session_token = auth_service.create_user(user_create)

        expires_at = datetime.utcnow() + timedelta(days=7)

        # Return in better-auth expected format
        response_data = {
            "user": {
                "id": user_response.id,
                "email": user_response.email,
                "name": request_body.name or user_response.email.split("@")[0],
                "image": request_body.image,
                "emailVerified": True,
                "createdAt": user_response.created_at.isoformat(),
                "updatedAt": user_response.updated_at.isoformat(),
            },
            "session": {
                "id": session_token,
                "token": session_token,
                "userId": user_response.id,
                "expiresAt": expires_at.isoformat(),
            },
        }

        json_response = JSONResponse(content=response_data, status_code=200)
        json_response.set_cookie(
            key="better-auth.session_token",
            value=session_token,
            httponly=True,
            secure=True,
            samesite="none",
            max_age=7 * 24 * 60 * 60,
        )
        return json_response

    except ValueError as e:
        return JSONResponse(
            content={"error": {"message": str(e), "status": 409}},
            status_code=409,
        )
    except Exception as e:
        return JSONResponse(
            content={"error": {"message": f"Error creating user: {str(e)}", "status": 500}},
            status_code=500,
        )


@router.post("/sign-in/email")
async def better_auth_signin(
    request_body: BetterAuthSignInRequest,
    request: Request,
    db: Session = Depends(get_db_session),
):
    """Sign in endpoint matching better-auth client SDK expected path"""
    auth_service = AuthService(db)
    result = auth_service.authenticate_user(request_body.email, request_body.password)

    if not result:
        return JSONResponse(
            content={"error": {"message": "Invalid email or password", "status": 401}},
            status_code=401,
        )

    user_response, session_token = result
    expires_at = datetime.utcnow() + timedelta(days=7)

    response_data = {
        "user": {
            "id": user_response.id,
            "email": user_response.email,
            "name": user_response.email.split("@")[0],
            "image": None,
            "emailVerified": True,
            "createdAt": user_response.created_at.isoformat(),
            "updatedAt": user_response.updated_at.isoformat(),
        },
        "session": {
            "id": session_token,
            "token": session_token,
            "userId": user_response.id,
            "expiresAt": expires_at.isoformat(),
        },
    }

    json_response = JSONResponse(content=response_data)
    json_response.set_cookie(
        key="better-auth.session_token",
        value=session_token,
        httponly=True,
        secure=True,
        samesite="none",
        max_age=7 * 24 * 60 * 60,
    )
    return json_response


@router.post("/sign-out")
async def better_auth_signout(
    request: Request,
    db: Session = Depends(get_db_session),
):
    """Sign out endpoint matching better-auth client SDK expected path"""
    # Try to get token from cookie or Authorization header
    token = request.cookies.get("better-auth.session_token")
    if not token:
        authorization = request.headers.get("Authorization")
        if authorization and authorization.startswith("Bearer "):
            token = authorization.split(" ")[1]

    if token:
        auth_service = AuthService(db)
        auth_service.revoke_session(token)

    json_response = JSONResponse(content={"success": True})
    json_response.delete_cookie(
        key="better-auth.session_token",
        httponly=True,
        secure=True,
        samesite="none",
    )
    return json_response


@router.get("/get-session")
async def better_auth_get_session(
    request: Request,
    db: Session = Depends(get_db_session),
):
    """Get session endpoint matching better-auth client SDK expected path"""
    # Try cookie first, then Authorization header
    token = request.cookies.get("better-auth.session_token")
    if not token:
        authorization = request.headers.get("Authorization")
        if authorization and authorization.startswith("Bearer "):
            token = authorization.split(" ")[1]

    if not token:
        return JSONResponse(content={"session": None, "user": None}, status_code=200)

    auth_service = AuthService(db)
    session_info = auth_service.validate_session(token)

    if not session_info:
        return JSONResponse(content={"session": None, "user": None}, status_code=200)

    # Get user info
    user = auth_service.get_user_by_id(session_info.user_id)
    profile = auth_service.get_user_profile(session_info.user_id)

    return JSONResponse(content={
        "session": {
            "id": session_info.id,
            "token": session_info.token,
            "userId": session_info.user_id,
            "expiresAt": session_info.expires_at.isoformat(),
        },
        "user": {
            "id": user.id if user else session_info.user_id,
            "email": user.email if user else None,
            "name": user.email.split("@")[0] if user else None,
            "image": None,
            "emailVerified": True,
            "createdAt": user.created_at.isoformat() if user else None,
            "updatedAt": user.updated_at.isoformat() if user else None,
            "profile": profile if profile else None,
        },
    })
