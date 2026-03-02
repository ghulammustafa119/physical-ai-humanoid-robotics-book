import os
from fastapi import APIRouter, Depends, HTTPException, status, Header, Request
from fastapi.responses import JSONResponse
from sqlmodel import Session
from typing import Optional
from datetime import datetime, timedelta
from ...db.dependency import get_db_session
from ...models.user import UserCreate, UserResponse, UserLogin
from ...models.profile import UserProfileCreate, UserProfileResponse, UserProfileUpdate
from ...services.auth_service import AuthService
from ...config.settings import settings


router = APIRouter()


@router.post("/auth/signup")
async def signup(
    user_create: UserCreate,
    db: Session = Depends(get_db_session)
):
    """
    Create a new user account
    """
    try:
        auth_service = AuthService(db)
        user_response, session_token = auth_service.create_user(user_create)

        expires_at = datetime.utcnow() + timedelta(days=7)

        user_dict = user_response.dict()
        user_dict["created_at"] = user_dict["created_at"].isoformat()
        user_dict["updated_at"] = user_dict["updated_at"].isoformat()

        response_data = {
            "user": user_dict,
            "session": {
                "token": session_token,
                "expires_at": expires_at.isoformat()
            },
            "profile_completeness": 0.0
        }

        json_response = JSONResponse(
            content=response_data,
            status_code=status.HTTP_201_CREATED,
        )

        # Set session cookie for cross-domain auth
        json_response.set_cookie(
            key="session_token",
            value=session_token,
            httponly=True,
            secure=True,
            samesite="none",
            max_age=7 * 24 * 60 * 60,  # 7 days
        )

        return json_response
    except ValueError as e:
        # Email already exists
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=str(e)
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating user: {str(e)}"
        )


@router.post("/auth/signin")
async def signin(
    user_login: UserLogin,
    db: Session = Depends(get_db_session)
):
    """
    Authenticate user with email and password
    """
    auth_service = AuthService(db)
    result = auth_service.authenticate_user(user_login.email, user_login.password)

    if not result:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid credentials"
        )

    user_response, session_token = result

    expires_at = datetime.utcnow() + timedelta(days=7)

    user_dict = user_response.dict()
    user_dict["created_at"] = user_dict["created_at"].isoformat()
    user_dict["updated_at"] = user_dict["updated_at"].isoformat()

    response_data = {
        "user": user_dict,
        "session": {
            "token": session_token,
            "expires_at": expires_at.isoformat()
        },
        "profile_completeness": 0.0
    }

    json_response = JSONResponse(
        content=response_data,
    )

    # Set session cookie for cross-domain auth
    json_response.set_cookie(
        key="session_token",
        value=session_token,
        httponly=True,
        secure=True,
        samesite="none",
        max_age=7 * 24 * 60 * 60,  # 7 days
    )

    return json_response


@router.post("/auth/signout")
async def signout(
    token: str,  # This would typically come from Authorization header or cookie
    db: Session = Depends(get_db_session)
):
    """
    End user session (sign out)
    """
    auth_service = AuthService(db)
    success = auth_service.revoke_session(token)

    if not success:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired session"
        )

    return {"message": "Successfully signed out"}


@router.get("/auth/session")
async def get_session(
    token: str,  # This would typically come from Authorization header
    db: Session = Depends(get_db_session)
):
    """
    Get current session information
    """
    auth_service = AuthService(db)
    session_info = auth_service.validate_session(token)

    if not session_info:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired session"
        )

    return session_info


@router.get("/auth/profile", response_model=UserProfileResponse)
async def get_profile(
    request: Request,
    db: Session = Depends(get_db_session)
):
    """
    Get user profile information using session token from Authorization header
    """
    authorization = request.headers.get("Authorization")
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Session token required in Authorization header"
        )

    token = authorization.split(" ")[1]
    auth_service = AuthService(db)
    session_info = auth_service.validate_session(token)

    if not session_info:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired session"
        )

    profile = auth_service.get_user_profile(session_info.user_id)

    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User profile not found"
        )

    # Convert dict to UserProfileResponse
    return UserProfileResponse(
        id=profile['id'],
        user_id=profile['user_id'],
        programming_level=profile['programming_level'],
        python_level=profile['python_level'],
        ai_ml_level=profile['ai_ml_level'],
        robotics_level=profile['robotics_level'],
        system_type=profile['system_type'],
        gpu_availability=profile['gpu_availability'],
        hardware_access=profile['hardware_access'],
        simulator_experience=profile['simulator_experience'],
        profile_completeness=profile['profile_completeness'],
        created_at=profile['created_at'],
        updated_at=profile['updated_at']
    )


@router.put("/auth/profile", response_model=UserProfileResponse)
async def update_profile(
    request: Request,
    profile_update: UserProfileUpdate,
    db: Session = Depends(get_db_session)
):
    """
    Update user profile information using token from Authorization header
    """
    authorization = request.headers.get("Authorization")
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Session token required in Authorization header"
        )

    token = authorization.split(" ")[1]
    auth_service = AuthService(db)
    session_info = auth_service.validate_session(token)

    if not session_info:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired session"
        )

    # Convert Pydantic model to dict for update
    profile_data = profile_update.dict(exclude_unset=True)

    updated_profile = auth_service.update_user_profile(session_info.user_id, profile_data)

    if not updated_profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User profile not found"
        )

    # Convert dict to UserProfileResponse
    return UserProfileResponse(
        id=updated_profile['id'],
        user_id=updated_profile['user_id'],
        programming_level=updated_profile['programming_level'],
        python_level=updated_profile['python_level'],
        ai_ml_level=updated_profile['ai_ml_level'],
        robotics_level=updated_profile['robotics_level'],
        system_type=updated_profile['system_type'],
        gpu_availability=updated_profile['gpu_availability'],
        hardware_access=updated_profile['hardware_access'],
        simulator_experience=updated_profile['simulator_experience'],
        profile_completeness=updated_profile['profile_completeness'],
        created_at=updated_profile['created_at'],
        updated_at=updated_profile['updated_at']
    )


# Additional endpoints that might be useful:

@router.get("/auth/user", response_model=UserResponse)
async def get_user(
    user_id: str,  # This would typically be extracted from validated token
    db: Session = Depends(get_db_session)
):
    """
    Get user information
    """
    auth_service = AuthService(db)
    user = auth_service.get_user_by_id(user_id)

    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    return user


# Note: In a real implementation, you would use middleware to extract user_id from token
# and pass it to the endpoint functions automatically.
# The token would come from the Authorization header like:
# Authorization: Bearer <token>
# And you would create middleware to validate the token and extract user info.