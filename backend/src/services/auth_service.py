import bcrypt
from datetime import datetime, timedelta
from typing import Optional, Tuple
import secrets
import os
from sqlmodel import Session, select
from ..models.user import User, UserCreate, UserResponse
from ..models.session import Session as SessionModel, SessionCreate, SessionResponse
from ..models.profile import UserProfile, UserProfileCreate


class AuthService:
    """
    Service class for handling authentication operations
    """

    def __init__(self, db_session: Session):
        self.db = db_session

    def hash_password(self, password: str) -> str:
        """
        Hash a password using bcrypt
        """
        salt = bcrypt.gensalt()
        hashed = bcrypt.hashpw(password.encode('utf-8'), salt)
        return hashed.decode('utf-8')

    def verify_password(self, plain_password: str, hashed_password: str) -> bool:
        """
        Verify a plain password against a hashed password
        """
        return bcrypt.checkpw(plain_password.encode('utf-8'), hashed_password.encode('utf-8'))

    def generate_session_token(self) -> str:
        """
        Generate a secure session token
        """
        return secrets.token_urlsafe(32)

    def create_user(self, user_create: UserCreate) -> Tuple[UserResponse, str]:
        """
        Create a new user with email validation and password hashing
        Returns the created user and a session token
        """
        # Check if user already exists
        existing_user = self.db.exec(
            select(User).where(User.email == user_create.email)
        ).first()

        if existing_user:
            raise ValueError("Email already registered")

        # Hash the password
        hashed_password = self.hash_password(user_create.password)

        # Create the user
        user = User(
            email=user_create.email,
            password_hash=hashed_password
        )

        self.db.add(user)
        self.db.commit()
        self.db.refresh(user)

        # Create a session token
        session_token = self.generate_session_token()

        # Create session
        session = SessionModel(
            user_id=user.id,
            token=session_token,
            expires_at=datetime.utcnow() + timedelta(days=7)  # 7 days expiry
        )

        self.db.add(session)
        self.db.commit()

        # Create empty profile for the user
        profile = UserProfile(
            user_id=user.id,
            profile_completeness=0.0  # Initially incomplete
        )

        self.db.add(profile)
        self.db.commit()

        return UserResponse(
            id=user.id,
            email=user.email,
            created_at=user.created_at,
            updated_at=user.updated_at
        ), session_token

    def authenticate_user(self, email: str, password: str) -> Optional[Tuple[UserResponse, str]]:
        """
        Authenticate a user with email and password
        Returns user and session token if successful, None otherwise
        """
        user = self.db.exec(
            select(User).where(User.email == email)
        ).first()

        if not user or not self.verify_password(password, user.password_hash):
            return None

        # Create a new session
        session_token = self.generate_session_token()
        session = SessionModel(
            user_id=user.id,
            token=session_token,
            expires_at=datetime.utcnow() + timedelta(days=7)  # 7 days expiry
        )

        self.db.add(session)
        self.db.commit()

        return UserResponse(
            id=user.id,
            email=user.email,
            created_at=user.created_at,
            updated_at=user.updated_at
        ), session_token

    def validate_session(self, token: str) -> Optional[SessionResponse]:
        """
        Validate a session token and return session info if valid
        """
        session = self.db.exec(
            select(SessionModel).where(SessionModel.token == token)
        ).first()

        if not session or session.expires_at < datetime.utcnow():
            return None

        return SessionResponse(
            id=session.id,
            user_id=session.user_id,
            token=session.token,
            expires_at=session.expires_at,
            created_at=session.created_at
        )

    def revoke_session(self, token: str) -> bool:
        """
        Revoke (delete) a session by token
        """
        session = self.db.exec(
            select(SessionModel).where(SessionModel.token == token)
        ).first()

        if session:
            self.db.delete(session)
            self.db.commit()
            return True

        return False

    def get_user_by_id(self, user_id: str) -> Optional[UserResponse]:
        """
        Get user by ID
        """
        user = self.db.exec(
            select(User).where(User.id == user_id)
        ).first()

        if not user:
            return None

        return UserResponse(
            id=user.id,
            email=user.email,
            created_at=user.created_at,
            updated_at=user.updated_at
        )

    def get_user_by_email(self, email: str) -> Optional[UserResponse]:
        """
        Get user by email
        """
        user = self.db.exec(
            select(User).where(User.email == email)
        ).first()

        if not user:
            return None

        return UserResponse(
            id=user.id,
            email=user.email,
            created_at=user.created_at,
            updated_at=user.updated_at
        )

    def get_user_profile(self, user_id: str) -> Optional[dict]:
        """
        Get user profile information
        """
        profile = self.db.exec(
            select(UserProfile).where(UserProfile.user_id == user_id)
        ).first()

        if not profile:
            return None

        return {
            'id': profile.id,
            'user_id': profile.user_id,
            'programming_level': profile.programming_level,
            'python_level': profile.python_level,
            'ai_ml_level': profile.ai_ml_level,
            'robotics_level': profile.robotics_level,
            'system_type': profile.system_type,
            'gpu_availability': profile.gpu_availability,
            'hardware_access': profile.hardware_access,
            'simulator_experience': profile.simulator_experience,
            'profile_completeness': profile.profile_completeness,
            'created_at': profile.created_at,
            'updated_at': profile.updated_at
        }

    def update_user_profile(self, user_id: str, profile_data: dict) -> Optional[dict]:
        """
        Update user profile information
        """
        profile = self.db.exec(
            select(UserProfile).where(UserProfile.user_id == user_id)
        ).first()

        if not profile:
            return None

        # Update fields that are provided
        for field, value in profile_data.items():
            if hasattr(profile, field):
                setattr(profile, field, value)

        # Recalculate completeness score
        total_fields = 8
        filled_fields = 0
        if profile.programming_level: filled_fields += 1
        if profile.python_level: filled_fields += 1
        if profile.ai_ml_level: filled_fields += 1
        if profile.robotics_level: filled_fields += 1
        if profile.system_type: filled_fields += 1
        if profile.gpu_availability: filled_fields += 1
        if profile.hardware_access: filled_fields += 1
        if profile.simulator_experience and len(profile.simulator_experience) > 0: filled_fields += 1

        profile.profile_completeness = round(filled_fields / total_fields, 2)

        self.db.add(profile)
        self.db.commit()
        self.db.refresh(profile)

        return {
            'id': profile.id,
            'user_id': profile.user_id,
            'programming_level': profile.programming_level,
            'python_level': profile.python_level,
            'ai_ml_level': profile.ai_ml_level,
            'robotics_level': profile.robotics_level,
            'system_type': profile.system_type,
            'gpu_availability': profile.gpu_availability,
            'hardware_access': profile.hardware_access,
            'simulator_experience': profile.simulator_experience,
            'profile_completeness': profile.profile_completeness,
            'created_at': profile.created_at,
            'updated_at': profile.updated_at
        }


# Example usage:
# from backend.src.db.connection import get_session
#
# with next(get_session()) as session:
#     auth_service = AuthService(session)
#
#     # Create user
#     user_create = UserCreate(email="test@example.com", password="SecurePass123")
#     user, token = auth_service.create_user(user_create)
#
#     # Authenticate user
#     result = auth_service.authenticate_user("test@example.com", "SecurePass123")
#     if result:
#         user, token = result
#         print(f"Authenticated user: {user.email}")