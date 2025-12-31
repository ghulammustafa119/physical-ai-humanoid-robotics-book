"""
Unit tests for the authentication service
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from datetime import datetime, timedelta
from backend.src.services.auth_service import AuthService
from backend.src.models.user import UserCreate, UserLogin
from backend.src.db.connection import get_session
from sqlmodel import SQLModel, create_engine, Session
from backend.src.models.user import User
from backend.src.models.session import Session as UserSession
from backend.src.models.profile import UserProfile
from passlib.context import CryptContext
import uuid


@pytest.fixture
def mock_db_session():
    """Mock database session for testing"""
    session = MagicMock(spec=Session)
    return session


@pytest.fixture
def auth_service():
    """Create an instance of AuthService for testing"""
    return AuthService()


@pytest.mark.asyncio
async def test_create_user_success(auth_service, mock_db_session):
    """Test successful user creation"""
    # Mock user data
    user_create = UserCreate(
        email="test@example.com",
        password="password123"
    )

    # Mock database responses
    mock_user = User(
        id=uuid.uuid4(),
        email="test@example.com",
        password_hash="$2b$12$..."
    )

    with patch.object(auth_service, '_hash_password', return_value="$2b$12$..."), \
         patch.object(auth_service, '_create_user_in_db', return_value=mock_user):

        result = await auth_service.create_user(user_create, mock_db_session)

        assert result.email == "test@example.com"
        assert result.password_hash == "$2b$12$..."


@pytest.mark.asyncio
async def test_authenticate_user_success(auth_service, mock_db_session):
    """Test successful user authentication"""
    # Mock user data
    user_login = UserLogin(
        email="test@example.com",
        password="password123"
    )

    # Mock user in database
    mock_user = User(
        id=uuid.uuid4(),
        email="test@example.com",
        password_hash="$2b$12$..."  # This is a valid bcrypt hash
    )

    # Mock password verification to return True
    with patch.object(auth_service, '_verify_password', return_value=True), \
         patch.object(auth_service, '_get_user_by_email', return_value=mock_user):

        result = await auth_service.authenticate_user(user_login, mock_db_session)

        assert result is not None
        assert result.email == "test@example.com"


@pytest.mark.asyncio
async def test_authenticate_user_invalid_credentials(auth_service, mock_db_session):
    """Test authentication with invalid credentials"""
    # Mock user data
    user_login = UserLogin(
        email="test@example.com",
        password="wrongpassword"
    )

    # Mock user in database
    mock_user = User(
        id=uuid.uuid4(),
        email="test@example.com",
        password_hash="$2b$12$..."  # This is a valid bcrypt hash
    )

    # Mock password verification to return False
    with patch.object(auth_service, '_verify_password', return_value=False), \
         patch.object(auth_service, '_get_user_by_email', return_value=mock_user):

        result = await auth_service.authenticate_user(user_login, mock_db_session)

        assert result is None


@pytest.mark.asyncio
async def test_authenticate_user_user_not_found(auth_service, mock_db_session):
    """Test authentication when user doesn't exist"""
    # Mock user data
    user_login = UserLogin(
        email="nonexistent@example.com",
        password="password123"
    )

    # Mock no user found
    with patch.object(auth_service, '_get_user_by_email', return_value=None):

        result = await auth_service.authenticate_user(user_login, mock_db_session)

        assert result is None


@pytest.mark.asyncio
async def test_create_session_success(auth_service, mock_db_session):
    """Test successful session creation"""
    user_id = uuid.uuid4()

    with patch.object(auth_service, '_create_session_in_db') as mock_create_session:
        mock_session = UserSession(
            id=uuid.uuid4(),
            user_id=user_id,
            token="test_token",
            expires_at=datetime.utcnow() + timedelta(hours=24)
        )
        mock_create_session.return_value = mock_session

        result = await auth_service.create_session(user_id, mock_db_session)

        assert result.user_id == user_id
        assert result.token is not None


@pytest.mark.asyncio
async def test_get_session_valid(auth_service, mock_db_session):
    """Test getting a valid session"""
    session_token = "valid_session_token"
    user_id = uuid.uuid4()

    mock_session = UserSession(
        id=uuid.uuid4(),
        user_id=user_id,
        token=session_token,
        expires_at=datetime.utcnow() + timedelta(hours=1)
    )

    with patch.object(auth_service, '_get_session_by_token', return_value=mock_session):
        result = await auth_service.get_session(session_token, mock_db_session)

        assert result is not None
        assert result.user_id == user_id


@pytest.mark.asyncio
async def test_get_session_expired(auth_service, mock_db_session):
    """Test getting an expired session"""
    session_token = "expired_session_token"

    mock_session = UserSession(
        id=uuid.uuid4(),
        user_id=uuid.uuid4(),
        token=session_token,
        expires_at=datetime.utcnow() - timedelta(hours=1)  # Expired
    )

    with patch.object(auth_service, '_get_session_by_token', return_value=mock_session):
        result = await auth_service.get_session(session_token, mock_db_session)

        assert result is None


@pytest.mark.asyncio
async def test_get_session_not_found(auth_service, mock_db_session):
    """Test getting a non-existent session"""
    session_token = "nonexistent_session_token"

    with patch.object(auth_service, '_get_session_by_token', return_value=None):
        result = await auth_service.get_session(session_token, mock_db_session)

        assert result is None


@pytest.mark.asyncio
async def test_logout_user_success(auth_service, mock_db_session):
    """Test successful user logout"""
    session_token = "valid_session_token"

    with patch.object(auth_service, '_delete_session_by_token', return_value=True):
        result = await auth_service.logout_user(session_token, mock_db_session)

        assert result is True


@pytest.mark.asyncio
async def test_logout_user_not_found(auth_service, mock_db_session):
    """Test logout with non-existent session"""
    session_token = "nonexistent_session_token"

    with patch.object(auth_service, '_delete_session_by_token', return_value=False):
        result = await auth_service.logout_user(session_token, mock_db_session)

        assert result is False


def test_hash_password(auth_service):
    """Test password hashing"""
    password = "testpassword123"

    hashed = auth_service._hash_password(password)

    # Verify that the hash is not the same as the original password
    assert hashed != password
    # Verify that the hash starts with the bcrypt prefix
    assert hashed.startswith('$2b$')


def test_verify_password_correct(auth_service):
    """Test password verification with correct password"""
    password = "testpassword123"
    hashed = auth_service._hash_password(password)

    result = auth_service._verify_password(password, hashed)

    assert result is True


def test_verify_password_incorrect(auth_service):
    """Test password verification with incorrect password"""
    password = "testpassword123"
    wrong_password = "wrongpassword123"
    hashed = auth_service._hash_password(password)

    result = auth_service._verify_password(wrong_password, hashed)

    assert result is False


@pytest.mark.asyncio
async def test_create_user_profile_success(auth_service, mock_db_session):
    """Test successful user profile creation"""
    user_id = uuid.uuid4()
    profile_data = {
        "programming_level": "intermediate",
        "python_level": "strong",
        "ai_ml_level": "applied",
        "robotics_level": "practical",
        "system_type": "desktop",
        "gpu_availability": "nvidia_cuda",
        "hardware_access": "real",
        "simulator_experience": ["gazebo", "isaac_sim"]
    }

    mock_profile = UserProfile(
        id=uuid.uuid4(),
        user_id=user_id,
        **profile_data
    )

    with patch.object(auth_service, '_create_profile_in_db', return_value=mock_profile):
        result = await auth_service.create_user_profile(user_id, profile_data, mock_db_session)

        assert result.user_id == user_id
        assert result.programming_level == "intermediate"
        assert result.simulator_experience == ["gazebo", "isaac_sim"]


@pytest.mark.asyncio
async def test_update_user_profile_success(auth_service, mock_db_session):
    """Test successful user profile update"""
    user_id = uuid.uuid4()
    update_data = {"programming_level": "advanced"}

    mock_profile = UserProfile(
        id=uuid.uuid4(),
        user_id=user_id,
        programming_level="advanced",  # Updated value
        python_level="strong",
        ai_ml_level="applied",
        robotics_level="practical",
        system_type="desktop",
        gpu_availability="nvidia_cuda",
        hardware_access="real",
        simulator_experience=["gazebo", "isaac_sim"]
    )

    with patch.object(auth_service, '_update_profile_in_db', return_value=mock_profile):
        result = await auth_service.update_user_profile(user_id, update_data, mock_db_session)

        assert result.user_id == user_id
        assert result.programming_level == "advanced"


@pytest.mark.asyncio
async def test_get_user_profile_success(auth_service, mock_db_session):
    """Test successful user profile retrieval"""
    user_id = uuid.uuid4()

    mock_profile = UserProfile(
        id=uuid.uuid4(),
        user_id=user_id,
        programming_level="intermediate",
        python_level="strong",
        ai_ml_level="applied",
        robotics_level="practical",
        system_type="desktop",
        gpu_availability="nvidia_cuda",
        hardware_access="real",
        simulator_experience=["gazebo", "isaac_sim"]
    )

    with patch.object(auth_service, '_get_profile_by_user_id', return_value=mock_profile):
        result = await auth_service.get_user_profile(user_id, mock_db_session)

        assert result.user_id == user_id
        assert result.programming_level == "intermediate"
        assert result.simulator_experience == ["gazebo", "isaac_sim"]