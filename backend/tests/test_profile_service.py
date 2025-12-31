"""
Unit tests for the profile service
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from datetime import datetime
from backend.src.services.profile_service import ProfileService
from backend.src.models.profile import UserProfile
from backend.src.models.user import User
from sqlmodel import Session
import uuid


@pytest.fixture
def mock_db_session():
    """Mock database session for testing"""
    session = MagicMock(spec=Session)
    return session


@pytest.fixture
def profile_service():
    """Create an instance of ProfileService for testing"""
    return ProfileService()


@pytest.mark.asyncio
async def test_create_profile_success(profile_service, mock_db_session):
    """Test successful profile creation"""
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

    with patch.object(profile_service, '_create_profile_in_db', return_value=mock_profile):
        result = await profile_service.create_profile(user_id, profile_data, mock_db_session)

        assert result.user_id == user_id
        assert result.programming_level == "intermediate"
        assert result.simulator_experience == ["gazebo", "isaac_sim"]


@pytest.mark.asyncio
async def test_update_profile_success(profile_service, mock_db_session):
    """Test successful profile update"""
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

    with patch.object(profile_service, '_update_profile_in_db', return_value=mock_profile):
        result = await profile_service.update_profile(user_id, update_data, mock_db_session)

        assert result.user_id == user_id
        assert result.programming_level == "advanced"


@pytest.mark.asyncio
async def test_get_profile_by_user_id_success(profile_service, mock_db_session):
    """Test successful profile retrieval by user ID"""
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

    with patch.object(profile_service, '_get_profile_by_user_id', return_value=mock_profile):
        result = await profile_service.get_profile_by_user_id(user_id, mock_db_session)

        assert result.user_id == user_id
        assert result.programming_level == "intermediate"
        assert result.simulator_experience == ["gazebo", "isaac_sim"]


@pytest.mark.asyncio
async def test_get_profile_by_user_id_not_found(profile_service, mock_db_session):
    """Test profile retrieval for non-existent user"""
    user_id = uuid.uuid4()

    with patch.object(profile_service, '_get_profile_by_user_id', return_value=None):
        result = await profile_service.get_profile_by_user_id(user_id, mock_db_session)

        assert result is None


@pytest.mark.asyncio
async def test_delete_profile_success(profile_service, mock_db_session):
    """Test successful profile deletion"""
    user_id = uuid.uuid4()

    with patch.object(profile_service, '_delete_profile_by_user_id', return_value=True):
        result = await profile_service.delete_profile(user_id, mock_db_session)

        assert result is True


@pytest.mark.asyncio
async def test_delete_profile_not_found(profile_service, mock_db_session):
    """Test profile deletion for non-existent profile"""
    user_id = uuid.uuid4()

    with patch.object(profile_service, '_delete_profile_by_user_id', return_value=False):
        result = await profile_service.delete_profile(user_id, mock_db_session)

        assert result is False


def test_calculate_profile_completeness_complete():
    """Test profile completeness calculation for complete profile"""
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

    completeness = ProfileService.calculate_profile_completeness(profile_data)

    # All required fields are present, so completeness should be 1.0 (100%)
    assert completeness == 1.0


def test_calculate_profile_completeness_partial():
    """Test profile completeness calculation for partial profile"""
    profile_data = {
        "programming_level": "intermediate",
        "python_level": "strong",
        # Missing other fields
        "simulator_experience": ["gazebo"]
    }

    completeness = ProfileService.calculate_profile_completeness(profile_data)

    # Only 3 out of 8 fields are present, so completeness should be 3/8 = 0.375
    assert completeness == 0.375


def test_calculate_profile_completeness_empty():
    """Test profile completeness calculation for empty profile"""
    profile_data = {}

    completeness = ProfileService.calculate_profile_completeness(profile_data)

    # No fields are present, so completeness should be 0.0
    assert completeness == 0.0


def test_calculate_profile_completeness_with_none_values():
    """Test profile completeness calculation with None values"""
    profile_data = {
        "programming_level": "intermediate",
        "python_level": None,
        "ai_ml_level": "applied",
        "robotics_level": "",
        "system_type": "desktop",
        "gpu_availability": "nvidia_cuda",
        "hardware_access": None,
        "simulator_experience": []
    }

    completeness = ProfileService.calculate_profile_completeness(profile_data)

    # Only 4 fields have valid values (not None or empty), so completeness should be 4/8 = 0.5
    assert completeness == 0.5


@pytest.mark.asyncio
async def test_update_profile_partial_data(profile_service, mock_db_session):
    """Test profile update with partial data"""
    user_id = uuid.uuid4()
    update_data = {
        "programming_level": "advanced",
        "python_level": "strong"
        # Only updating 2 fields, not all fields
    }

    # Original profile data
    original_profile = UserProfile(
        id=uuid.uuid4(),
        user_id=user_id,
        programming_level="intermediate",
        python_level="basic",
        ai_ml_level="applied",
        robotics_level="practical",
        system_type="desktop",
        gpu_availability="nvidia_cuda",
        hardware_access="real",
        simulator_experience=["gazebo"]
    )

    # Updated profile data
    updated_profile = UserProfile(
        id=original_profile.id,
        user_id=user_id,
        programming_level="advanced",  # Updated
        python_level="strong",  # Updated
        ai_ml_level="applied",  # Unchanged
        robotics_level="practical",  # Unchanged
        system_type="desktop",  # Unchanged
        gpu_availability="nvidia_cuda",  # Unchanged
        hardware_access="real",  # Unchanged
        simulator_experience=["gazebo"]  # Unchanged
    )

    with patch.object(profile_service, '_update_profile_in_db', return_value=updated_profile):
        result = await profile_service.update_profile(user_id, update_data, mock_db_session)

        assert result.user_id == user_id
        assert result.programming_level == "advanced"
        assert result.python_level == "strong"
        assert result.ai_ml_level == "applied"  # Should remain unchanged
        assert result.simulator_experience == ["gazebo"]  # Should remain unchanged


@pytest.mark.asyncio
async def test_create_profile_with_defaults(profile_service, mock_db_session):
    """Test profile creation with minimal data (using defaults)"""
    user_id = uuid.uuid4()
    profile_data = {
        "programming_level": "beginner"
        # Only providing one field, others should have defaults
    }

    mock_profile = UserProfile(
        id=uuid.uuid4(),
        user_id=user_id,
        programming_level="beginner",
        python_level="none",  # Default
        ai_ml_level="none",  # Default
        robotics_level="none",  # Default
        system_type="laptop",  # Default
        gpu_availability="none",  # Default
        hardware_access="none",  # Default
        simulator_experience=[]  # Default
    )

    with patch.object(profile_service, '_create_profile_in_db', return_value=mock_profile):
        result = await profile_service.create_profile(user_id, profile_data, mock_db_session)

        assert result.user_id == user_id
        assert result.programming_level == "beginner"
        assert result.python_level == "none"
        assert result.ai_ml_level == "none"
        assert result.simulator_experience == []


@pytest.mark.asyncio
async def test_get_or_create_profile_exists(profile_service, mock_db_session):
    """Test get_or_create_profile when profile already exists"""
    user_id = uuid.uuid4()

    existing_profile = UserProfile(
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

    with patch.object(profile_service, '_get_profile_by_user_id', return_value=existing_profile):
        result = await profile_service.get_or_create_profile(user_id, mock_db_session)

        assert result.user_id == user_id
        assert result.programming_level == "intermediate"


@pytest.mark.asyncio
async def test_get_or_create_profile_new(profile_service, mock_db_session):
    """Test get_or_create_profile when profile doesn't exist (should create)"""
    user_id = uuid.uuid4()

    # First call returns None (profile doesn't exist)
    # Second call returns the created profile
    created_profile = UserProfile(
        id=uuid.uuid4(),
        user_id=user_id,
        programming_level="beginner",  # Default
        python_level="none",  # Default
        ai_ml_level="none",  # Default
        robotics_level="none",  # Default
        system_type="laptop",  # Default
        gpu_availability="none",  # Default
        hardware_access="none",  # Default
        simulator_experience=[]  # Default
    )

    with patch.object(profile_service, '_get_profile_by_user_id', side_effect=[None, created_profile]), \
         patch.object(profile_service, '_create_profile_in_db', return_value=created_profile):
        result = await profile_service.get_or_create_profile(user_id, mock_db_session)

        assert result.user_id == user_id
        assert result.programming_level == "beginner"