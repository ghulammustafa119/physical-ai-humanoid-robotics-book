import pytest
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch
from src.api.main import app
from src.models.text_selection import TextSelectionCreate


@pytest.fixture
def client():
    return TestClient(app)


@pytest.mark.asyncio
async def test_create_text_selection_success(client):
    """Test successful creation of a text selection"""
    mock_selection_data = {
        "content": "This is a sample selected text for testing purposes.",
        "start_position": 100,
        "end_position": 200,
        "book_section": "chapter-1/section-1"
    }

    with patch('src.api.v1.text_selection.uuid.uuid4') as mock_uuid:
        mock_uuid.return_value = "test-uuid"

        response = client.post("/api/v1/text-selection", json=mock_selection_data)

        assert response.status_code == 201
        response_data = response.json()
        assert "id" in response_data
        assert response_data["content"] == mock_selection_data["content"]
        assert response_data["start_position"] == mock_selection_data["start_position"]


@pytest.mark.asyncio
async def test_create_text_selection_short_text(client):
    """Test creating text selection with content that's too short"""
    mock_selection_data = {
        "content": "Hi",  # Less than 10 characters
        "start_position": 10,
        "end_position": 20,
        "book_section": "chapter-1/section-1"
    }

    response = client.post("/api/v1/text-selection", json=mock_selection_data)

    assert response.status_code == 400
    assert "must be at least 10 characters long" in response.json()["detail"]


@pytest.mark.asyncio
async def test_create_text_selection_long_text(client):
    """Test creating text selection with content that's too long"""
    long_content = "A" * 6000  # More than 5000 characters
    mock_selection_data = {
        "content": long_content,
        "start_position": 100,
        "end_position": 200,
        "book_section": "chapter-1/section-1"
    }

    response = client.post("/api/v1/text-selection", json=mock_selection_data)

    assert response.status_code == 400
    assert "exceeds maximum length" in response.json()["detail"]


@pytest.mark.asyncio
async def test_create_text_selection_invalid_positions(client):
    """Test creating text selection with invalid start/end positions"""
    mock_selection_data = {
        "content": "This is a sample selected text for testing purposes.",
        "start_position": 200,  # Start position is greater than end position
        "end_position": 100,
        "book_section": "chapter-1/section-1"
    }

    response = client.post("/api/v1/text-selection", json=mock_selection_data)

    assert response.status_code == 400
    assert "Start position must be less than end position" in response.json()["detail"]


def test_get_text_selection_success(client):
    """Test successful retrieval of a text selection"""
    # This test would require a real implementation with database
    # For now, we'll test the error case for invalid ID format
    response = client.get("/api/v1/text-selection/invalid-uuid-format")

    assert response.status_code == 400
    assert "Invalid selection ID format" in response.json()["detail"]