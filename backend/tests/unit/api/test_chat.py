import pytest
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch
from src.api.main import app


@pytest.fixture
def client():
    return TestClient(app)


@pytest.mark.asyncio
async def test_chat_endpoint_with_selected_text(client):
    """Test chat endpoint with selected text restriction"""
    mock_request = {
        "query": "What does this text say?",
        "selected_text": "This is a sample selected text for testing purposes.",
        "book_section": "chapter-1/section-1"
    }

    with patch('src.services.rag_service.rag_service.query_with_selected_text') as mock_query:
        mock_response = {
            "id": "test-id",
            "query_id": "test-query-id",
            "response_text": "This is a test response based on the selected text.",
            "sources": [
                {
                    "title": "Test Source",
                    "url": "/book/chapter-1/section-1",
                    "page": 1,
                    "section": "1.1"
                }
            ],
            "confidence_score": 0.85,
            "timestamp": "2023-10-20T10:30:00Z",
            "token_usage": {
                "input_tokens": 10,
                "output_tokens": 15,
                "total_tokens": 25
            }
        }
        mock_query.return_value = mock_response

        response = client.post("/api/v1/chat", json=mock_request)

        assert response.status_code == 200
        response_data = response.json()
        assert response_data["response_text"] == mock_response["response_text"]
        assert len(response_data["sources"]) == 1


@pytest.mark.asyncio
async def test_chat_endpoint_without_selected_text(client):
    """Test chat endpoint without selected text (full content search)"""
    mock_request = {
        "query": "What is the main topic of this chapter?",
        "book_section": "chapter-1/section-1"
    }

    with patch('src.services.rag_service.rag_service.query_with_selected_text') as mock_query:
        mock_response = {
            "id": "test-id",
            "query_id": "test-query-id",
            "response_text": "This is a test response based on the full content.",
            "sources": [
                {
                    "title": "Full Content Source",
                    "url": "/book/chapter-1",
                    "page": 1,
                    "section": "Introduction"
                }
            ],
            "confidence_score": 0.80,
            "timestamp": "2023-10-20T10:30:00Z",
            "token_usage": {
                "input_tokens": 8,
                "output_tokens": 12,
                "total_tokens": 20
            }
        }
        mock_query.return_value = mock_response

        response = client.post("/api/v1/chat", json=mock_request)

        assert response.status_code == 200
        response_data = response.json()
        assert response_data["response_text"] == mock_response["response_text"]


@pytest.mark.asyncio
async def test_chat_endpoint_empty_query(client):
    """Test chat endpoint with empty query"""
    mock_request = {
        "query": "",
        "selected_text": "This is a sample selected text for testing purposes."
    }

    response = client.post("/api/v1/chat", json=mock_request)

    assert response.status_code == 400
    assert "Query cannot be empty" in response.json()["detail"]


@pytest.mark.asyncio
async def test_chat_endpoint_long_query(client):
    """Test chat endpoint with query that's too long"""
    long_query = "A" * 2000  # Assuming this exceeds max length
    mock_request = {
        "query": long_query,
        "selected_text": "This is a sample selected text for testing purposes."
    }

    response = client.post("/api/v1/chat", json=mock_request)

    assert response.status_code == 400
    assert "exceeds maximum length" in response.json()["detail"]


def test_get_session(client):
    """Test getting a specific session"""
    response = client.get("/api/v1/chat/sessions/test-session-id")

    assert response.status_code == 200
    response_data = response.json()
    assert "session_id" in response_data


def test_list_sessions(client):
    """Test listing sessions"""
    response = client.get("/api/v1/chat/sessions")

    assert response.status_code == 200
    response_data = response.json()
    assert "sessions" in response_data