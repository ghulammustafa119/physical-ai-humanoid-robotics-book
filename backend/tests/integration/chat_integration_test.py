import pytest
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch
from src.api.main import app
from src.services.rag_service import rag_service


@pytest.fixture
def client():
    return TestClient(app)


@pytest.mark.asyncio
async def test_complete_chat_flow_with_text_selection(client):
    """Test the complete flow: text selection → chat query → response with sources"""
    # First, create a text selection
    selection_data = {
        "content": "RAG (Retrieval Augmented Generation) is a technique that combines information retrieval with text generation. It allows language models to access external knowledge sources to provide more accurate and up-to-date responses.",
        "start_position": 100,
        "end_position": 300,
        "book_section": "chapter-1/section-1"
    }

    with patch('src.api.v1.text_selection.uuid.uuid4') as mock_uuid:
        mock_uuid.return_value = "test-selection-uuid"

        selection_response = client.post("/api/v1/text-selection", json=selection_data)
        assert selection_response.status_code == 201

    # Then, use that selected text in a chat query
    chat_request = {
        "query": "What is RAG?",
        "selected_text": selection_data["content"],
        "book_section": "chapter-1/section-1"
    }

    with patch.object(rag_service, 'query_with_selected_text') as mock_query:
        mock_response = {
            "id": "test-id",
            "query_id": "test-query-id",
            "response_text": "RAG (Retrieval Augmented Generation) is a technique that combines information retrieval with text generation.",
            "sources": [
                {
                    "title": "Chapter 1: Introduction to RAG",
                    "url": "/book/chapter-1/section-1",
                    "page": 5,
                    "section": "1.1",
                    "text_snippet": "RAG (Retrieval Augmented Generation) is a technique..."
                }
            ],
            "confidence_score": 0.9,
            "timestamp": "2023-10-20T10:30:00Z",
            "token_usage": {
                "input_tokens": 10,
                "output_tokens": 20,
                "total_tokens": 30
            }
        }
        mock_query.return_value = mock_response

        chat_response = client.post("/api/v1/chat", json=chat_request)

        assert chat_response.status_code == 200
        response_data = chat_response.json()
        assert "RAG" in response_data["response_text"]
        assert len(response_data["sources"]) == 1
        assert response_data["confidence_score"] == 0.9


@pytest.mark.asyncio
async def test_chat_flow_without_selected_text(client):
    """Test chat flow without selected text (full content search)"""
    chat_request = {
        "query": "What is the main topic of this book?",
        "book_section": "chapter-1/section-1"
    }

    with patch.object(rag_service, 'query_with_selected_text') as mock_query:
        mock_response = {
            "id": "test-id",
            "query_id": "test-query-id",
            "response_text": "The main topic of this book is Physical AI and Humanoid Robotics.",
            "sources": [
                {
                    "title": "Full Book Content",
                    "url": "/book/chapter-1/section-1",
                    "page": 1,
                    "section": "Introduction",
                    "text_snippet": "Physical AI and Humanoid Robotics..."
                }
            ],
            "confidence_score": 0.85,
            "timestamp": "2023-10-20T10:30:00Z",
            "token_usage": {
                "input_tokens": 8,
                "output_tokens": 15,
                "total_tokens": 23
            }
        }
        mock_query.return_value = mock_response

        chat_response = client.post("/api/v1/chat", json=chat_request)

        assert chat_response.status_code == 200
        response_data = chat_response.json()
        assert "Physical AI" in response_data["response_text"]
        assert response_data["confidence_score"] == 0.85


@pytest.mark.asyncio
async def test_error_handling_in_chat_flow(client):
    """Test error handling in the chat flow"""
    # Test with empty query
    chat_request = {
        "query": "",
        "selected_text": "Some text for context"
    }

    chat_response = client.post("/api/v1/chat", json=chat_request)

    assert chat_response.status_code == 400
    assert "Query cannot be empty" in chat_response.json()["detail"]


@pytest.mark.asyncio
async def test_large_text_selection_handling(client):
    """Test handling of large text selections"""
    large_content = "A" * 6000  # This exceeds the max length
    selection_data = {
        "content": large_content,
        "start_position": 100,
        "end_position": 6100,
        "book_section": "chapter-1/section-1"
    }

    selection_response = client.post("/api/v1/text-selection", json=selection_data)

    # The API should handle this gracefully, possibly by truncating
    assert selection_response.status_code in [201, 400]  # Either success with truncation or error


@pytest.mark.asyncio
async def test_unrelated_query_handling(client):
    """Test handling of queries unrelated to selected text"""
    selection_data = {
        "content": "This text is about RAG (Retrieval Augmented Generation) techniques.",
        "start_position": 100,
        "end_position": 200,
        "book_section": "chapter-1/section-1"
    }

    with patch('src.api.v1.text_selection.uuid.uuid4') as mock_uuid:
        mock_uuid.return_value = "test-selection-uuid"

        selection_response = client.post("/api/v1/text-selection", json=selection_data)
        assert selection_response.status_code == 201

    # Query unrelated to the selected text
    chat_request = {
        "query": "What is the weather like today?",
        "selected_text": selection_data["content"],
        "book_section": "chapter-1/section-1"
    }

    with patch.object(rag_service, 'query_with_selected_text') as mock_query:
        # Mock a response that indicates the query is unrelated
        mock_response = {
            "id": "test-id",
            "query_id": "test-query-id",
            "response_text": "Based on the selected text, I cannot provide a relevant answer to: 'What is the weather like today?'. The selected text does not contain information related to your query.",
            "sources": [
                {
                    "title": "Mock Source Title",
                    "url": "/book/chapter-1/section-1",
                    "page": 1,
                    "section": "1.1",
                    "text_snippet": "This text is about RAG (Retrieval Augmented Generation) techniques."
                }
            ],
            "confidence_score": 0.1,
            "timestamp": "2023-10-20T10:30:00Z",
            "token_usage": {
                "input_tokens": 8,
                "output_tokens": 35,
                "total_tokens": 43
            }
        }
        mock_query.return_value = mock_response

        chat_response = client.post("/api/v1/chat", json=chat_request)

        assert chat_response.status_code == 200
        response_data = chat_response.json()
        # The response should indicate that the query is unrelated to the selected text
        assert "cannot provide a relevant answer" in response_data["response_text"]
        assert response_data["confidence_score"] == 0.1  # Low confidence


def test_health_check_endpoint(client):
    """Test the health check endpoint"""
    response = client.get("/health")

    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"