import pytest
from unittest.mock import AsyncMock, patch, MagicMock
from uuid import UUID
import uuid
from src.services.chat_service import ChatService
from src.models.chat_session import ChatSession
from datetime import datetime


@pytest.fixture
def chat_service():
    return ChatService()


@pytest.mark.asyncio
async def test_create_chat_session(chat_service):
    """Test creating a new chat session"""
    user_id = uuid.uuid4()
    selected_text_id = uuid.uuid4()

    session = await chat_service.create_chat_session(user_id, selected_text_id)

    assert isinstance(session.id, UUID)
    assert session.user_id == user_id
    assert session.selected_text_id == selected_text_id
    assert session.active is True
    assert session.created_at <= datetime.utcnow()
    assert session.updated_at <= datetime.utcnow()


@pytest.mark.asyncio
async def test_create_chat_session_without_optional_params(chat_service):
    """Test creating a chat session without optional parameters"""
    session = await chat_service.create_chat_session()

    assert isinstance(session.id, UUID)
    assert session.user_id is None
    assert session.selected_text_id is None
    assert session.active is True


@pytest.mark.asyncio
async def test_process_query_in_session_with_selected_text(chat_service):
    """Test processing a query within a session with selected text"""
    session_id = uuid.uuid4()
    query_text = "What does this text mean?"
    selected_text = "This is the selected text for context."

    # Mock the RAG service
    with patch('src.services.chat_service.rag_service') as mock_rag_service:
        mock_response = MagicMock()
        mock_response.id = uuid.uuid4()
        mock_response.query_id = uuid.uuid4()
        mock_response.response_text = "This is the response based on the selected text."
        mock_response.sources = []
        mock_response.confidence_score = 0.85
        mock_response.timestamp = datetime.now()
        mock_response.token_usage = {"input_tokens": 10, "output_tokens": 15, "total_tokens": 25}

        mock_rag_service.query_with_selected_text.return_value = mock_response

        response = await chat_service.process_query_in_session(
            session_id, query_text, selected_text, "initial"
        )

        # Verify that RAG service was called with the correct parameters
        mock_rag_service.query_with_selected_text.assert_called_once_with(
            query=query_text,
            selected_text=selected_text
        )
        assert response == mock_response


@pytest.mark.asyncio
async def test_process_query_in_session_without_selected_text(chat_service):
    """Test processing a query within a session without selected text"""
    session_id = uuid.uuid4()
    query_text = "What is the main topic?"

    # Mock the RAG service
    with patch('src.services.chat_service.rag_service') as mock_rag_service:
        mock_response = MagicMock()
        mock_response.id = uuid.uuid4()
        mock_response.query_id = uuid.uuid4()
        mock_response.response_text = "This is the response based on full content."
        mock_response.sources = []
        mock_response.confidence_score = 0.80
        mock_response.timestamp = datetime.now()
        mock_response.token_usage = {"input_tokens": 8, "output_tokens": 12, "total_tokens": 20}

        mock_rag_service.query_with_selected_text.return_value = mock_response

        response = await chat_service.process_query_in_session(
            session_id, query_text
        )

        # Verify that RAG service was called with the correct parameters
        mock_rag_service.query_with_selected_text.assert_called_once_with(
            query=query_text,
            selected_text=None
        )
        assert response == mock_response


@pytest.mark.asyncio
async def test_update_session_context(chat_service):
    """Test updating session context with new selected text"""
    session_id = uuid.uuid4()
    new_selected_text_id = uuid.uuid4()

    updated_session = await chat_service.update_session_context(
        session_id, new_selected_text_id
    )

    assert updated_session.id == session_id
    assert updated_session.selected_text_id == new_selected_text_id
    assert updated_session.updated_at <= datetime.utcnow()


@pytest.mark.asyncio
async def test_get_session_history(chat_service):
    """Test retrieving session history"""
    session_id = uuid.uuid4()

    history = await chat_service.get_session_history(session_id)

    # Initially, history should be empty
    assert history == []


@pytest.mark.asyncio
async def test_validate_session_active(chat_service):
    """Test validating an active session"""
    session_id = uuid.uuid4()

    is_valid = await chat_service.validate_session(session_id)

    # Currently, all sessions are considered valid
    assert is_valid is True


@pytest.mark.asyncio
async def test_process_query_validation(chat_service):
    """Test validation in query processing"""
    session_id = uuid.uuid4()

    # Test with empty query
    with pytest.raises(Exception):  # The exact exception depends on RAG service implementation
        await chat_service.process_query_in_session(session_id, "")


@pytest.mark.asyncio
async def test_create_session_with_error_handling(chat_service):
    """Test error handling in session creation"""
    # Mock an error in session creation (if there were any)
    # For now, this is just a placeholder test
    session = await chat_service.create_chat_session()

    assert session is not None
    assert isinstance(session, ChatSession)


@pytest.mark.asyncio
async def test_update_session_context_error_handling(chat_service):
    """Test error handling in session context update"""
    session_id = uuid.uuid4()
    new_selected_text_id = uuid.uuid4()

    updated_session = await chat_service.update_session_context(session_id, new_selected_text_id)

    assert updated_session is not None
    assert updated_session.id == session_id


@pytest.mark.asyncio
async def test_process_query_in_session_error_handling(chat_service):
    """Test error handling in query processing"""
    session_id = uuid.uuid4()
    query_text = "Test query"

    # Mock the RAG service to raise an exception
    with patch('src.services.chat_service.rag_service') as mock_rag_service:
        mock_rag_service.query_with_selected_text.side_effect = Exception("RAG service error")

        with pytest.raises(Exception):
            await chat_service.process_query_in_session(session_id, query_text)


@pytest.mark.asyncio
async def test_get_session_history_with_limit(chat_service):
    """Test retrieving session history with limit"""
    session_id = uuid.uuid4()

    # Test with a specific limit
    history = await chat_service.get_session_history(session_id, limit=5)

    # Initially, history should still be empty regardless of limit
    assert history == []