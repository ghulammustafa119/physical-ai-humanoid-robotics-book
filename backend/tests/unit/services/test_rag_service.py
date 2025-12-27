import pytest
from unittest.mock import AsyncMock, patch, MagicMock
from src.services.rag_service import RAGService
from src.models.response import SourceReference


@pytest.fixture
def rag_service():
    return RAGService()


@pytest.mark.asyncio
async def test_query_with_selected_text_restriction(rag_service):
    """Test that queries with selected text are properly restricted"""
    query = "What does this text say?"
    selected_text = "This is a sample selected text for testing purposes."

    with patch.object(rag_service, '_generate_response_from_selected_text') as mock_gen_response:
        mock_gen_response.return_value = "This is a response based on the selected text."

        with patch.object(rag_service, '_create_sources_from_selected_text') as mock_create_sources:
            mock_source = SourceReference(
                title="Test Source",
                url="/book/test-section",
                page=1,
                section="1.1",
                text_snippet="Test snippet"
            )
            mock_create_sources.return_value = [mock_source]

            response = await rag_service.query_with_selected_text(query, selected_text)

            # Verify that the selected text path was used
            mock_gen_response.assert_called_once_with(query, selected_text, None)
            mock_create_sources.assert_called_once_with(selected_text, None)
            assert response.response_text == "This is a response based on the selected text."


@pytest.mark.asyncio
async def test_query_without_selected_text(rag_service):
    """Test that queries without selected text use full content"""
    query = "What is the main topic?"

    with patch.object(rag_service, '_generate_response_from_full_content') as mock_gen_response:
        mock_gen_response.return_value = "This is a response based on full content."

        with patch.object(rag_service, '_create_sources_from_full_content') as mock_create_sources:
            mock_source = SourceReference(
                title="Full Content Source",
                url="/book/all",
                page=1,
                section="Introduction",
                text_snippet="Full content snippet"
            )
            mock_create_sources.return_value = [mock_source]

            response = await rag_service.query_with_selected_text(query)

            # Verify that the full content path was used
            mock_gen_response.assert_called_once_with(query, None)
            mock_create_sources.assert_called_once_with(None)
            assert response.response_text == "This is a response based on full content."


@pytest.mark.asyncio
async def test_query_validation_empty_query(rag_service):
    """Test that empty queries are rejected"""
    with pytest.raises(ValueError, match="Query cannot be empty"):
        await rag_service.query_with_selected_text("")


@pytest.mark.asyncio
async def test_query_validation_long_query(rag_service):
    """Test that overly long queries are rejected"""
    long_query = "A" * 2000  # Assuming this exceeds max length

    with pytest.raises(ValueError, match="exceeds maximum length"):
        await rag_service.query_with_selected_text(long_query)


@pytest.mark.asyncio
async def test_query_validation_long_selected_text(rag_service):
    """Test that overly long selected text is rejected"""
    query = "What does this text say?"
    long_selected_text = "A" * 10000  # Assuming this exceeds max length

    with pytest.raises(ValueError, match="exceeds maximum length"):
        await rag_service.query_with_selected_text(query, long_selected_text)


def test_validate_text_selection_valid(rag_service):
    """Test validation of valid text selection"""
    from src.models.text_selection import TextSelection
    from datetime import datetime
    import uuid

    valid_selection = TextSelection(
        id=uuid.uuid4(),
        content="This is a valid selection that meets the minimum length requirement.",
        start_position=100,
        end_position=200,
        book_section="chapter-1/section-1",
        user_id=None,
        created_at=datetime.now()
    )

    is_valid = rag_service.validate_text_selection(valid_selection)
    assert is_valid is True


def test_validate_text_selection_too_short(rag_service):
    """Test validation of text selection that's too short"""
    from src.models.text_selection import TextSelection
    from datetime import datetime
    import uuid

    short_selection = TextSelection(
        id=uuid.uuid4(),
        content="Hi",  # Too short
        start_position=10,
        end_position=20,
        book_section="chapter-1/section-1",
        user_id=None,
        created_at=datetime.now()
    )

    is_valid = rag_service.validate_text_selection(short_selection)
    assert is_valid is False


def test_validate_text_selection_invalid_positions(rag_service):
    """Test validation of text selection with invalid start/end positions"""
    from src.models.text_selection import TextSelection
    from datetime import datetime
    import uuid

    invalid_selection = TextSelection(
        id=uuid.uuid4(),
        content="This is a sample text",
        start_position=200,  # Start position is greater than end position
        end_position=100,
        book_section="chapter-1/section-1",
        user_id=None,
        created_at=datetime.now()
    )

    is_valid = rag_service.validate_text_selection(invalid_selection)
    assert is_valid is False


@pytest.mark.asyncio
async def test_generate_response_from_selected_text(rag_service):
    """Test that response generation from selected text works correctly"""
    query = "What does this text say?"
    selected_text = "This is a sample selected text for testing purposes."

    # Mock the OpenAI service call
    with patch('src.services.rag_service.openai_service') as mock_openai_service:
        mock_result = {
            "response_text": "This is a response based on the selected text.",
            "token_usage": {
                "input_tokens": 10,
                "output_tokens": 15,
                "total_tokens": 25
            }
        }
        mock_openai_service.generate_response.return_value = mock_result

        result = await rag_service._generate_response_from_selected_text(query, selected_text)

        # Verify that OpenAI service was called with correct parameters
        mock_openai_service.generate_response.assert_called_once_with(
            query=query,
            context=selected_text,
            selected_text=selected_text
        )
        assert result == "This is a response based on the selected text."


@pytest.mark.asyncio
async def test_generate_response_from_full_content(rag_service):
    """Test that response generation from full content works correctly"""
    query = "What is the main topic?"
    book_section = "chapter-1/section-1"

    # Mock the OpenAI service call
    with patch('src.services.rag_service.openai_service') as mock_openai_service:
        mock_result = {
            "response_text": "This is a response based on the full content.",
            "token_usage": {
                "input_tokens": 8,
                "output_tokens": 12,
                "total_tokens": 20
            }
        }
        mock_openai_service.generate_response.return_value = mock_result

        result = await rag_service._generate_response_from_full_content(query, book_section)

        # Verify that OpenAI service was called with correct parameters
        mock_openai_service.generate_response.assert_called_once_with(
            query=query,
            context="Book section: chapter-1/section-1"
        )
        assert result == "This is a response based on the full content."


def test_create_sources_from_selected_text(rag_service):
    """Test creation of sources from selected text"""
    selected_text = "This is a sample selected text for testing purposes."
    book_section = "chapter-1/section-1"

    sources = rag_service._create_sources_from_selected_text(selected_text, book_section)

    assert len(sources) == 1
    source = sources[0]
    assert source.title == "Mock Source Title"
    assert source.url == "/book/chapter-1/section-1"
    assert source.page == 1
    assert source.section == "chapter-1/section-1"
    assert "This is a mock source reference" in source.text_snippet


def test_create_sources_from_full_content(rag_service):
    """Test creation of sources from full content"""
    book_section = "chapter-1/section-1"

    sources = rag_service._create_sources_from_full_content(book_section)

    assert len(sources) == 1
    source = sources[0]
    assert source.title == "Full Book Content"
    assert source.url == "/book/chapter-1/section-1"
    assert source.page == 1
    assert source.section == "chapter-1/section-1"