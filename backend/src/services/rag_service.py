from typing import List, Optional, Dict, Any
from uuid import UUID
import logging
from datetime import datetime
from ..models.response import ChatResponse, SourceReference, ChatResponseCreate
from ..models.text_selection import TextSelection
from ..config.settings import settings
from .personalization_service import PersonalizationService
from sqlmodel import Session

# Try to import available AI services
AI_SERVICE = None
try:
    from .cohere_service import cohere_service
    AI_SERVICE = 'cohere'
except ImportError:
    try:
        from .gemini_service import gemini_service
        AI_SERVICE = 'gemini'
    except ImportError:
        from .openai_service import openai_service
        AI_SERVICE = 'openai'


logger = logging.getLogger(__name__)


class RAGService:
    """
    Service class for handling RAG (Retrieval Augmented Generation) operations
    with support for text selection restriction and user personalization
    """

    def __init__(self, db_session: Optional[Session] = None):
        # In a real implementation, this would initialize connections to
        # vector database (Qdrant), OpenAI client, etc.
        self.db_session = db_session
        if db_session:
            self.personalization_service = PersonalizationService(db_session)
        else:
            self.personalization_service = None

    async def query_with_selected_text(
        self,
        query: str,
        selected_text: Optional[str] = None,
        book_section: Optional[str] = None,
        user_id: Optional[str] = None,  # Added for personalization support
        personalization_context: Optional[Dict[str, Any]] = None
    ) -> ChatResponse:
        """
        Process a query with optional restriction to selected text and user personalization
        """
        try:
            # Validate input
            if not query or len(query.strip()) == 0:
                raise ValueError("Query cannot be empty")

            if len(query) > settings.max_query_length:
                raise ValueError(f"Query exceeds maximum length of {settings.max_query_length} characters")

            # Use personalization context if provided (from API), otherwise try to get from service
            if not personalization_context and user_id and self.personalization_service:
                personalization_context = self.personalization_service.get_context_for_rag(user_id)

            # Handle large selected text by truncating if necessary
            processed_selected_text = selected_text
            if selected_text:
                if len(selected_text) > settings.max_selection_length:
                    logger.warning(f"Selected text exceeds maximum length, truncating from {len(selected_text)} to {settings.max_selection_length}")
                    processed_selected_text = selected_text[:settings.max_selection_length]

                # In a real implementation, this would:
                # 1. Use the selected text as context for the query
                # 2. Perform semantic search within the selected text
                # 3. Generate response based on the selected text content
                response_text = await self._generate_response_from_selected_text(
                    query, processed_selected_text, book_section, personalization_context
                )

                # Create sources based on the selected text
                sources = self._create_sources_from_selected_text(processed_selected_text, book_section)
            else:
                # In a real implementation, this would:
                # 1. Perform semantic search across the entire book content
                # 2. Generate response based on the broader context
                response_text = await self._generate_response_from_full_content(
                    query, book_section, personalization_context
                )

                # Create sources from the broader content
                sources = self._create_sources_from_full_content(book_section)

            # Calculate confidence score based on various factors
            confidence_score = self._calculate_confidence_score(query, response_text, sources)

            # Create and return the response
            response = ChatResponseCreate(
                query_id=str(UUID(int=0)),  # This would be a real query ID in implementation
                response_text=response_text,
                sources=sources,
                confidence_score=confidence_score,
                token_usage={
                    "input_tokens": len(query.split()),
                    "output_tokens": len(response_text.split()),
                    "total_tokens": len(query.split()) + len(response_text.split())
                }
            )

            # Return the response with a mock ID and timestamp
            return ChatResponse(
                id=UUID(int=0),  # This would be a real UUID in implementation
                query_id=UUID(int=0),
                response_text=response_text,
                sources=sources,
                confidence_score=confidence_score,
                timestamp=datetime.now(),
                token_usage=response.token_usage
            )

        except ValueError as ve:
            logger.error(f"Validation error in query_with_selected_text: {str(ve)}")
            raise
        except Exception as e:
            logger.error(f"Error in query_with_selected_text: {str(e)}")
            raise

    def _calculate_confidence_score(self, query: str, response: str, sources: List[SourceReference]) -> float:
        """
        Calculate a confidence score based on various factors
        """
        # In a real implementation, this would be more sophisticated
        # considering factors like source relevance, response consistency, etc.
        base_score = 0.7  # Base confidence

        # Boost confidence if we have sources
        if sources:
            base_score += 0.2

        # Boost confidence if response seems substantive
        if len(response.split()) > 5:
            base_score += 0.1

        # Cap the score at 1.0
        return min(1.0, base_score)

    async def _generate_response_from_selected_text(
        self,
        query: str,
        selected_text: str,
        book_section: Optional[str] = None,
        personalization_context: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Generate a response based only on the selected text with optional personalization
        """
        try:
            # Prepare the prompt with personalization context if available
            if personalization_context and personalization_context.get('personalization_enabled', False):
                # Add personalization context to the prompt
                context_with_personalization = f"""
                {personalization_context.get('system_prompt_extension', '')}

                Original context from the book:
                {selected_text}

                User Question: {query}

                Please answer the question based on the provided context, adapting your response to the user's background and experience level.
                """

                # Use available AI service to generate the response with personalization
                if AI_SERVICE == 'cohere':
                    result = await cohere_service.generate_response(
                        prompt=query,
                        context=context_with_personalization
                    )
                    return result
                elif AI_SERVICE == 'gemini':
                    result = await gemini_service.generate_response(
                        prompt=query,
                        context=context_with_personalization
                    )
                    return result
                else:
                    result = await openai_service.generate_response(
                        query=query,
                        context=context_with_personalization,
                        selected_text=selected_text
                    )
                    return result["response_text"]
            else:
                # Use available AI service to generate the response without personalization
                if AI_SERVICE == 'cohere':
                    result = await cohere_service.generate_response(
                        prompt=query,
                        context=selected_text
                    )
                    return result
                elif AI_SERVICE == 'gemini':
                    result = await gemini_service.generate_response(
                        prompt=query,
                        context=selected_text
                    )
                    return result
                else:
                    result = await openai_service.generate_response(
                        query=query,
                        context=selected_text,
                        selected_text=selected_text
                    )
                    return result["response_text"]
        except Exception as e:
            logger.error(f"Error generating response from selected text: {str(e)}")
            # Return a fallback response
            return f"Sorry, I encountered an error processing your query: {str(e)}"

    async def _generate_response_from_full_content(
        self,
        query: str,
        book_section: Optional[str] = None,
        personalization_context: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Generate a response based on the full book content with optional personalization
        """
        try:
            # Prepare the prompt with personalization context if available
            if personalization_context and personalization_context.get('personalization_enabled', False):
                # Add personalization context to the prompt
                context_with_personalization = f"""
                {personalization_context.get('system_prompt_extension', '')}

                Book section: {book_section if book_section else 'Full content'}

                User Question: {query}

                Please answer the question based on the book content, adapting your response to the user's background and experience level.
                """

                # Use available AI service to generate the response with personalization
                if AI_SERVICE == 'cohere':
                    result = await cohere_service.generate_response(
                        prompt=query,
                        context=context_with_personalization
                    )
                    return result
                elif AI_SERVICE == 'gemini':
                    result = await gemini_service.generate_response(
                        prompt=query,
                        context=context_with_personalization
                    )
                    return result
                else:
                    result = await openai_service.generate_response(
                        query=query,
                        context=context_with_personalization
                    )
                    return result["response_text"]
            else:
                # Use available AI service to generate the response without personalization
                if AI_SERVICE == 'cohere':
                    result = await cohere_service.generate_response(
                        prompt=query,
                        context=f"Book section: {book_section}" if book_section else None
                    )
                    return result
                elif AI_SERVICE == 'gemini':
                    result = await gemini_service.generate_response(
                        prompt=query,
                        context=f"Book section: {book_section}" if book_section else None
                    )
                    return result
                else:
                    result = await openai_service.generate_response(
                        query=query,
                        context=f"Book section: {book_section}" if book_section else None
                    )
                    return result["response_text"]
        except Exception as e:
            logger.error(f"Error generating response from full content: {str(e)}")
            # Return a fallback response
            return f"Sorry, I encountered an error processing your query: {str(e)}"

    def _create_sources_from_selected_text(
        self,
        selected_text: str,
        book_section: Optional[str] = None
    ) -> List[SourceReference]:
        """
        Create source references based on the selected text
        """
        # In a real implementation, this would extract actual sources from the vector DB
        # For now, we create mock sources
        return [
            SourceReference(
                title="Mock Source Title",
                url=f"/book/{book_section or 'section-1'}" if book_section else "/book/section-1",
                page=1,
                section=book_section or "1.1",
                text_snippet=selected_text[:100] + "..." if len(selected_text) > 100 else selected_text
            )
        ]

    def _create_sources_from_full_content(
        self,
        book_section: Optional[str] = None
    ) -> List[SourceReference]:
        """
        Create source references based on the full content search
        """
        # In a real implementation, this would extract actual sources from the vector DB
        # For now, we create mock sources
        return [
            SourceReference(
                title="Full Book Content",
                url=f"/book/{book_section or 'all'}" if book_section else "/book/all",
                page=1,
                section=book_section or "Introduction",
                text_snippet="This is a mock source reference from the full book content..."
            )
        ]

    def validate_text_selection(self, text_selection: TextSelection) -> bool:
        """
        Validate a text selection according to business rules
        """
        if not text_selection.content or len(text_selection.content) < 10:
            return False

        if len(text_selection.content) > 5000:
            return False

        if text_selection.start_position >= text_selection.end_position:
            return False

        return True

    async def handle_edge_case_large_selection(self, selection: str) -> str:
        """
        Handle edge case when text selection is too large
        """
        logger.warning(f"Large text selection detected: {len(selection)} characters")
        # Return a truncated version or handle as needed
        if len(selection) > settings.max_selection_length:
            return selection[:settings.max_selection_length]
        return selection

    async def handle_unrelated_query(self, query: str, selected_text: str) -> ChatResponse:
        """
        Handle case when query is unrelated to selected text
        """
        # In a real implementation, this would check semantic similarity
        # For now, we return a response indicating the issue
        response_text = f"Based on the selected text, I cannot provide a relevant answer to: '{query}'. The selected text does not contain information related to your query."

        sources = self._create_sources_from_selected_text(selected_text)

        return ChatResponse(
            id=UUID(int=0),
            query_id=UUID(int=0),
            response_text=response_text,
            sources=sources,
            confidence_score=0.1,  # Low confidence since answer is limited
            timestamp=datetime.now(),
            token_usage={
                "input_tokens": len(query.split()),
                "output_tokens": len(response_text.split()),
                "total_tokens": len(query.split()) + len(response_text.split())
            }
        )


# Create a singleton instance
rag_service = RAGService()