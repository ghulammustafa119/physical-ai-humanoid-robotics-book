from typing import List, Optional
from uuid import UUID
import uuid
from datetime import datetime
import logging
from ..models.chat_session import ChatSession
from ..models.user_query import UserQuery, UserQueryCreate
from ..models.response import ChatResponse
from ..models.text_selection import TextSelection
from ..services.rag_service import rag_service


logger = logging.getLogger(__name__)


class ChatService:
    """
    Service class for handling chat session management and conversation flow
    """

    def __init__(self):
        pass

    async def create_chat_session(
        self,
        user_id: Optional[UUID] = None,
        selected_text_id: Optional[UUID] = None
    ) -> ChatSession:
        """
        Create a new chat session
        """
        try:
            session_id = uuid.uuid4()

            # In a real implementation, this would create a session in the database
            session = ChatSession(
                id=session_id,
                user_id=user_id,
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow(),
                active=True,
                selected_text_id=selected_text_id
            )

            return session

        except Exception as e:
            logger.error(f"Error creating chat session: {str(e)}")
            raise

    async def process_query_in_session(
        self,
        session_id: UUID,
        query_text: str,
        selected_text: Optional[str] = None,
        query_type: str = "initial"
    ) -> ChatResponse:
        """
        Process a user query within a chat session context
        """
        try:
            # In a real implementation, you would:
            # 1. Validate the session exists and is active
            # 2. Create a UserQuery record
            # 3. Process the query with the RAG service
            # 4. Store the response
            # 5. Update the session timestamp

            # For now, we'll use the RAG service directly
            response = await rag_service.query_with_selected_text(
                query=query_text,
                selected_text=selected_text
            )

            return response

        except Exception as e:
            logger.error(f"Error processing query in session: {str(e)}")
            raise

    async def update_session_context(
        self,
        session_id: UUID,
        selected_text_id: Optional[UUID] = None
    ) -> ChatSession:
        """
        Update the context of an existing session (e.g., when user selects new text)
        """
        try:
            # In a real implementation, this would update the session in the database
            # For now, return a mock updated session
            session = ChatSession(
                id=session_id,
                user_id=None,  # Would come from auth in real implementation
                created_at=datetime.utcnow(),  # Would be from DB in real implementation
                updated_at=datetime.utcnow(),
                active=True,
                selected_text_id=selected_text_id
            )

            return session

        except Exception as e:
            logger.error(f"Error updating session context: {str(e)}")
            raise

    async def get_session_history(
        self,
        session_id: UUID,
        limit: int = 10
    ) -> List[dict]:
        """
        Retrieve the conversation history for a session
        """
        try:
            # In a real implementation, this would fetch from the database
            # For now, return empty history
            return []

        except Exception as e:
            logger.error(f"Error getting session history: {str(e)}")
            raise

    async def validate_session(
        self,
        session_id: UUID
    ) -> bool:
        """
        Validate that a session exists and is active
        """
        try:
            # In a real implementation, this would check the database
            # For now, assume all sessions are valid
            return True

        except Exception as e:
            logger.error(f"Error validating session: {str(e)}")
            return False


# Create a singleton instance
chat_service = ChatService()