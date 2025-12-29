from fastapi import APIRouter, HTTPException, Depends
from typing import Optional
from uuid import UUID
import uuid
from pydantic import BaseModel
from ...models.response import ChatResponse
from ...models.text_selection import TextSelection
from ...services.rag_service import rag_service
from ...config.database import get_db_session
from sqlalchemy.orm import Session


router = APIRouter()


class ChatRequest(BaseModel):
    query: str
    selectedText: Optional[str] = None
    selected_text: Optional[str] = None
    sessionId: Optional[str] = None
    session_id: Optional[str] = None
    bookSection: Optional[str] = None
    book_section: Optional[str] = None


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest
):
    """
    Main chat endpoint that processes user queries with optional text selection restriction
    """
    try:
        # Extract parameters (support both camelCase and snake_case)
        query = request.query
        selected_text = request.selected_text or request.selectedText
        book_section = request.book_section or request.bookSection

        # Validate input
        if not query or len(query.strip()) == 0:
            raise HTTPException(
                status_code=400,
                detail="Query cannot be empty"
            )

        # Process the query using the RAG service
        response = await rag_service.query_with_selected_text(
            query=query,
            selected_text=selected_text,
            book_section=book_section
        )

        # In a real implementation, you would:
        # 1. Create a UserQuery record in the database
        # 2. Store the ChatResponse in the database
        # 3. Link them with the session_id if provided
        # 4. Return the response

        return response

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error processing chat query: {str(e)}"
        )


@router.get("/sessions/{session_id}")
async def get_session(
    session_id: str,
    db: Session = Depends(get_db_session)
):
    """
    Get details of a specific chat session
    """
    try:
        # In a real implementation, you would fetch the session from the database
        # For now, return a mock response
        return {
            "session_id": session_id,
            "active": True,
            "created_at": "2023-10-20T10:30:00Z",
            "updated_at": "2023-10-20T10:30:00Z"
        }
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error retrieving session: {str(e)}"
        )


@router.get("/sessions")
async def list_sessions(
    db: Session = Depends(get_db_session)
):
    """
    List all chat sessions for the authenticated user
    """
    try:
        # In a real implementation, you would fetch sessions from the database
        # For now, return a mock response
        return {
            "sessions": []
        }
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error listing sessions: {str(e)}"
        )