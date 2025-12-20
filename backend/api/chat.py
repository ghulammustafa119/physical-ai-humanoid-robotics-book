from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import logging

# Import the RAG service
from services.rag import rag_service

router = APIRouter()

# Request/Response models
class ChatRequest(BaseModel):
    query: str
    context: Optional[dict] = None

class Source(BaseModel):
    content: str
    source: str
    page: Optional[int] = None

class ChatResponse(BaseModel):
    response: str
    sources: List[Source]
    context: Optional[dict] = None

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint that processes user queries against book content
    """
    try:
        # Validate the query first
        validation_result = rag_service.validate_query(request.query)
        if not validation_result["valid"]:
            raise HTTPException(status_code=400, detail=validation_result["message"])

        # Process the query using RAG
        result = rag_service.process_query(request.query)

        return ChatResponse(
            response=result["response"],
            sources=result["sources"],
            context=result["context"]
        )
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logging.error(f"Error processing chat request: {e}")
        raise HTTPException(status_code=500, detail="Error processing chat request")

@router.post("/validate-query")
async def validate_query_endpoint(request: ChatRequest):
    """
    Validate that the query is appropriate for the RAG system
    """
    validation_result = rag_service.validate_query(request.query)

    if not validation_result["valid"]:
        raise HTTPException(status_code=400, detail=validation_result["message"])

    return {"valid": True, "message": validation_result["message"]}