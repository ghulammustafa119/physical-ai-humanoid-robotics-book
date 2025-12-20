from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Dict, Any, Optional
from services.integration import integration_service
import asyncio

router = APIRouter()

class SyncRequest(BaseModel):
    source_path: str = "physical-ai-book/docs"

class FeedbackRequest(BaseModel):
    query: str
    response: str
    feedback: str
    rating: Optional[int] = None

class IntegrationResponse(BaseModel):
    status: str
    message: str
    data: Optional[Dict[str, Any]] = None

@router.post("/sync-content", response_model=IntegrationResponse)
async def sync_content(request: SyncRequest):
    """
    Synchronize book content with the RAG system
    """
    try:
        result = await integration_service.sync_content(request.source_path)
        return IntegrationResponse(**result)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error during content synchronization: {str(e)}")

@router.get("/content-mapping", response_model=IntegrationResponse)
async def get_content_mapping():
    """
    Get mapping of book content to vector database entries
    """
    try:
        result = integration_service.get_content_mapping()
        return IntegrationResponse(**result)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting content mapping: {str(e)}")

@router.post("/feedback", response_model=IntegrationResponse)
async def add_feedback(request: FeedbackRequest):
    """
    Add user feedback for a query-response pair
    """
    try:
        result = await integration_service.add_feedback(
            request.query,
            request.response,
            request.feedback,
            request.rating
        )
        return IntegrationResponse(**result)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error recording feedback: {str(e)}")

@router.get("/fallback-content")
async def get_fallback_content(query: str = ""):
    """
    Get fallback content when RAG system cannot answer the query
    """
    try:
        content = integration_service.get_fallback_content(query)
        return {"content": content}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting fallback content: {str(e)}")