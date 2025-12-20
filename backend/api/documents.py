from fastapi import APIRouter, HTTPException, BackgroundTasks
from pydantic import BaseModel
from typing import Dict, Any, Optional
from services.document_ingestion import document_ingestion_service
import logging

router = APIRouter()

class IngestionRequest(BaseModel):
    source_path: str = "physical-ai-book/docs"
    force_reindex: bool = False

class IngestionResponse(BaseModel):
    status: str
    processed_documents: int
    stored_chunks: int
    message: str

class StatusResponse(BaseModel):
    status: str
    collection_name: str
    vectors_count: int
    message: str

@router.post("/ingest", response_model=IngestionResponse)
async def ingest_documents(request: IngestionRequest):
    """
    Ingest book documents into the vector database
    """
    try:
        result = await document_ingestion_service.ingest_documents(request.source_path)
        return IngestionResponse(**result)
    except Exception as e:
        logging.error(f"Error during document ingestion: {e}")
        raise HTTPException(status_code=500, detail=f"Error during document ingestion: {str(e)}")

@router.get("/status", response_model=StatusResponse)
async def get_ingestion_status():
    """
    Get the current status of document ingestion
    """
    try:
        status = document_ingestion_service.get_ingestion_status()
        return StatusResponse(**status)
    except Exception as e:
        logging.error(f"Error getting ingestion status: {e}")
        raise HTTPException(status_code=500, detail=f"Error getting ingestion status: {str(e)}")

@router.post("/reindex")
async def reindex_documents(request: IngestionRequest):
    """
    Clear existing index and reindex all documents
    """
    try:
        # In a real implementation, we would clear the existing collection
        # and then re-ingest all documents
        # For now, we'll just call the regular ingestion
        result = await document_ingestion_service.ingest_documents(request.source_path)
        return IngestionResponse(**result)
    except Exception as e:
        logging.error(f"Error during reindexing: {e}")
        raise HTTPException(status_code=500, detail=f"Error during reindexing: {str(e)}")