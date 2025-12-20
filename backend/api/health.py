from fastapi import APIRouter
from typing import Dict

router = APIRouter()

@router.get("/health")
async def health_check():
    """
    Health check endpoint to verify API is running
    """
    return {
        "status": "healthy",
        "service": "Physical AI & Humanoid Robotics Book RAG API",
        "version": "1.0.0"
    }

@router.get("/ready")
async def readiness_check():
    """
    Readiness check to verify all dependencies are available
    """
    # In the future, this could check database connections, etc.
    return {
        "status": "ready",
        "dependencies": {
            "database": "not_connected",  # Will be updated when database is implemented
            "vector_db": "not_connected",  # Will be updated when Qdrant is implemented
            "external_apis": "not_connected"  # Will be updated when OpenAI is implemented
        }
    }