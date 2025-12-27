from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from typing import Dict, Any
import uvicorn
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import API routers
from api.chat import router as chat_router
from api.health import router as health_router
from api.documents import router as documents_router
from api.integration import router as integration_router

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Book - RAG API",
    description="API for RAG chatbot that answers questions based on book content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers
app.include_router(chat_router, prefix="/api/v1", tags=["chat"])
app.include_router(health_router, prefix="/api/v1", tags=["health"])
app.include_router(documents_router, prefix="/api/v1", tags=["documents"])
app.include_router(integration_router, prefix="/api/v1", tags=["integration"])

@app.get("/")
async def root():
    return {"message": "Physical AI & Humanoid Robotics Book RAG API"}

if __name__ == "__main__":
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)