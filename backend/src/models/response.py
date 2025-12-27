from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime
from uuid import UUID


class SourceReference(BaseModel):
    title: str
    url: str
    page: int
    section: Optional[str] = None
    text_snippet: Optional[str] = None


class ChatResponseBase(BaseModel):
    query_id: UUID
    response_text: str
    sources: List[SourceReference]
    confidence_score: float = 0.0
    token_usage: Optional[Dict[str, int]] = None

    class Config:
        from_attributes = True


class ChatResponseCreate(ChatResponseBase):
    pass


class ChatResponseUpdate(BaseModel):
    response_text: Optional[str] = None
    sources: Optional[List[SourceReference]] = None
    confidence_score: Optional[float] = None
    token_usage: Optional[Dict[str, int]] = None


class ChatResponse(ChatResponseBase):
    id: UUID
    timestamp: datetime