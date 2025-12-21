from pydantic import BaseModel
from typing import List, Optional


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