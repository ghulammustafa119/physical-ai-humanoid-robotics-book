from pydantic import BaseModel
from typing import Optional
from datetime import datetime
from uuid import UUID


class ChatSessionBase(BaseModel):
    user_id: Optional[UUID] = None
    active: bool = True
    selected_text_id: Optional[UUID] = None

    class Config:
        from_attributes = True


class ChatSessionCreate(ChatSessionBase):
    pass


class ChatSessionUpdate(BaseModel):
    user_id: Optional[UUID] = None
    active: Optional[bool] = None
    selected_text_id: Optional[UUID] = None


class ChatSession(ChatSessionBase):
    id: UUID
    created_at: datetime
    updated_at: datetime