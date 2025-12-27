from pydantic import BaseModel
from typing import Optional
from datetime import datetime
from uuid import UUID


class UserQueryBase(BaseModel):
    session_id: UUID
    query_text: str
    selected_text_id: Optional[UUID] = None
    query_type: str = "initial"  # initial, followup, context_change

    class Config:
        from_attributes = True


class UserQueryCreate(UserQueryBase):
    pass


class UserQueryUpdate(BaseModel):
    query_text: Optional[str] = None
    selected_text_id: Optional[UUID] = None
    query_type: Optional[str] = None


class UserQuery(UserQueryBase):
    id: UUID
    timestamp: datetime