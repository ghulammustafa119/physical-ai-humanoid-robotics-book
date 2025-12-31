from datetime import datetime
from typing import Optional
from pydantic import BaseModel
from sqlmodel import SQLModel, Field


import uuid

class SessionBase(SQLModel):
    """Base model for session with common fields"""


class Session(SessionBase, table=True):
    """SQLModel for session table in database"""
    id: Optional[str] = Field(
        default_factory=lambda: str(uuid.uuid4()),
        primary_key=True,
        index=True
    )
    user_id: str
    token: str
    expires_at: datetime
    created_at: datetime = Field(default_factory=datetime.utcnow)
    ip_address: Optional[str] = None
    user_agent: Optional[str] = None


class SessionCreate(SessionBase):
    """Model for creating a new session"""
    user_id: str
    token: str
    expires_at: datetime
    ip_address: Optional[str] = None
    user_agent: Optional[str] = None


class SessionUpdate(SQLModel):
    """Model for updating session information"""
    expires_at: Optional[datetime] = None
    ip_address: Optional[str] = None
    user_agent: Optional[str] = None


class SessionResponse(BaseModel):
    """Response model for session"""
    id: str
    user_id: str
    token: str
    expires_at: datetime
    created_at: datetime

    class Config:
        from_attributes = True


class SessionValidate(BaseModel):
    """Model for validating session"""
    token: str