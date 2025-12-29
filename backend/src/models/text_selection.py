from pydantic import BaseModel
from typing import Optional
from datetime import datetime
from uuid import UUID


class TextSelectionBase(BaseModel):
    content: str
    start_position: int
    end_position: int
    book_section: str
    user_id: Optional[UUID] = None

    class Config:
        from_attributes = True


class TextSelectionCreate(TextSelectionBase):
    pass


class TextSelectionUpdate(BaseModel):
    content: Optional[str] = None
    start_position: Optional[int] = None
    end_position: Optional[int] = None
    book_section: Optional[str] = None


class TextSelection(TextSelectionBase):
    id: UUID
    created_at: datetime