from pydantic import BaseModel
from typing import Optional
from datetime import datetime
from uuid import UUID


class BookContentBase(BaseModel):
    title: str
    content: str
    book_id: str
    section_path: str
    page_number: int

    class Config:
        from_attributes = True


class BookContentCreate(BookContentBase):
    pass


class BookContentUpdate(BaseModel):
    title: Optional[str] = None
    content: Optional[str] = None
    book_id: Optional[str] = None
    section_path: Optional[str] = None
    page_number: Optional[int] = None


class BookContent(BookContentBase):
    id: UUID
    created_at: datetime
    updated_at: datetime
    # Note: vector_embedding is not included here as it's a binary field
    # that would be handled separately in the database layer