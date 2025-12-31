from datetime import datetime
from typing import Optional
from pydantic import BaseModel, EmailStr, validator
from sqlmodel import SQLModel, Field


class UserBase(SQLModel):
    """Base model for user with common fields"""
    email: EmailStr
    password_hash: str


class User(UserBase, table=True):
    """SQLModel for user table in database"""
    id: Optional[str] = Field(default=None, primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)


class UserCreate(UserBase):
    """Model for creating a new user"""
    email: EmailStr
    password: str  # Raw password before hashing

    @validator('password')
    def validate_password(cls, v):
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters long')
        if not any(c.isupper() for c in v):
            raise ValueError('Password must contain at least one uppercase letter')
        if not any(c.islower() for c in v):
            raise ValueError('Password must contain at least one lowercase letter')
        if not any(c.isdigit() for c in v):
            raise ValueError('Password must contain at least one digit')
        return v


class UserUpdate(SQLModel):
    """Model for updating user information"""
    email: Optional[EmailStr] = None


class UserResponse(BaseModel):
    """Response model for user (excludes sensitive data)"""
    id: str
    email: EmailStr
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class UserLogin(SQLModel):
    """Model for user login"""
    email: EmailStr
    password: str