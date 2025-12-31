from datetime import datetime
from typing import List, Optional, Union
from pydantic import BaseModel
from sqlmodel import SQLModel, Field
from sqlalchemy import JSON


import uuid

class UserProfileBase(SQLModel):
    """Base model for user profile with common fields"""


class UserProfile(UserProfileBase, table=True):
    """SQLModel for user profile table in database"""
    id: Optional[str] = Field(
        default_factory=lambda: str(uuid.uuid4()),
        primary_key=True,
        index=True
    )
    user_id: str
    programming_level: Optional[str] = None
    python_level: Optional[str] = None
    ai_ml_level: Optional[str] = None
    robotics_level: Optional[str] = None
    system_type: Optional[str] = None
    gpu_availability: Optional[str] = None
    hardware_access: Optional[str] = None
    simulator_experience: Optional[List[str]] = Field(default_factory=list, sa_type=JSON)
    profile_completeness: float = 0.0
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)


class UserProfileCreate(UserProfileBase):
    """Model for creating a new user profile"""
    user_id: str
    programming_level: Optional[str] = None
    python_level: Optional[str] = None
    ai_ml_level: Optional[str] = None
    robotics_level: Optional[str] = None
    system_type: Optional[str] = None
    gpu_availability: Optional[str] = None
    hardware_access: Optional[str] = None
    simulator_experience: Optional[List[str]] = Field(default_factory=list, sa_type=JSON)

    class Config:
        schema_extra = {
            "example": {
                "programming_level": "intermediate",
                "python_level": "strong",
                "ai_ml_level": "basic",
                "robotics_level": "none",
                "system_type": "laptop",
                "gpu_availability": "integrated",
                "hardware_access": "simulators",
                "simulator_experience": ["gazebo"]
            }
        }


class UserProfileUpdate(SQLModel):
    """Model for updating user profile"""
    programming_level: Optional[str] = None
    python_level: Optional[str] = None
    ai_ml_level: Optional[str] = None
    robotics_level: Optional[str] = None
    system_type: Optional[str] = None
    gpu_availability: Optional[str] = None
    hardware_access: Optional[str] = None
    simulator_experience: Optional[List[str]] = None


class UserProfileResponse(BaseModel):
    """Response model for user profile"""
    id: Optional[str] = None
    user_id: Optional[str] = None
    programming_level: Optional[str] = None
    python_level: Optional[str] = None
    ai_ml_level: Optional[str] = None
    robotics_level: Optional[str] = None
    system_type: Optional[str] = None
    gpu_availability: Optional[str] = None
    hardware_access: Optional[str] = None
    simulator_experience: Optional[List[str]] = []
    profile_completeness: Optional[float] = 0.0
    created_at: Optional[datetime] = None
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True


class PersonalizationContext(BaseModel):
    """Context model for RAG personalization"""
    skill_level: str
    python_level: str
    ai_ml_level: str
    robotics_level: str
    system_type: str
    has_gpu: bool
    hardware_access: str
    simulators: List[str]
    is_complete: bool

    @classmethod
    def from_profile(cls, profile: UserProfileResponse) -> 'PersonalizationContext':
        """Create PersonalizationContext from UserProfileResponse"""
        if not profile:
            return cls(
                skill_level='beginner',
                python_level='none',
                ai_ml_level='none',
                robotics_level='none',
                system_type='laptop',
                has_gpu=False,
                hardware_access='none',
                simulators=[],
                is_complete=False
            )

        gpu_availability_map = {
            'nvidia_cuda': True,
            'integrated': True,
            'none': False
        }

        return cls(
            skill_level=profile.programming_level or 'beginner',
            python_level=profile.python_level or 'none',
            ai_ml_level=profile.ai_ml_level or 'none',
            robotics_level=profile.robotics_level or 'none',
            system_type=profile.system_type or 'laptop',
            has_gpu=gpu_availability_map.get(profile.gpu_availability, False),
            hardware_access=profile.hardware_access or 'none',
            simulators=profile.simulator_experience or [],
            is_complete=profile.profile_completeness >= 0.75
        )


class ProfileCompletenessRequest(BaseModel):
    """Request model for calculating profile completeness"""
    programming_level: Optional[str] = None
    python_level: Optional[str] = None
    ai_ml_level: Optional[str] = None
    robotics_level: Optional[str] = None
    system_type: Optional[str] = None
    gpu_availability: Optional[str] = None
    hardware_access: Optional[str] = None
    simulator_experience: Optional[List[str]] = None

    def calculate_completeness(self) -> float:
        """Calculate profile completeness score"""
        total_fields = 8
        filled_fields = 0

        if self.programming_level: filled_fields += 1
        if self.python_level: filled_fields += 1
        if self.ai_ml_level: filled_fields += 1
        if self.robotics_level: filled_fields += 1
        if self.system_type: filled_fields += 1
        if self.gpu_availability: filled_fields += 1
        if self.hardware_access: filled_fields += 1
        if self.simulator_experience and len(self.simulator_experience) > 0: filled_fields += 1

        return round(filled_fields / total_fields, 2)