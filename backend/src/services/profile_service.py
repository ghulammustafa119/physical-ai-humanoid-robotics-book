from typing import Optional, List, Dict, Any
from sqlmodel import Session, select
from datetime import datetime
from ..models.profile import UserProfile, UserProfileCreate, UserProfileUpdate, ProfileCompletenessRequest
from ..models.user import User


class ProfileService:
    """
    Service class for managing user profiles
    """

    def __init__(self, db_session: Session):
        self.db = db_session

    def create_profile(self, user_id: str, profile_data: dict) -> UserProfile:
        """
        Create a new user profile
        """
        # Calculate profile completeness
        completeness_request = ProfileCompletenessRequest(**profile_data)
        completeness = completeness_request.calculate_completeness()

        profile = UserProfile(
            user_id=user_id,
            programming_level=profile_data.get('programming_level'),
            python_level=profile_data.get('python_level'),
            ai_ml_level=profile_data.get('ai_ml_level'),
            robotics_level=profile_data.get('robotics_level'),
            system_type=profile_data.get('system_type'),
            gpu_availability=profile_data.get('gpu_availability'),
            hardware_access=profile_data.get('hardware_access'),
            simulator_experience=profile_data.get('simulator_experience', []),
            profile_completeness=completeness
        )

        self.db.add(profile)
        self.db.commit()
        self.db.refresh(profile)

        return profile

    def get_profile(self, user_id: str) -> Optional[Dict[str, Any]]:
        """
        Get user profile by user_id
        """
        profile = self.db.exec(
            select(UserProfile).where(UserProfile.user_id == user_id)
        ).first()

        if not profile:
            return None

        return {
            'id': profile.id,
            'user_id': profile.user_id,
            'programming_level': profile.programming_level,
            'python_level': profile.python_level,
            'ai_ml_level': profile.ai_ml_level,
            'robotics_level': profile.robotics_level,
            'system_type': profile.system_type,
            'gpu_availability': profile.gpu_availability,
            'hardware_access': profile.hardware_access,
            'simulator_experience': profile.simulator_experience,
            'profile_completeness': profile.profile_completeness,
            'created_at': profile.created_at,
            'updated_at': profile.updated_at
        }

    def update_profile(self, user_id: str, updates: dict) -> Optional[UserProfile]:
        """
        Update user profile fields
        """
        profile = self.db.exec(
            select(UserProfile).where(UserProfile.user_id == user_id)
        ).first()

        if not profile:
            return None

        # Update only the fields that are provided in updates
        for field, value in updates.items():
            if hasattr(profile, field) and value is not None:
                setattr(profile, field, value)

        # Recalculate profile completeness
        completeness_request = ProfileCompletenessRequest(
            programming_level=profile.programming_level,
            python_level=profile.python_level,
            ai_ml_level=profile.ai_ml_level,
            robotics_level=profile.robotics_level,
            system_type=profile.system_type,
            gpu_availability=profile.gpu_availability,
            hardware_access=profile.hardware_access,
            simulator_experience=profile.simulator_experience
        )
        profile.profile_completeness = completeness_request.calculate_completeness()

        self.db.add(profile)
        self.db.commit()
        self.db.refresh(profile)

        return profile

    def calculate_completeness(self, profile: UserProfile) -> float:
        """
        Calculate profile completeness score
        """
        completeness_request = ProfileCompletenessRequest(
            programming_level=profile.programming_level,
            python_level=profile.python_level,
            ai_ml_level=profile.ai_ml_level,
            robotics_level=profile.robotics_level,
            system_type=profile.system_type,
            gpu_availability=profile.gpu_availability,
            hardware_access=profile.hardware_access,
            simulator_experience=profile.simulator_experience
        )
        return completeness_request.calculate_completeness()

    def handle_partial_profile(self, user_id: str, partial_data: dict) -> UserProfile:
        """
        Handle creation or update of a profile with partial data
        """
        existing_profile = self.db.exec(
            select(UserProfile).where(UserProfile.user_id == user_id)
        ).first()

        if existing_profile:
            # Update existing profile with new data
            for field, value in partial_data.items():
                if hasattr(existing_profile, field) and value is not None:
                    setattr(existing_profile, field, value)

            # Recalculate completeness
            completeness_request = ProfileCompletenessRequest(
                programming_level=existing_profile.programming_level,
                python_level=existing_profile.python_level,
                ai_ml_level=existing_profile.ai_ml_level,
                robotics_level=existing_profile.robotics_level,
                system_type=existing_profile.system_type,
                gpu_availability=existing_profile.gpu_availability,
                hardware_access=existing_profile.hardware_access,
                simulator_experience=existing_profile.simulator_experience
            )
            existing_profile.profile_completeness = completeness_request.calculate_completeness()

            self.db.add(existing_profile)
            self.db.commit()
            self.db.refresh(existing_profile)

            return existing_profile
        else:
            # Create new profile with partial data
            return self.create_profile(user_id, partial_data)

    def get_user_with_profile(self, user_id: str) -> Optional[Dict[str, Any]]:
        """
        Get user information along with their profile
        """
        user = self.db.exec(
            select(User).where(User.id == user_id)
        ).first()

        if not user:
            return None

        profile_data = self.get_profile(user_id)

        return {
            'user': {
                'id': user.id,
                'email': user.email,
                'created_at': user.created_at,
                'updated_at': user.updated_at
            },
            'profile': profile_data
        }

    def get_profile_summary(self, user_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a summary of the user's profile for quick access
        """
        profile = self.db.exec(
            select(UserProfile).where(UserProfile.user_id == user_id)
        ).first()

        if not profile:
            return None

        return {
            'user_id': profile.user_id,
            'profile_completeness': profile.profile_completeness,
            'programming_level': profile.programming_level,
            'python_level': profile.python_level,
            'ai_ml_level': profile.ai_ml_level,
            'robotics_level': profile.robotics_level,
            'system_type': profile.system_type,
            'has_gpu': profile.gpu_availability in ['integrated', 'nvidia_cuda'],
            'simulator_experience': profile.simulator_experience
        }

    def update_profile_completeness(self, user_id: str) -> float:
        """
        Recalculate and update the profile completeness for a user
        """
        profile = self.db.exec(
            select(UserProfile).where(UserProfile.user_id == user_id)
        ).first()

        if not profile:
            return 0.0

        completeness = self.calculate_completeness(profile)
        profile.profile_completeness = completeness

        self.db.add(profile)
        self.db.commit()

        return completeness


# Example usage:
# from backend.src.db.connection import get_session
#
# with next(get_session()) as session:
#     profile_service = ProfileService(session)
#
#     # Create profile
#     profile_data = {
#         'programming_level': 'intermediate',
#         'python_level': 'strong',
#         'ai_ml_level': 'basic',
#         'robotics_level': 'none',
#         'system_type': 'laptop',
#         'gpu_availability': 'integrated',
#         'hardware_access': 'simulators',
#         'simulator_experience': ['gazebo']
#     }
#
#     profile = profile_service.create_profile('user_id_here', profile_data)
#     print(f"Profile created with completeness: {profile.profile_completeness}")