from typing import Optional, Dict, Any
from sqlmodel import Session, select
from ..models.profile import UserProfile, PersonalizationContext
from ..models.user import User


class PersonalizationService:
    """
    Service class for handling personalization context for RAG chatbot
    """

    def __init__(self, db_session: Session):
        self.db = db_session

    def get_personalization_context(self, user_id: str) -> PersonalizationContext:
        """
        Get personalization context for a user
        """
        # Fetch user profile from database
        profile = self.db.exec(
            select(UserProfile).where(UserProfile.user_id == user_id)
        ).first()

        # Create personalization context from profile or return default
        if profile:
            return PersonalizationContext.from_profile(
                profile
            )
        else:
            # Return default context for unauthenticated users
            return PersonalizationContext(
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

    def get_context_for_rag(self, user_id: Optional[str]) -> Dict[str, Any]:
        """
        Get context specifically formatted for RAG system
        """
        if not user_id:
            # Return generic context for unauthenticated users
            return {
                'user_context': None,
                'personalization_enabled': False,
                'system_prompt_extension': ''
            }

        context = self.get_personalization_context(user_id)

        # Create system prompt extension based on user profile
        prompt_extension = self._create_system_prompt_extension(context)

        return {
            'user_context': context.dict(),
            'personalization_enabled': True,
            'system_prompt_extension': prompt_extension
        }

    def _create_system_prompt_extension(self, context: PersonalizationContext) -> str:
        """
        Create a system prompt extension based on personalization context
        """
        if not context.is_complete:
            # If profile is incomplete, provide minimal personalization
            return f"""
            When responding to this user's questions:
            - Adapt explanation depth based on their skill level: {context.skill_level}
            - Consider their hardware limitations (GPU: {'available' if context.has_gpu else 'not available'})
            """

        # For complete profiles, provide detailed personalization
        return f"""
        User Profile:
        - Skill Level: {context.skill_level}
        - Python Experience: {context.python_level}
        - AI/ML Experience: {context.ai_ml_level}
        - Robotics Experience: {context.robotics_level}
        - System: {context.system_type}
        - GPU Available: {'Yes' if context.has_gpu else 'No'}
        - Hardware Access: {context.hardware_access}
        - Simulators Used: {', '.join(context.simulators) if context.simulators else 'None'}

        When responding to this user's questions:
        - Adapt explanation depth based on their skill level ({context.skill_level})
        - Provide code examples optimized for their hardware (GPU: {'available' if context.has_gpu else 'not available'})
        - Adjust complexity based on their experience levels (Python: {context.python_level}, AI/ML: {context.ai_ml_level}, Robotics: {context.robotics_level})
        - Consider their hardware access ({context.hardware_access}) when suggesting practical exercises
        - Reference simulators they're familiar with ({', '.join(context.simulators) if context.simulators else 'none specified'})
        """

    def get_user_advice(self, user_id: str, topic: str) -> str:
        """
        Generate personalized advice for a specific topic based on user profile
        """
        context = self.get_personalization_context(user_id)

        advice_parts = []

        # Skill level advice
        if context.skill_level == 'beginner':
            advice_parts.append("Start with fundamental concepts and simple examples.")
        elif context.skill_level == 'intermediate':
            advice_parts.append("Build on existing knowledge with more complex examples.")
        else:  # advanced
            advice_parts.append("Focus on advanced techniques and optimization.")

        # Hardware considerations
        if not context.has_gpu and 'deep learning' in topic.lower():
            advice_parts.append("Since you don't have a GPU, consider using CPU-optimized implementations or cloud-based solutions.")

        if context.hardware_access == 'simulators' and topic in ['robotics', 'control']:
            advice_parts.append("Focus on simulation-based learning since you have simulator access.")

        return " ".join(advice_parts)

    def get_content_adaptation(self, user_id: Optional[str], content_level: str = 'intermediate') -> Dict[str, Any]:
        """
        Get content adaptation settings based on user profile
        """
        if not user_id:
            return {
                'difficulty': 'intermediate',
                'examples_complexity': 'moderate',
                'theoretical_depth': 'balanced',
                'practical_focus': 'general'
            }

        context = self.get_personalization_context(user_id)

        # Determine content adaptation based on user profile
        difficulty = content_level
        if context.skill_level == 'beginner':
            difficulty = 'beginner'
        elif context.skill_level == 'advanced' and content_level == 'intermediate':
            difficulty = 'advanced'

        return {
            'difficulty': difficulty,
            'examples_complexity': 'simple' if context.skill_level == 'beginner' else 'moderate',
            'theoretical_depth': 'basic' if context.skill_level == 'beginner' else 'detailed',
            'practical_focus': context.hardware_access
        }

    def is_user_ready_for_topic(self, user_id: str, topic: str) -> Dict[str, Any]:
        """
        Check if user is ready for a specific topic based on their profile
        """
        context = self.get_personalization_context(user_id)

        readiness = {
            'ready': True,
            'prerequisites': [],
            'recommendations': []
        }

        # Example logic for different topics
        if topic.lower() in ['deep learning', 'neural networks'] and context.ai_ml_level == 'none':
            readiness['ready'] = False
            readiness['prerequisites'] = ['Basic AI/ML concepts', 'Python programming']
            readiness['recommendations'] = ['Start with introductory AI/ML tutorials']

        if topic.lower() in ['gpu programming', 'cuda'] and not context.has_gpu:
            readiness['ready'] = True but 'with limitations'
            readiness['recommendations'] = ['Consider cloud GPU services for hands-on practice']

        return readiness

    def get_learning_path_suggestion(self, user_id: str) -> Dict[str, Any]:
        """
        Suggest a learning path based on user profile
        """
        context = self.get_personalization_context(user_id)

        path_suggestions = []

        if context.skill_level == 'beginner':
            path_suggestions.extend([
                "Start with Python fundamentals",
                "Learn basic programming concepts",
                "Explore introductory AI/ML resources"
            ])
        elif context.skill_level == 'intermediate':
            path_suggestions.extend([
                "Deepen your understanding of core concepts",
                "Work on more complex projects",
                "Explore specialized topics based on interests"
            ])
        else:  # advanced
            path_suggestions.extend([
                "Focus on cutting-edge research",
                "Contribute to open-source projects",
                "Mentor others in the field"
            ])

        # Hardware-specific suggestions
        if context.hardware_access == 'simulators':
            path_suggestions.append("Emphasize simulation-based learning and practice")

        if context.has_gpu:
            path_suggestions.append("Leverage GPU computing for intensive tasks")

        return {
            'suggested_path': path_suggestions,
            'estimated_time': f"{len(path_suggestions)} months for comprehensive coverage",
            'next_steps': path_suggestions[:2] if path_suggestions else []
        }


# Example usage:
# from backend.src.db.connection import get_session
#
# with next(get_session()) as session:
#     personalization_service = PersonalizationService(session)
#
#     # Get context for RAG
#     context = personalization_service.get_context_for_rag('user_id_here')
#     print(f"Personalization enabled: {context['personalization_enabled']}")
#
#     # Get learning path
#     path = personalization_service.get_learning_path_suggestion('user_id_here')
#     print(f"Suggested path: {path['suggested_path']}")