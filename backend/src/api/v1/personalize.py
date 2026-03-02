from fastapi import APIRouter, HTTPException, status, Request, Depends
from pydantic import BaseModel
from typing import Optional
from sqlmodel import Session
from ...db.dependency import get_db_session
from ...services.auth_service import AuthService
from ...services.personalization_service import PersonalizationService
import logging

logger = logging.getLogger(__name__)

# Import available AI service (same fallback pattern as rag_service)
AI_SERVICE = None
try:
    from ...services.cohere_service import cohere_service
    AI_SERVICE = 'cohere'
except ImportError:
    try:
        from ...services.gemini_service import gemini_service
        AI_SERVICE = 'gemini'
    except ImportError:
        from ...services.openai_service import openai_service
        AI_SERVICE = 'openai'

router = APIRouter()


class PersonalizeRequest(BaseModel):
    chapter_content: str
    chapter_title: str


class PersonalizeResponse(BaseModel):
    personalized_content: str
    user_level: str


@router.post("/personalize", response_model=PersonalizeResponse)
async def personalize_chapter(
    request_body: PersonalizeRequest,
    request: Request,
    db: Session = Depends(get_db_session),
):
    """
    Personalize chapter content based on user profile.
    Requires authentication via Bearer token.
    """
    # Extract and validate auth token
    authorization = request.headers.get("Authorization")
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required. Please sign in.",
        )

    token = authorization.split(" ")[1]
    auth_service = AuthService(db)
    session_info = auth_service.validate_session(token)

    if not session_info:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired session",
        )

    # Get personalization context
    personalization_service = PersonalizationService(db)
    context = personalization_service.get_personalization_context(session_info.user_id)

    # Build the personalization prompt
    system_prompt = personalization_service._create_system_prompt_extension(context)

    # Truncate chapter content if too long (keep first ~4000 chars for LLM context window)
    chapter_text = request_body.chapter_content
    if len(chapter_text) > 6000:
        chapter_text = chapter_text[:6000] + "\n\n[Content truncated for processing...]"

    prompt = f"""You are a textbook content personalizer. Your job is to adapt the following chapter content
to match the reader's background and skill level. Keep the same topics and structure but adjust:
- Explanation depth and complexity based on their level
- Code examples complexity
- Analogies and references appropriate for their background
- Hardware-specific guidance based on their setup

{system_prompt}

Chapter Title: {request_body.chapter_title}

Original Chapter Content (in Markdown):
{chapter_text}

Instructions:
- Return the personalized content in Markdown format
- Keep all headings and structure intact
- Adapt explanations, examples, and depth to the user's level
- If the user is a beginner, add more context and simpler explanations
- If the user is advanced, add more technical depth and optimization tips
- Keep the content roughly the same length
- Do NOT add any preamble like "Here is the personalized content" - just return the markdown directly"""

    try:
        if AI_SERVICE == 'cohere':
            personalized = await cohere_service.generate_response(
                prompt=prompt,
                context=None,
                max_tokens=4000,
            )
        elif AI_SERVICE == 'gemini':
            personalized = await gemini_service.generate_response(
                prompt=prompt,
                context=None,
                max_tokens=4000,
            )
        else:
            result = await openai_service.generate_response(
                query=prompt,
                context=None,
            )
            personalized = result["response_text"]

        return PersonalizeResponse(
            personalized_content=personalized,
            user_level=context.skill_level,
        )

    except Exception as e:
        logger.error(f"Error personalizing chapter: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error personalizing content: {str(e)}",
        )
