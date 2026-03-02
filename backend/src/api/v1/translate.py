from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel
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


class TranslateRequest(BaseModel):
    chapter_content: str
    chapter_title: str


class TranslateResponse(BaseModel):
    translated_content: str
    source_language: str
    target_language: str


@router.post("/translate", response_model=TranslateResponse)
async def translate_chapter(request_body: TranslateRequest):
    """
    Translate chapter content to Urdu.
    No authentication required — any visitor can translate.
    """
    # Truncate chapter content if too long
    chapter_text = request_body.chapter_content
    if len(chapter_text) > 6000:
        chapter_text = chapter_text[:6000] + "\n\n[Content truncated for processing...]"

    prompt = f"""You are a professional technical translator. Translate the following technical textbook content
from English to Urdu. Follow these rules strictly:

1. Translate all explanatory text, descriptions, and instructions to Urdu
2. Keep ALL code blocks, code snippets, variable names, and command examples in English
3. Keep technical terms like "ROS 2", "NVIDIA Isaac", "Gazebo", "SLAM", "LiDAR", "GPU", "API", "SDK" in English
4. Keep file paths, URLs, and configuration values in English
5. Use Urdu script (not Roman Urdu)
6. Maintain the same Markdown formatting (headings, bold, lists, code blocks)
7. Do NOT add any preamble like "Here is the translation" — return only the translated markdown

Chapter Title: {request_body.chapter_title}

Content to translate:
{chapter_text}"""

    try:
        if AI_SERVICE == 'cohere':
            translated = await cohere_service.generate_response(
                prompt=prompt,
                context=None,
                max_tokens=4000,
            )
        elif AI_SERVICE == 'gemini':
            translated = await gemini_service.generate_response(
                prompt=prompt,
                context=None,
                max_tokens=4000,
            )
        else:
            result = await openai_service.generate_response(
                query=prompt,
                context=None,
            )
            translated = result["response_text"]

        return TranslateResponse(
            translated_content=translated,
            source_language="english",
            target_language="urdu",
        )

    except Exception as e:
        logger.error(f"Error translating chapter: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error translating content: {str(e)}",
        )
