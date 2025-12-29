import os
import logging
from typing import Optional
import google.generativeai as genai
from ..config.settings import settings

logger = logging.getLogger(__name__)


class GeminiService:
    """Service for interacting with Google Gemini API"""

    def __init__(self):
        self.model = None
        self._initialize_client()

    def _initialize_client(self):
        """Initialize the Gemini client"""
        try:
            # Configure authentication
            if settings.google_application_credentials:
                credentials_path = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
                    settings.google_application_credentials
                )
                if os.path.exists(credentials_path):
                    os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = credentials_path
                    logger.info(f"Using Google Service Account credentials from {credentials_path}")

            # Configure Gemini
            genai.configure(api_key=settings.google_api_key if settings.google_api_key else None)

            # Initialize model
            self.model = genai.GenerativeModel(settings.gemini_model)
            logger.info(f"Gemini service initialized with model: {settings.gemini_model}")

        except Exception as e:
            logger.error(f"Failed to initialize Gemini service: {e}")
            raise

    async def generate_response(
        self,
        prompt: str,
        context: Optional[str] = None,
        max_tokens: Optional[int] = None,
        temperature: Optional[float] = None
    ) -> str:
        """
        Generate a response using Gemini

        Args:
            prompt: The user's query
            context: Optional context from retrieved documents
            max_tokens: Maximum tokens in response
            temperature: Temperature for generation

        Returns:
            Generated response text
        """
        try:
            # Build the full prompt
            full_prompt = prompt
            if context:
                full_prompt = f"""Context from the book:
{context}

User Question: {prompt}

Please answer the question based on the provided context. If the context doesn't contain relevant information, say so."""

            # Configure generation
            generation_config = {
                "max_output_tokens": max_tokens or settings.rag_max_tokens,
                "temperature": temperature or settings.rag_temperature,
            }

            # Generate response
            response = self.model.generate_content(
                full_prompt,
                generation_config=generation_config
            )

            if not response or not response.text:
                logger.warning("Empty response from Gemini")
                return "I apologize, but I couldn't generate a response. Please try again."

            return response.text

        except Exception as e:
            logger.error(f"Error generating response with Gemini: {e}")
            raise Exception(f"Failed to generate response: {str(e)}")


# Create a singleton instance
gemini_service = GeminiService()
