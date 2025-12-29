import logging
from typing import Optional
import cohere
from ..config.settings import settings

logger = logging.getLogger(__name__)


class CohereService:
    """Service for interacting with Cohere API"""

    def __init__(self):
        self.client = None
        self._initialize_client()

    def _initialize_client(self):
        """Initialize the Cohere client"""
        try:
            # Check if Cohere API key is available
            cohere_key = getattr(settings, 'cohere_api_key', None)
            if not cohere_key:
                logger.warning("No Cohere API key found in settings")
                return

            # Initialize Cohere client
            self.client = cohere.Client(cohere_key)
            logger.info("Cohere service initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize Cohere service: {e}")
            raise

    async def generate_response(
        self,
        prompt: str,
        context: Optional[str] = None,
        max_tokens: Optional[int] = None,
        temperature: Optional[float] = None
    ) -> str:
        """
        Generate a response using Cohere

        Args:
            prompt: The user's query
            context: Optional context from retrieved documents
            max_tokens: Maximum tokens in response
            temperature: Temperature for generation

        Returns:
            Generated response text
        """
        try:
            if not self.client:
                raise Exception("Cohere client not initialized. Please check API key.")

            # Build the full prompt
            full_prompt = prompt
            if context:
                full_prompt = f"""Context from the book:
{context}

User Question: {prompt}

Please answer the question based on the provided context. If the context doesn't contain relevant information, say so."""

            # Generate response using Cohere
            model_to_use = settings.cohere_model
            logger.info(f"Using Cohere model: {model_to_use}")
            response = self.client.chat(
                message=full_prompt,
                model=model_to_use,
                max_tokens=max_tokens or settings.rag_max_tokens,
                temperature=temperature or settings.rag_temperature,
            )

            if not response or not response.text:
                logger.warning("Empty response from Cohere")
                return "I apologize, but I couldn't generate a response. Please try again."

            return response.text

        except Exception as e:
            logger.error(f"Error generating response with Cohere: {e}")
            raise Exception(f"Failed to generate response: {str(e)}")


# Create a singleton instance
cohere_service = CohereService()
