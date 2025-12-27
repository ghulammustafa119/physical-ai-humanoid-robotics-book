import openai
from typing import List, Optional, Dict, Any
from uuid import UUID
import logging
from ..config.settings import settings


logger = logging.getLogger(__name__)


class OpenAIService:
    """
    Service class for handling OpenAI API integration
    """

    def __init__(self):
        # Initialize OpenAI client
        openai.api_key = settings.openai_api_key

    async def generate_response(
        self,
        query: str,
        context: Optional[str] = None,
        selected_text: Optional[str] = None,
        max_tokens: Optional[int] = None
    ) -> Dict[str, Any]:
        """
        Generate a response using OpenAI's API
        """
        try:
            # Set up the messages for the chat completion
            messages = []

            # Add system message to set the context
            system_message = (
                "You are an AI assistant helping readers understand the Physical AI & Humanoid Robotics book. "
                "Answer questions based only on the provided context. "
                "If the context doesn't contain information to answer the question, say so clearly. "
                "Always provide source attribution when possible."
            )
            messages.append({"role": "system", "content": system_message})

            # Add context if provided
            if context:
                messages.append({
                    "role": "system",
                    "content": f"Context: {context}"
                })

            # Add selected text if provided (this restricts the answer to this text)
            if selected_text:
                messages.append({
                    "role": "system",
                    "content": f"Selected text (restrict your answer to this content): {selected_text}"
                })

            # Add the user query
            messages.append({"role": "user", "content": query})

            # Set default max_tokens if not provided
            if max_tokens is None:
                max_tokens = settings.rag_max_tokens

            # Call OpenAI API
            response = await openai.ChatCompletion.acreate(
                model=settings.openai_model,
                messages=messages,
                max_tokens=max_tokens,
                temperature=settings.rag_temperature,
                top_p=1,
                frequency_penalty=0,
                presence_penalty=0
            )

            # Extract the response
            content = response.choices[0].message.content
            usage = response.usage

            return {
                "response_text": content,
                "token_usage": {
                    "input_tokens": usage.prompt_tokens,
                    "output_tokens": usage.completion_tokens,
                    "total_tokens": usage.total_tokens
                },
                "confidence_score": 0.85  # This would be calculated based on various factors in a real implementation
            }

        except Exception as e:
            logger.error(f"Error calling OpenAI API: {str(e)}")
            raise

    async def validate_api_key(self) -> bool:
        """
        Validate that the OpenAI API key is working
        """
        try:
            # Make a simple test call to validate the API key
            response = await openai.ChatCompletion.acreate(
                model=settings.openai_model,
                messages=[{"role": "user", "content": "Test"}],
                max_tokens=5
            )
            return True
        except Exception as e:
            logger.error(f"OpenAI API key validation failed: {str(e)}")
            return False

    async def embed_text(self, text: str) -> List[float]:
        """
        Create embeddings for text using OpenAI's embedding API
        """
        try:
            response = await openai.Embedding.acreate(
                input=text,
                model="text-embedding-ada-002"  # Using a standard embedding model
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error creating embeddings: {str(e)}")
            raise


# Create a singleton instance
openai_service = OpenAIService()