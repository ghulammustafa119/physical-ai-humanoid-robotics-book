from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # App settings
    app_name: str = "Advanced RAG Chatbot API"
    app_version: str = "1.0.0"
    debug: bool = False
    environment: str = "development"

    # API settings
    api_v1_prefix: str = "/api/v1"
    allowed_origins: str = "*"  # In production, specify actual origins

    # Database settings
    database_url: str

    # Qdrant settings
    qdrant_url: str
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "book_content"

    # OpenAI settings
    openai_api_key: str
    openai_model: str = "gpt-3.5-turbo"

    # RAG settings
    rag_max_tokens: int = 2000
    rag_temperature: float = 0.7
    rag_top_k: int = 5
    rag_similarity_threshold: float = 0.7

    # Performance settings
    response_timeout: int = 30  # seconds
    max_query_length: int = 1000
    max_selection_length: int = 5000

    class Config:
        env_file = ".env"


settings = Settings()