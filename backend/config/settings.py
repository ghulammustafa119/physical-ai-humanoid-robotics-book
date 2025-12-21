import os
from typing import Optional
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    # API Settings
    app_name: str = "Physical AI & Humanoid Robotics Book - RAG API"
    app_version: str = "1.0.0"
    debug: bool = os.getenv("DEBUG", "False").lower() == "true"
    port: int = int(os.getenv("PORT", "8000"))

    # Database Settings
    database_url: str = os.getenv("DATABASE_URL", "postgresql://user:password@localhost/book_db")
    neon_database_url: str = os.getenv("NEON_DATABASE_URL", "")

    # Vector Database Settings (Qdrant)
    qdrant_url: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "book_content")

    # Google Gemini Settings
    google_api_key: str = os.getenv("GOOGLE_API_KEY", "")
    embedding_model: str = os.getenv("EMBEDDING_MODEL", "text-embedding-004")  # Using Google's embedding model
    chat_model: str = os.getenv("CHAT_MODEL", "gemini-pro")
    gemini_model: str = os.getenv("GEMINI_MODEL", "gemini-pro")

    # Application Settings
    max_query_length: int = int(os.getenv("MAX_QUERY_LENGTH", "1000"))
    max_response_length: int = int(os.getenv("MAX_RESPONSE_LENGTH", "2000"))
    max_context_length: int = int(os.getenv("MAX_CONTEXT_LENGTH", "3000"))

    # Rate Limiting
    requests_per_minute: int = int(os.getenv("REQUESTS_PER_MINUTE", "60"))

    # File Processing
    chunk_size: int = int(os.getenv("CHUNK_SIZE", "1000"))
    chunk_overlap: int = int(os.getenv("CHUNK_OVERLAP", "200"))

    class Config:
        env_file = ".env"

# Create settings instance
settings = Settings()