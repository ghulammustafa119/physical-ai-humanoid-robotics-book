from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlmodel import Session, create_engine as create_sql_engine
from typing import Generator
import os
from ..config.settings import settings


# Create the database engine
# Using SQLModel engine for compatibility with SQLModel tables
engine = create_sql_engine(
    settings.database_url,
    echo=settings.debug,  # Log SQL queries in debug mode
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300,    # Recycle connections every 5 minutes
    pool_size=10,        # Number of connection objects to maintain
    max_overflow=20,     # Additional connections beyond pool_size
    connect_args={
        "connect_timeout": 10,  # Timeout for establishing connection
    }
)


def get_session() -> Generator[Session, None, None]:
    """
    Dependency function to get database session
    Used with FastAPI dependency injection
    """
    with Session(engine) as session:
        yield session


# Alternative function for direct session access (if not using FastAPI DI)
def create_session() -> Session:
    """
    Create a new database session
    Use this when not using FastAPI dependency injection
    """
    return Session(engine)


# Test connection function
def test_connection():
    """
    Test the database connection
    """
    try:
        with Session(engine) as session:
            # Try a simple query to test connection
            result = session.exec("SELECT 1").first()
            if result:
                print("Database connection successful!")
                return True
    except Exception as e:
        print(f"Database connection failed: {e}")
        return False


# Initialize tables (run this once at application startup)
def init_db():
    """
    Initialize the database tables
    This should be called once at application startup
    """
    from sqlmodel import SQLModel
    from ..models.user import User
    from ..models.session import Session as SessionModel
    from ..models.profile import UserProfile

    # Create all tables
    SQLModel.metadata.create_all(engine)
    print("Database tables created successfully!")


if __name__ == "__main__":
    # Test the connection when running this file directly
    test_connection()
    init_db()