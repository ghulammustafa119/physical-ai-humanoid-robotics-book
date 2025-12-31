from typing import Generator
from sqlmodel import Session
from .connection import engine


def get_db_session() -> Generator[Session, None, None]:
    """
    FastAPI dependency to provide database session

    Usage in FastAPI endpoints:
    ```
    from fastapi import Depends
    from backend.src.db.dependency import get_db_session

    @app.get("/users")
    def get_users(db: Session = Depends(get_db_session)):
        # Use db session here
        pass
    ```
    """
    with Session(engine) as session:
        yield session


# Alternative dependency with error handling
def get_db_session_with_error_handling() -> Generator[Session, None, None]:
    """
    FastAPI dependency to provide database session with error handling
    """
    try:
        with Session(engine) as session:
            yield session
    except Exception as e:
        # Log the error if you have a logging system
        # logger.error(f"Database session error: {e}")
        raise e


# For testing purposes - a session that can be reused
def get_test_db_session() -> Generator[Session, None, None]:
    """
    Test dependency that uses a test database connection
    """
    # This would typically connect to a test database
    # For now, using the same engine but in a real app you'd have a separate test engine
    with Session(engine) as session:
        yield session


# Async version (though SQLModel is synchronous)
async def get_async_db_session():
    """
    Async wrapper for database session (if needed for async endpoints)
    Note: SQLModel is synchronous, so this is just a wrapper
    """
    with Session(engine) as session:
        yield session