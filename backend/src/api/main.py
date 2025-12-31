from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from ..middleware.error_handler import LoggingMiddleware
from .v1.chat import router as chat_router
from .v1.text_selection import router as text_selection_router
from .v1.book_content import router as book_content_router
from .v1.auth import router as auth_router
from ..config.settings import settings


def create_app() -> FastAPI:
    app = FastAPI(
        title=settings.app_name,
        version=settings.app_version,
        debug=settings.debug,
        docs_url="/docs" if settings.debug else None,
        redoc_url="/redoc" if settings.debug else None,
    )

    # Add custom middleware
    app.add_middleware(LoggingMiddleware)

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://localhost:3000", "http://localhost:3001", "http://127.0.0.1:3000"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Include API routers
    app.include_router(
        chat_router,
        prefix=settings.api_v1_prefix,
        tags=["chat"]
    )

    app.include_router(
        auth_router,
        prefix=settings.api_v1_prefix,
        tags=["authentication"]
    )

    app.include_router(
        text_selection_router,
        prefix=settings.api_v1_prefix,
        tags=["text-selection"]
    )

    app.include_router(
        book_content_router,
        prefix=settings.api_v1_prefix,
        tags=["book-content"]
    )

    @app.get("/")
    async def root():
        return {"message": "Advanced RAG Chatbot API", "version": settings.app_version}

    @app.get("/health")
    async def health_check():
        return {"status": "healthy", "version": settings.app_version}

    return app


app = create_app()