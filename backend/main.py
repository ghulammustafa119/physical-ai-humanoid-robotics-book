import uvicorn
from src.api.main import app
from src.config.settings import settings


if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True if settings.debug else False,
        debug=settings.debug
    )