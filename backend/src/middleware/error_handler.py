from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
import logging
from typing import Callable, Awaitable
from starlette.middleware.base import BaseHTTPMiddleware


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class LoggingMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: Request, call_next: Callable) -> None:
        logger.info(f"Request: {request.method} {request.url}")
        response = await call_next(request)
        logger.info(f"Response: {response.status_code}")
        return response


async def http_exception_handler(request: Request, exc: HTTPException):
    """
    Custom HTTP exception handler
    """
    logger.error(f"HTTP Exception: {exc.status_code} - {exc.detail}")
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": {
                "type": "http_exception",
                "message": exc.detail,
                "status_code": exc.status_code
            }
        }
    )


async def general_exception_handler(request: Request, exc: Exception):
    """
    General exception handler for unhandled exceptions
    """
    logger.error(f"Unhandled Exception: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": {
                "type": "internal_error",
                "message": "An internal server error occurred",
                "status_code": 500
            }
        }
    )


# Additional middleware can be added here