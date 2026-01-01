import uvicorn
import os
from src.api.main import app

if __name__ == "__main__":
    # Hugging Face Spaces requires binding to 0.0.0.0 and port 7860
    port = int(os.environ.get("PORT", 7860))
    uvicorn.run(app, host="0.0.0.0", port=port)
