# Physical AI & Humanoid Robotics Book - RAG API

This is the backend API for the Physical AI & Humanoid Robotics Book RAG chatbot. It allows users to ask questions about the book content and get accurate answers with source attribution.

## Features

- FastAPI-based REST API
- RAG (Retrieval-Augmented Generation) functionality
- Vector database storage with Qdrant
- Document ingestion from book content
- Semantic search and response generation
- Query caching for performance
- Content synchronization
- User feedback collection

## Deployment

### Hugging Face Spaces (Backend)

The backend is configured for deployment as a Python SDK Space on Hugging Face.

#### Hugging Face Deployment Steps:

1. Create a new Space on [Hugging Face](https://huggingface.co/new-space).
2. Select **Python SDK** as the Space SDK.
3. Commit the `backend/` directory content (including `app.py`, `src/`, etc.) to the Space repository.
4. Set the following **Secrets** in your Space settings:
   - `DATABASE_URL`: Your PostgreSQL connection string (e.g., from Neon).
   - `SECRET_KEY`: A secure random string for session tokens.
   - `COHERE_API_KEY`: Your Cohere API key.
   - `ENVIRONMENT`: Set to `production`.
5. The Space will automatically deploy using `app.py` as the entrypoint on port 7860.

#### Production Auth Architecture:

- **Frontend**: GitHub Pages (`https://ghulammustafa119.github.io/physical-ai-humanoid-robotics-book`)
- **Backend**: Hugging Face Spaces (`https://<your-space-name>.hf.space`)
- **Cross-Domain Auth**: Enabled via CORS `allow_origins`, `allow_credentials=True`, and Secure HTTP-only cookies with `SameSite=None`.

**Architecture Diagram:**
```text
[ Browser ] <---(HTTPS)---> [ GitHub Pages (Frontend) ]
    |                              ^
    | (Fetch with credentials)      |
    v                              |
[ Hugging Face Space (Backend) ] --|
    |                              |
    | <---(SQL)--- [ Neon Postgres ]
```

### Environment Variables for Production

| Variable | Description | Recommended for Prod |
|----------|-------------|----------------------|
| `DATABASE_URL` | Postgres connection string | Neon Serverless |
| `SECRET_KEY` | Secure random string | `openssl rand -hex 32` |
| `ENVIRONMENT` | Deployment environment | `production` |
| `ALLOWED_ORIGINS` | CORS origins | Configured in `main.py` |

## Prerequisites

- Python 3.11+
- Cohere API key (free tier available at https://cohere.com/)
- Qdrant vector database (cloud recommended)
- PostgreSQL database (Neon Serverless recommended)

## Setup

1. Create virtual environment:
```bash
python -m venv venv_simple
source venv_simple/bin/activate  # On Windows: venv_simple\Scripts\activate
```

2. Install dependencies:
```bash
pip install fastapi uvicorn pydantic pydantic-settings cohere python-dotenv
```

3. Set up environment variables:
```bash
cp .env.example .env
# Edit .env with your actual configuration
```

Required environment variables in `.env`:
```env
# Cohere Configuration (REQUIRED)
COHERE_API_KEY=your_cohere_api_key_here
COHERE_MODEL=command-r-08-2024

# Database Configuration
DATABASE_URL=postgresql://user:password@host/database

# Qdrant Configuration
QDRANT_URL=https://your-qdrant-instance.com
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_content

# Application Settings
DEBUG=True
ENVIRONMENT=development
ALLOWED_ORIGINS=*
```

4. Start the API server:
```bash
python main.py
# Or with uvicorn directly:
# uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

## Environment Variables

### AI Model Configuration
- `COHERE_API_KEY`: Your Cohere API key (get free tier at https://cohere.com/)
- `COHERE_MODEL`: Model to use (default: `command-r-08-2024`)

### Database Configuration
- `DATABASE_URL`: PostgreSQL connection string (Neon Serverless recommended)

### Vector Database Configuration
- `QDRANT_URL`: URL to your Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `QDRANT_COLLECTION_NAME`: Collection name for book embeddings (default: `book_content`)

### Application Settings
- `DEBUG`: Enable debug mode (default: `True`)
- `ENVIRONMENT`: Environment name (default: `development`)
- `ALLOWED_ORIGINS`: CORS allowed origins (default: `*` for development)
- `RAG_MAX_TOKENS`: Max tokens for AI responses (default: `2000`)
- `RAG_TEMPERATURE`: Temperature for generation (default: `0.7`)

### Optional: Alternative AI Models
The system also supports Google Gemini and OpenAI as fallback options:
- `GOOGLE_APPLICATION_CREDENTIALS`: Path to Google service account JSON
- `GEMINI_MODEL`: Google Gemini model name
- `OPENAI_API_KEY`: OpenAI API key
- `OPENAI_MODEL`: OpenAI model name

## API Endpoints

### Chat
- `POST /api/v1/chat` - Ask questions about book content
- `POST /api/v1/validate-query` - Validate a query

### Documents
- `POST /api/v1/ingest` - Ingest book documents into vector DB
- `GET /api/v1/status` - Get ingestion status
- `POST /api/v1/reindex` - Reindex all documents

### Integration
- `POST /api/v1/sync-content` - Synchronize content with book
- `GET /api/v1/content-mapping` - Get content mapping
- `POST /api/v1/feedback` - Submit feedback on responses
- `GET /api/v1/fallback-content` - Get fallback content

### Health
- `GET /api/v1/health` - Health check
- `GET /api/v1/ready` - Readiness check

## Ingesting Book Content

To ingest your book content into the RAG system:

1. Make sure your book content is in the `physical-ai-book/docs` directory as markdown files
2. Call the ingestion endpoint:
```bash
curl -X POST http://localhost:8000/api/v1/ingest
```

Or run the ingestion script:
```bash
python ingest_documents.py
```

## Testing the API

Once the API is running and content is ingested, you can test it:

```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2 in robotics?"}'
```

## Architecture

The RAG system works as follows:
1. Book content is ingested and stored as vector embeddings in Qdrant
2. User queries are converted to embeddings and searched against the vector database
3. Relevant content is retrieved and used as context for the LLM
4. Cohere's `command-r-08-2024` model generates a response based on the retrieved context
5. Responses include source attribution to the original book content

## Chat Interface Integration

The backend API is integrated with a Docusaurus-based frontend featuring:

- **Toggle-able Chat Panel**: Purple gradient button in bottom-right corner
- **Real-time AI Responses**: Powered by Cohere API
- **Clean UI**: Modern design with smooth animations
- **Context-Aware**: Can answer questions about book content
- **Source Attribution**: Responses include references to original content

### Frontend Integration Files
Located in `physical-ai-book/src/`:
- `components/ChatPanel/index.tsx` - Main chat component
- `components/ChatPanel/styles.module.css` - Chat styling
- `theme/Root.tsx` - Global chat panel injection

### API Request Format
```typescript
POST /api/v1/chat
Content-Type: application/json

{
  "query": "What is ROS 2?",
  "selectedText": "optional selected text from book",
  "sessionId": "optional session id"
}
```

### API Response Format
```json
{
  "query_id": "uuid",
  "response_text": "AI generated response...",
  "sources": [
    {
      "title": "Source Title",
      "url": "/book/section",
      "page": 1,
      "section": "1.1",
      "text_snippet": "Relevant text..."
    }
  ],
  "confidence_score": 0.95,
  "token_usage": {
    "input_tokens": 10,
    "output_tokens": 150,
    "total_tokens": 160
  }
}
```

## AI Model: Cohere command-r-08-2024

This system uses Cohere's `command-r-08-2024` model which offers:
- **Free Tier Available**: 1000 API calls per month
- **High Quality Responses**: Advanced reasoning capabilities
- **Fast Performance**: Low latency for real-time chat
- **Multilingual Support**: Works in multiple languages
- **RAG-Optimized**: Designed for retrieval-augmented generation

**Note**: Previous models (`command-r`, `command-r-plus`) were deprecated September 15, 2025.