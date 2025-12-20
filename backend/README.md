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

## Prerequisites

- Python 3.8+
- Google API key for Gemini
- Qdrant vector database (can be local or cloud)
- PostgreSQL database (Neon recommended)

## Setup

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Set up environment variables:
```bash
cp .env .env.local
# Edit .env.local with your actual configuration
```

3. Start the API server:
```bash
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

## Environment Variables

- `GOOGLE_API_KEY`: Your Google API key for Gemini
- `QDRANT_URL`: URL to your Qdrant instance
- `QDRANT_API_KEY`: API key for Qdrant (if using cloud)
- `DATABASE_URL`: PostgreSQL connection string
- `NEON_DATABASE_URL`: Neon PostgreSQL connection string

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
4. The LLM generates a response based on the retrieved context
5. Responses include source attribution to the original book content