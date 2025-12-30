# Quickstart Guide: Advanced RAG Chatbot

## Prerequisites

- Python 3.11+
- Node.js 18+ (for frontend development)
- Docker (for local development environment)
- OpenAI API key
- Qdrant Cloud account
- Neon Postgres account

## Environment Setup

### Backend Setup
```bash
# Navigate to the backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install backend dependencies
pip install -r requirements.txt

# Set up environment variables
cp .env.example .env
# Edit .env with your API keys and database URLs
```

### Frontend Setup
```bash
# Navigate to the frontend directory
cd frontend
npm install
```

## Configuration

### Environment Variables (.env)
```bash
# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_ASSISTANT_ID=your_assistant_id_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_content

# Neon Postgres Configuration
DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname

# Application Configuration
APP_ENV=development
DEBUG=true
```

## Running the Application

### Backend (FastAPI)
```bash
cd backend
python -m uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

### Frontend (React)
```bash
cd frontend
npm start
```

## API Endpoints

### Chat API
- `POST /api/v1/chat` - Send query and receive response
- `GET /api/v1/chat/sessions` - List chat sessions
- `GET /api/v1/chat/sessions/{session_id}` - Get specific session

### Text Selection API
- `POST /api/v1/text-selection` - Create text selection
- `GET /api/v1/text-selection/{selection_id}` - Get specific selection

### Book Content API
- `GET /api/v1/book-content` - List book sections
- `GET /api/v1/book-content/{section_id}` - Get specific section

## Testing

### Backend Tests
```bash
cd backend
pytest tests/unit/ -v
pytest tests/integration/ -v
```

### Frontend Tests
```bash
cd frontend
npm test
```

## Deployment

### Local Development
```bash
# Run both backend and frontend
# Backend: python -m uvicorn src.api.main:app --reload
# Frontend: npm start
```

### Production Build
```bash
# Backend: Deploy to cloud provider (AWS, GCP, Azure)
# Frontend: Build and deploy to CDN or static hosting
cd frontend
npm run build
```

## Troubleshooting

### Common Issues
1. **Qdrant Connection**: Verify QDRANT_URL and QDRANT_API_KEY in .env
2. **Database Connection**: Check DATABASE_URL format and credentials
3. **OpenAI API**: Ensure OPENAI_API_KEY is valid and has proper permissions

### Debugging API Calls
- Enable DEBUG=true in .env for detailed logging
- Check application logs in backend for errors
- Use browser dev tools to inspect API requests/responses