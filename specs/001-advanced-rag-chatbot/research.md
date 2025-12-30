# Research Notes: Advanced RAG Chatbot Implementation

## Decision: Text Selection Implementation Approach
**Rationale**: Using browser's Selection API combined with document ranges to capture user-selected text with precise positioning information. This approach allows us to maintain the context of where the selection occurred in the book content.
**Alternatives considered**:
- Simple text highlighting without position tracking (rejected - no context preservation)
- Custom text selection library (rejected - browser API is sufficient)

## Decision: Backend Architecture Pattern
**Rationale**: FastAPI with dependency injection for clean separation of concerns. Using Pydantic models for request/response validation to ensure data integrity. This follows Python best practices and provides async capabilities for handling concurrent requests.
**Alternatives considered**:
- Flask (rejected - less async support and fewer built-in features)
- Django (rejected - overkill for API-only service)

## Decision: Vector Database Strategy
**Rationale**: Qdrant Cloud with semantic search capabilities for finding relevant content within selected text. Qdrant provides efficient similarity search and is well-suited for RAG applications.
**Alternatives considered**:
- Pinecone (rejected - cost considerations for development)
- Weaviate (rejected - Qdrant better integration with Python ecosystem)

## Decision: Frontend State Management
**Rationale**: React hooks with Context API for managing chat sessions and text selections. This provides a clean way to share state across components without excessive prop drilling.
**Alternatives considered**:
- Redux (rejected - overkill for this feature scope)
- Zustand (rejected - Context API sufficient for this use case)

## Decision: OpenAI Integration Pattern
**Rationale**: Using OpenAI Assistants API for conversation management and function calling capabilities. This provides better control over the conversation flow and allows for custom functions to retrieve specific content.
**Alternatives considered**:
- Direct ChatCompletions API (rejected - less control over conversation context)
- LangChain (rejected - adds unnecessary complexity for this use case)

## Decision: Source Attribution Method
**Rationale**: Storing document metadata with vector embeddings and returning source references with each response. This ensures proper attribution while maintaining performance.
**Alternatives considered**:
- Post-hoc attribution (rejected - less accurate)
- Simple document references (rejected - not specific enough)