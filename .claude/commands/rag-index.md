# RAG Indexing Skill

Index book content into the Qdrant vector database for the RAG chatbot.

## Usage

```
/rag-index [--chapter <path>] [--all]
```

## Instructions

Index book chapter content into the Qdrant vector store used by the Advanced RAG Chatbot.

1. **Discover content**:
   - If `--chapter` is provided, index only that file
   - If `--all` is provided, find all `.md` files in `physical-ai-book/docs/`
   - List files to be indexed and confirm with the user

2. **Read configuration**:
   - Read `backend/src/config/settings.py` for Qdrant connection settings
   - Read `backend/src/services/qdrant_service.py` for the indexing service interface
   - Understand the chunking strategy and embedding model in use

3. **Prepare chunks**: For each chapter file:
   - Split content into chunks of ~500 tokens with 50-token overlap
   - Preserve section headings as metadata
   - Extract frontmatter (title, description) as metadata
   - Tag each chunk with: `chapter_title`, `section`, `file_path`, `chunk_index`

4. **Index**: Use the backend's existing indexing service:
   - Run the indexing script: `python -m backend.scripts.index_content`
   - Or call the Qdrant service directly if a script is available
   - Verify indexed document count matches expected chunks

5. **Verify**:
   - Run a test query against the indexed content
   - Confirm relevant results are returned
   - Report: total chunks indexed, collection name, any errors

## Chunking Strategy

```
Max chunk size: 500 tokens
Overlap: 50 tokens
Split on: paragraph boundaries (double newline)
Preserve: section headings prepended to each chunk
Metadata: chapter_title, section_heading, file_path, chunk_index
```

## Environment

Requires these environment variables (from `.env`):
- `QDRANT_URL` — Qdrant Cloud endpoint
- `QDRANT_API_KEY` — Qdrant API key
- `COHERE_API_KEY` or `OPENAI_API_KEY` — for embeddings
