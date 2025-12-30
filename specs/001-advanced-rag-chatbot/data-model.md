# Data Model: Advanced RAG Chatbot

## Core Entities

### TextSelection
- **id**: UUID (primary key)
- **content**: string (the selected text)
- **start_position**: integer (character position where selection starts)
- **end_position**: integer (character position where selection ends)
- **book_section**: string (reference to book section/chapter)
- **created_at**: datetime (timestamp of selection)
- **user_id**: UUID (optional, for user-specific selections)

### ChatSession
- **id**: UUID (primary key)
- **user_id**: UUID (optional, for user-specific sessions)
- **created_at**: datetime
- **updated_at**: datetime
- **active**: boolean (whether session is currently active)
- **selected_text_id**: UUID (foreign key to TextSelection, optional)

### UserQuery
- **id**: UUID (primary key)
- **session_id**: UUID (foreign key to ChatSession)
- **query_text**: string (user's question)
- **selected_text_id**: UUID (foreign key to TextSelection, optional)
- **timestamp**: datetime
- **query_type**: enum (e.g., "initial", "followup", "context_change")

### ChatResponse
- **id**: UUID (primary key)
- **query_id**: UUID (foreign key to UserQuery)
- **response_text**: string (AI-generated response)
- **sources**: JSON (list of source references with page/chapter info)
- **confidence_score**: float (0-1 confidence in response accuracy)
- **timestamp**: datetime
- **token_usage**: JSON (input_tokens, output_tokens, total_tokens)

### BookContent
- **id**: UUID (primary key)
- **title**: string (chapter/section title)
- **content**: string (full text content)
- **book_id**: string (identifier for the book)
- **section_path**: string (hierarchical path like "chapter-1/section-2")
- **page_number**: integer (page reference)
- **vector_embedding**: binary (vector representation for semantic search)
- **created_at**: datetime
- **updated_at**: datetime

## Relationships

```
TextSelection (1) <- (n) UserQuery
ChatSession (1) <- (n) UserQuery
UserQuery (1) -> (1) ChatResponse
BookContent (n) -> (n) ChatResponse (via sources)
```

## Validation Rules

### TextSelection
- Content must be between 10 and 5000 characters
- Start position must be less than end position
- Book section must exist in the book content index

### ChatSession
- Cannot have more than 50 queries per session
- Session must be associated with valid user if user_id is provided

### UserQuery
- Query text must be between 1 and 1000 characters
- Must have either selected_text_id or be part of a session with selected text

### ChatResponse
- Response text must be provided
- Sources must be valid references to book content
- Confidence score must be between 0 and 1

### BookContent
- Content must be between 50 and 10000 characters
- Section path must follow the format "chapter-X/section-Y"
- Vector embedding must exist for semantic search