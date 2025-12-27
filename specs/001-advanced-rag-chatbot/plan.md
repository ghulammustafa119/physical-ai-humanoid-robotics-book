# Implementation Plan: Advanced RAG Chatbot – User-Selected Text Query & Interactive UI

**Branch**: `001-advanced-rag-chatbot` | **Date**: 2025-12-27 | **Spec**: [link to spec](./spec.md)
**Input**: Feature specification from `/specs/001-advanced-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of an Advanced RAG Chatbot that allows users to select text within the Physical AI & Humanoid Robotics book and get contextual answers restricted to that selected text. The system will integrate with OpenAI Agents/ChatKit SDK for real-time responses while maintaining RAG integrity by ensuring answers only come from indexed book content with proper source attribution. The solution includes both backend API endpoints and frontend UI components for text selection and interactive chat functionality.

## Technical Context

**Language/Version**: Python 3.11, TypeScript 5.0+
**Primary Dependencies**: FastAPI, OpenAI Agents SDK, Qdrant client, Neon Postgres, React 18+
**Storage**: Neon Serverless Postgres (relational data), Qdrant Cloud (vector embeddings), file storage for book content
**Testing**: pytest for backend, Jest/React Testing Library for frontend
**Target Platform**: Web application (Docusaurus-based book interface)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: 95% of queries respond within 3 seconds, 90%+ source attribution accuracy
**Constraints**: <3s p95 response time, RAG integrity enforced (no external content), proper source attribution
**Scale/Scope**: Single book content, concurrent users up to 100, multi-turn conversations supported

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

1. **Technical Accuracy**: All implementations must be verified against official documentation (FastAPI, OpenAI Agents SDK, Qdrant, Neon Postgres)
2. **AI-Native Methodology**: Follow Spec-Driven Development with SDD using Spec-Kit Plus
3. **Test-First (NON-NEGOTIABLE)**: Specifications written → User approved → Tests fail → Then implement
4. **Reproducibility**: All code, configurations, and architectures must be traceable and reproducible
5. **Systems Thinking**: Design end-to-end solution covering AI → UI → Backend → Vector DB integration
6. **RAG Integrity**: Chatbot must answer questions strictly from book content with proper source attribution, no hallucinated content

## Project Structure

### Documentation (this feature)

```text
specs/001-advanced-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── text_selection.py      # Text selection data models
│   │   ├── chat_session.py        # Chat session models
│   │   ├── book_content.py        # Book content models
│   │   └── response.py            # Response models with attribution
│   ├── services/
│   │   ├── rag_service.py         # RAG service with text selection logic
│   │   ├── vector_db_service.py   # Vector database operations
│   │   ├── chat_service.py        # Chat session management
│   │   └── openai_service.py      # OpenAI Agents integration
│   ├── api/
│   │   ├── v1/
│   │   │   ├── chat.py           # Chat endpoints
│   │   │   ├── text_selection.py # Text selection endpoints
│   │   │   └── book_content.py   # Book content endpoints
│   │   └── main.py               # API router
│   └── config/
│       ├── settings.py            # Configuration settings
│       └── database.py            # Database configuration
└── tests/
    ├── unit/
    │   ├── models/
    │   ├── services/
    │   └── api/
    ├── integration/
    │   ├── chat_integration_test.py
    │   └── rag_integration_test.py
    └── contract/
        └── api_contract_test.py

frontend/
├── src/
│   ├── components/
│   │   ├── TextSelector.tsx       # Text selection UI component
│   │   ├── ChatInterface.tsx      # Chat interface component
│   │   ├── MessageDisplay.tsx     # Message display with sources
│   │   └── SessionManager.tsx     # Session management component
│   ├── services/
│   │   ├── api-client.ts          # API client for backend communication
│   │   └── chat-session.ts        # Frontend chat session management
│   ├── hooks/
│   │   ├── useTextSelection.ts    # Text selection hook
│   │   └── useChatSession.ts      # Chat session hook
│   └── types/
│       ├── chat.ts                # Chat-related TypeScript types
│       └── book.ts                # Book content types
└── tests/
    ├── unit/
    │   ├── components/
    │   └── services/
    └── integration/
        └── chat_integration_test.ts
```

**Structure Decision**: Web application structure selected with separate backend (FastAPI) and frontend (React/TypeScript) components to maintain clear separation of concerns. Backend handles RAG processing, vector DB operations, and OpenAI integration, while frontend provides interactive UI for text selection and chat functionality.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multi-component architecture | Separation of concerns for maintainability | Single monolithic app would create tight coupling between UI and backend logic |
| Vector database integration | Required for semantic search in RAG system | Simple keyword search would not provide contextual relevance needed for book content |
