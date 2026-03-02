# Tasks: Urdu Translation Button & Claude Code Subagents

**Branch**: `003-urdu-translate-subagents` | **Date**: 2026-03-02 | **Plan**: [plan.md](plan.md)

## Phase 1: Backend

### Task 1.1: Create translation endpoint [DONE]
- **File**: `backend/src/api/v1/translate.py` (NEW)
- **Covers**: FR-001, FR-002, FR-003, FR-007, FR-008
- **Description**: POST `/api/v1/translate` — accepts `chapter_content` and `chapter_title`, returns `translated_content`, `source_language`, `target_language`. No auth. Cohere → Gemini → OpenAI fallback. Truncates to 6000 chars.
- **Test**: `curl -X POST .../api/v1/translate -d '{"chapter_content":"...", "chapter_title":"..."}'` returns Urdu content

### Task 1.2: Register translate router [DONE]
- **File**: `backend/src/api/main.py` (EDIT)
- **Covers**: FR-001
- **Description**: Import `translate_router` and register with `settings.api_v1_prefix`, tag `["translation"]`
- **Test**: `/api/v1/translate` endpoint responds (not 404)

## Phase 2: Frontend

### Task 2.1: Create TranslateButton component [DONE]
- **File**: `physical-ai-book/src/components/TranslateButton/index.tsx` (NEW)
- **Covers**: FR-001, FR-002, FR-004, FR-005, FR-006, FR-007
- **Description**: React component following PersonalizeButton pattern. Shows "Translate to Urdu" button, loading spinner, "Show Original" toggle. Caches in sessionStorage. RTL panel with `dir="rtl"` for Urdu content.
- **Test**: Button visible on chapter pages, click translates, Show Original reverts

### Task 2.2: Create TranslateButton styles [DONE]
- **File**: `physical-ai-book/src/components/TranslateButton/styles.module.css` (NEW)
- **Covers**: FR-001
- **Description**: Green accent (#2e7d32) to differentiate from blue PersonalizeButton. Responsive, dark mode support.
- **Test**: Visual verification on chapter page

### Task 2.3: Add translateChapter utility [DONE]
- **File**: `physical-ai-book/src/utils/auth.ts` (EDIT)
- **Covers**: FR-002
- **Description**: `translateChapter(chapterContent, chapterTitle)` — POST to `/api/v1/translate`, no Bearer token, returns result or null.
- **Test**: Function returns translated content from API

### Task 2.4: Integrate TranslateButton into DocItem Layout [DONE]
- **File**: `physical-ai-book/src/theme/DocItem/Layout/index.tsx` (EDIT)
- **Covers**: FR-001
- **Description**: Import TranslateButton, render below PersonalizeButton.
- **Test**: `npm run build` passes, button appears on chapters

## Phase 3: Claude Code Agent Skills

### Task 3.1: Create book-chapter skill [DONE]
- **File**: `.claude/commands/book-chapter.md` (NEW)
- **Covers**: FR-009, FR-010
- **Description**: Chapter writing skill with research subagent delegation, outline generation, quality verification.
- **Test**: `/book-chapter` invokable in Claude Code

### Task 3.2: Create book-translate skill [DONE]
- **File**: `.claude/commands/book-translate.md` (NEW)
- **Covers**: FR-009, FR-010
- **Description**: Translation skill with glossary, quality checks, Urdu script rules.
- **Test**: `/book-translate` invokable in Claude Code

### Task 3.3: Create book-personalize skill [DONE]
- **File**: `.claude/commands/book-personalize.md` (NEW)
- **Covers**: FR-009, FR-010
- **Description**: Personalization skill for beginner/intermediate/advanced level adaptation.
- **Test**: `/book-personalize` invokable in Claude Code

### Task 3.4: Create rag-index skill [DONE]
- **File**: `.claude/commands/rag-index.md` (NEW)
- **Covers**: FR-009, FR-010
- **Description**: RAG indexing skill with chunking strategy and Qdrant workflow.
- **Test**: `/rag-index` invokable in Claude Code

### Task 3.5: Create book-review skill [DONE]
- **File**: `.claude/commands/book-review.md` (NEW)
- **Covers**: FR-009, FR-010
- **Description**: Content review skill with technical accuracy, code validity, and completeness checks.
- **Test**: `/book-review` invokable in Claude Code

## Phase 4: Deploy

### Task 4.1: Build and deploy [DONE]
- **Covers**: SC-006
- **Description**: `npm run build`, push to GitHub (frontend), push to HF Spaces (backend).
- **Test**: Build passes, live site shows TranslateButton, API returns Urdu translation.
