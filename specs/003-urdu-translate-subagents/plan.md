# Implementation Plan: Urdu Translation Button & Claude Code Subagents

**Branch**: `003-urdu-translate-subagents` | **Date**: 2026-03-02 | **Spec**: [spec.md](spec.md)

## Summary

Add a "Translate to Urdu" button to every chapter page (following PersonalizeButton pattern) and create 5 reusable Claude Code agent skills for book development. Translation uses the same AI service fallback (Cohere → Gemini → OpenAI) as the existing personalization endpoint.

## Technical Context

**Language/Version**: Python 3.11 (backend), TypeScript 5.0+ (frontend)
**Primary Dependencies**: FastAPI, Docusaurus 3, React 18, better-auth
**Storage**: Neon Serverless Postgres, Qdrant Cloud
**Testing**: Manual API testing via curl, npm run build
**Target Platform**: GitHub Pages (frontend), Hugging Face Spaces (backend)
**Project Type**: Web application (separate frontend + backend)

## Constitution Check

| Gate | Status | Notes |
|------|--------|-------|
| AI-Native Methodology (SDD) | PASS | Following spec → plan → tasks → implement |
| Tech Stack Compliance | PASS | Using FastAPI, Docusaurus, same AI services |
| Reproducibility | PASS | Same patterns as existing PersonalizeButton |
| RAG Integrity | N/A | Translation is separate from RAG |

## Project Structure

### Source Code (repository root)

```text
backend/
├── src/
│   ├── api/v1/
│   │   ├── translate.py          # NEW — translation endpoint
│   │   └── personalize.py        # REFERENCE — pattern to follow
│   ├── api/main.py               # EDIT — register translate router
│   └── services/
│       ├── cohere_service.py     # REUSE — AI fallback
│       ├── gemini_service.py     # REUSE — AI fallback
│       └── openai_service.py     # REUSE — AI fallback

physical-ai-book/
├── src/
│   ├── components/
│   │   ├── TranslateButton/
│   │   │   ├── index.tsx         # NEW — translate button component
│   │   │   └── styles.module.css # NEW — translate button styles
│   │   └── PersonalizeButton/    # REFERENCE — pattern to follow
│   ├── utils/auth.ts             # EDIT — add translateChapter()
│   └── theme/DocItem/Layout/
│       └── index.tsx             # EDIT — add TranslateButton

.claude/commands/
├── book-chapter.md               # NEW — chapter writing skill
├── book-translate.md             # NEW — translation skill
├── book-personalize.md           # NEW — personalization skill
├── rag-index.md                  # NEW — RAG indexing skill
└── book-review.md                # NEW — content review skill
```

## Implementation Steps

### Step 1: Backend — Translation Endpoint

**File**: `backend/src/api/v1/translate.py` (NEW)
- Copy pattern from `backend/src/api/v1/personalize.py`
- POST `/api/v1/translate`
- Request: `{ chapter_content: str, chapter_title: str }`
- Response: `{ translated_content: str, source_language: str, target_language: str }`
- No authentication required (any user can translate)
- Prompt: "Translate the following technical content about {title} to Urdu. Keep all code blocks, technical terms, variable names, and command examples in English. Translate explanatory text, descriptions, and instructions to Urdu."
- Truncate content to ~6000 chars
- Same AI fallback: Cohere → Gemini → OpenAI

**File**: `backend/src/api/main.py` (EDIT)
- Import `translate_router` from `.v1.translate`
- Register at `settings.api_v1_prefix` with tag `["translation"]`

### Step 2: Frontend — Translate Button Component

**File**: `physical-ai-book/src/components/TranslateButton/index.tsx` (NEW)
- Copy PersonalizeButton pattern
- Message: "Click to translate this chapter to Urdu"
- Button text: "Translate to Urdu"
- Loading: "Translating..." with spinner
- "Show Original" button after translation
- Cache key: `translate_{pathname}` in sessionStorage
- Extract content from `.theme-doc-markdown`
- No auth check needed (public feature)

**File**: `physical-ai-book/src/components/TranslateButton/styles.module.css` (NEW)
- Same structure as PersonalizeButton styles
- Use green accent color (`#2e7d32`) to differentiate from blue personalize

**File**: `physical-ai-book/src/utils/auth.ts` (EDIT)
- Add `translateChapter(chapterContent, chapterTitle)` function
- POST to `${API_BASE_URL}/api/v1/translate`
- No Bearer token needed
- Returns `{ translated_content, source_language, target_language }` or null

**File**: `physical-ai-book/src/theme/DocItem/Layout/index.tsx` (EDIT)
- Import and add `<TranslateButton />` below `<PersonalizeButton />`

### Step 3: Claude Code Agent Skills

Create 5 skill files in `.claude/commands/`:

**`book-chapter.md`**: Generates chapter outlines and content from learning objectives. Delegates to research subagent for technical accuracy verification.

**`book-translate.md`**: Translates book content to Urdu with quality checks. Includes prompt templates for technical translation.

**`book-personalize.md`**: Personalizes content based on user skill levels. Includes prompt templates for beginner/intermediate/advanced adaptations.

**`rag-index.md`**: Indexes book content into Qdrant vector DB. Includes chunking strategy and embedding workflow.

**`book-review.md`**: Reviews book content for technical accuracy, code validity, and completeness. Includes validation checklist.

### Step 4: Deploy

- `npm run build` in physical-ai-book
- Push to GitHub (frontend auto-deploys via GitHub Actions)
- Copy backend files to HF Spaces repo and push

## Verification

1. `npm run build` — no build errors
2. `curl -X POST .../api/v1/translate` with sample content → Urdu translation returned
3. Live site: "Translate to Urdu" button visible on every chapter
4. Click "Translate to Urdu" → Urdu content appears
5. Click "Show Original" → English content returns
6. Agent skills: `/book-chapter`, `/book-translate` etc. listed in Claude Code
