# Research: Urdu Translation Button & Claude Code Subagents

## Decision 1: Translation API Pattern

**Decision**: Reuse existing AI service fallback pattern (Cohere → Gemini → OpenAI)
**Rationale**: Same pattern already works for personalization endpoint. No new dependencies needed.
**Alternatives considered**: Google Translate API (adds dependency, costs money), dedicated translation model (overkill for hackathon)

## Decision 2: Authentication for Translation

**Decision**: No authentication required for translation
**Rationale**: Translation is a content feature, not a user-specific feature. Any visitor should be able to translate. Reduces friction.
**Alternatives considered**: Require login (adds friction, not required by hackathon spec)

## Decision 3: Agent Skills Location

**Decision**: Place in `.claude/commands/` as markdown files
**Rationale**: This is the standard Claude Code convention. Project already has `sp.*` skills there.
**Alternatives considered**: Custom MCP server (too complex), separate repo (not reusable within project)

## Decision 4: Content Truncation

**Decision**: Truncate to ~6000 chars (same as personalize endpoint)
**Rationale**: LLM context window limit. Consistent with existing pattern.
**Alternatives considered**: Chunked translation (complex, slow, harder to reassemble)
