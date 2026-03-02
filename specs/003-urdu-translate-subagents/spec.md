# Feature Specification: Urdu Translation Button & Claude Code Subagents

**Feature Branch**: `003-urdu-translate-subagents`
**Created**: 2026-03-02
**Status**: Draft
**Input**: User description: "Add Translate to Urdu button per chapter + Create reusable Claude Code agent skills"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Translate Chapter to Urdu (Priority: P1)

A reader opens any chapter in the Physical AI book and sees a "Translate to Urdu" button. They click it, the system translates the chapter content to Urdu using AI, and the translated content appears on the page. The reader can click "Show Original" to revert back to English.

**Why this priority**: Core hackathon requirement (#7) worth 50 bonus points. Directly visible to evaluators.

**Independent Test**: Open any chapter page, click "Translate to Urdu", verify Urdu text appears. Click "Show Original", verify English returns.

**Acceptance Scenarios**:

1. **Given** a reader is on any chapter page, **When** they click "Translate to Urdu", **Then** the chapter content is translated to Urdu and displayed on the page
2. **Given** translated content is displayed, **When** reader clicks "Show Original", **Then** the original English content is restored
3. **Given** a reader translates a chapter, **When** they navigate away and return, **Then** the cached translation is shown without re-translating
4. **Given** the translation service is unavailable, **When** reader clicks "Translate to Urdu", **Then** a user-friendly error message is displayed

---

### User Story 2 - Claude Code Agent Skills for Book Development (Priority: P2)

A developer uses Claude Code to work on the book project. They can invoke reusable agent skills (slash commands) like `/book-chapter`, `/book-translate`, `/book-personalize`, `/rag-index`, and `/book-review` to perform specialized book development tasks efficiently.

**Why this priority**: Hackathon requirement (#4) worth 50 bonus points. Demonstrates reusable intelligence via Claude Code Subagents.

**Independent Test**: Run each skill command in Claude Code and verify it produces meaningful, structured output for the given task.

**Acceptance Scenarios**:

1. **Given** a developer is in Claude Code, **When** they run `/book-chapter` with a topic, **Then** a structured chapter outline is generated
2. **Given** a developer is in Claude Code, **When** they run `/book-translate` with content, **Then** translation workflow guidance is provided
3. **Given** a developer is in Claude Code, **When** they run `/book-review`, **Then** content is analyzed for quality and accuracy

---

### Edge Cases

- What happens when chapter content is very short (< 50 characters)? Show "not enough content" error.
- What happens when chapter content is very long? Truncate to fit AI context window.
- What happens when AI service returns partial or malformed translation? Show error, don't display broken content.
- What happens when user clicks translate while translation is in progress? Button is disabled during loading.
- How does the system handle code blocks in translation? Code blocks and technical terms stay in English.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display a "Translate to Urdu" button at the start of each chapter page
- **FR-002**: System MUST translate chapter content to Urdu when the button is clicked
- **FR-003**: System MUST preserve code blocks and technical terms in English during translation
- **FR-004**: System MUST display a "Show Original" button after translation to revert to English
- **FR-005**: System MUST cache translated content per page in the browser session to avoid re-translation
- **FR-006**: System MUST show a loading indicator while translation is in progress
- **FR-007**: System MUST display a user-friendly error message if translation fails
- **FR-008**: System MUST truncate overly long content before sending to AI service
- **FR-009**: System MUST provide at least 5 reusable Claude Code agent skills for book development
- **FR-010**: Each agent skill MUST have clear instructions, input/output expectations, and subagent delegation patterns

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can translate any chapter to Urdu with a single click within 30 seconds
- **SC-002**: Users can revert to original English content with a single click instantly
- **SC-003**: Translated content preserves all code blocks and technical terms in English
- **SC-004**: Translation results are cached per session — repeat visits don't trigger re-translation
- **SC-005**: All 5 Claude Code agent skills are invokable and produce structured output
- **SC-006**: Both features build and deploy without errors to GitHub Pages and HF Spaces

## Assumptions

- Translation uses the same AI service fallback pattern as personalization (Cohere → Gemini → OpenAI)
- Translation does not require user authentication (any visitor can translate)
- The Translate button follows the same UI pattern as the existing Personalize button
- Agent skills are markdown files in `.claude/commands/` following existing sp.* patterns
