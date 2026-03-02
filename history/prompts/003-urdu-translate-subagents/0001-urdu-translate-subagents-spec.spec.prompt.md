---
id: "0001"
title: "Urdu Translate Subagents Spec"
stage: spec
date: 2026-03-02
surface: agent
model: claude-opus-4-6
feature: 003-urdu-translate-subagents
branch: 003-urdu-translate-subagents
user: mustafa
command: /sp.specify
labels: ["urdu", "translation", "subagents", "hackathon"]
links:
  spec: specs/003-urdu-translate-subagents/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-urdu-translate-subagents/spec.md
  - specs/003-urdu-translate-subagents/checklists/requirements.md
tests:
  - none
---

## Prompt

Urdu Translation Button and Claude Code Subagents - Two features: (1) Add a "Translate to Urdu" button at the start of each chapter that translates content to Urdu using AI, with a "Show Original" button to revert. Follow same pattern as existing PersonalizeButton. Backend endpoint at /api/v1/translate using Cohere/Gemini/OpenAI fallback. (2) Create reusable Claude Code agent skills for book-chapter writing, translation, personalization, RAG indexing, and content review in .claude/commands/

## Response snapshot

Created feature spec with 2 user stories (P1: Urdu translation button, P2: Claude Code agent skills), 10 functional requirements, 6 success criteria, 5 edge cases. All quality checklist items pass. No clarifications needed. Ready for /sp.plan.

## Outcome

- Impact: Two hackathon bonus features specified (100 pts total)
- Tests: none
- Files: spec.md, checklists/requirements.md
- Next prompts: /sp.plan to create implementation plan
- Reflection: Clean spec, no ambiguity — both features well-defined with existing patterns to follow

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): Quality checklist PASS (all items)
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a
