---
id: 1
title: Spec Analysis
stage: general
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: book
branch: 001-book-feature
user: default_user
command: /sp.analyze
labels: ["analysis","spec","cross-artifact"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - ./specs/book/spec.md
 - ./specs/book/plan.md
 - ./specs/book/tasks.md
 - .specify/memory/constitution.md
tests:
 - null
---

## Prompt

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Goal

Identify inconsistencies, duplications, ambiguities, and underspecified items across the three core artifacts (`spec.md`, `plan.md`, `tasks.md`) before implementation. This command MUST run only after `/sp.tasks` has successfully produced a complete `tasks.md`.

## Operating Constraints

**STRICTLY READ-ONLY**: Do **not** modify any files. Output a structured analysis report. Offer an optional remediation plan (user must explicitly approve before any follow-up editing commands would be invoked manually).

**Constitution Authority**: The project constitution (`.specify/memory/constitution.md`) is **non-negotiable** within this analysis scope. Constitution conflicts are automatically CRITICAL and require adjustment of the spec, plan, or tasks—not dilution, reinterpretation, or silent ignoring of the principle. If a principle itself needs to change, that must occur in a separate, explicit constitution update outside `/sp.analyze`.

## Execution Steps

### 1. Initialize Analysis Context

Run `.specify/scripts/bash/check-prerequisites.sh --json --require-tasks --include-tasks` once from repo root and parse JSON for FEATURE_DIR and AVAILABLE_DOCS. Derive absolute paths:

- SPEC = FEATURE_DIR/spec.md
- PLAN = FEATURE_DIR/plan.md
- TASKS = FEATURE_DIR/tasks.md

Abort with an error message if any required file is missing (instruct the user to run missing prerequisite command).
For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

### 2. Load Artifacts (Progressive Disclosure)

Load only the minimal necessary context from each artifact:

**From spec.md:**

- Overview/Context
- Functional Requirements
- Non-Functional Requirements
- User Stories
- Edge Cases (if present)

**From plan.md:**

- Architecture/stack choices
- Data Model references
- Phases
- Technical constraints

**From tasks.md:**

- Task IDs
- Descriptions
- Phase grouping
- Parallel markers [P]
- Referenced file paths

**From constitution:**

- Load `.specify/memory/constitution.md` for principle validation

### 3. Build Semantic Models

Create internal representations (do not include raw artifacts in output):

- **Requirements inventory**: Each functional + non-functional requirement with a stable key (derive slug based on imperative phrase; e.g., "User can upload file" → `user-can-upload-file`)
- **User story/action inventory**: Discrete user actions with acceptance criteria
- **Task coverage mapping**: Map each task to one or more requirements or stories (inference by keyword / explicit reference patterns like IDs or key phrases)
- **Constitution rule set**: Extract principle names and MUST/SHOULD normative statements

### 4. Detection Passes (Token-Efficient Analysis)

Focus on high-signal findings. Limit to 50 findings total; aggregate remainder in overflow summary.

#### A. Duplication Detection

- Identify near-duplicate requirements
- Mark lower-quality phrasing for consolidation

#### B. Ambiguity Detection

- Flag vague adjectives (fast, scalable, secure, intuitive, robust) lacking measurable criteria
- Flag unresolved placeholders (TODO, TKTK, ???, `<placeholder>`, etc.)

#### C. Underspecification

- Requirements with verbs but missing object or measurable outcome
- User stories missing acceptance criteria alignment
- Tasks referencing files or components not defined in spec/plan

#### D. Constitution Alignment

- Any requirement or plan element conflicting with a MUST principle
- Missing mandated sections or quality gates from constitution

#### E. Coverage Gaps

- Requirements with zero associated tasks
- Tasks with no mapped requirement/story
- Non-functional requirements not reflected in tasks (e.g., performance, security)

#### F. Inconsistency

- Terminology drift (same concept named differently across files)
- Data entities referenced in plan but absent in spec (or vice versa)
- Task ordering contradictions (e.g., integration tasks before foundational setup tasks without dependency note)
- Conflicting requirements (e.g., one requires Next.js while other specifies Vue)

### 5. Severity Assignment

Use this heuristic to prioritize findings:

- **CRITICAL**: Violates constitution MUST, missing core spec artifact, or requirement with zero coverage that blocks baseline functionality
- **HIGH**: Duplicate or conflicting requirement, ambiguous security/performance attribute, untestable acceptance criterion
- **MEDIUM**: Terminology drift, missing non-functional task coverage, underspecified edge case
- **LOW**: Style/wording improvements, minor redundancy not affecting execution order

### 6. Produce Compact Analysis Report

Output a Markdown report (no file writes) with the following structure:

## Specification Analysis Report

| ID | Category | Severity | Location(s) | Summary | Recommendation |
|----|----------|----------|-------------|---------|----------------|
| A1 | Duplication | HIGH | spec.md:L120-134 | Two similar requirements ... | Merge phrasing; keep clearer version |

(Add one row per finding; generate stable IDs prefixed by category initial.)

**Coverage Summary Table:**

| Requirement Key | Has Task? | Task IDs | Notes |
|-----------------|-----------|----------|-------|

**Constitution Alignment Issues:** (if any)

**Unmapped Tasks:** (if any)

**Metrics:**

- Total Requirements
- Total Tasks
- Coverage % (requirements with >=1 task)
- Ambiguity Count
- Duplication Count
- Critical Issues Count

### 7. Provide Next Actions

At end of report, output a concise Next Actions block:

- If CRITICAL issues exist: Recommend resolving before `/sp.implement`
- If only LOW/MEDIUM: User may proceed, but provide improvement suggestions
- Provide explicit command suggestions: e.g., "Run /sp.specify with refinement", "Run /sp.plan to adjust architecture", "Manually edit tasks.md to add coverage for 'performance-metrics'"

### 8. Offer Remediation

Ask the user: "Would you like me to suggest concrete remediation edits for the top N issues?" (Do NOT apply them automatically.)

## Operating Principles

### Context Efficiency

- **Minimal high-signal tokens**: Focus on actionable findings, not exhaustive documentation
- **Progressive disclosure**: Load artifacts incrementally; don't dump all content into analysis
- **Token-efficient output**: Limit findings table to 50 rows; summarize overflow
- **Deterministic results**: Rerunning without changes should produce consistent IDs and counts

### Analysis Guidelines

- **NEVER modify files** (this is read-only analysis)
- **NEVER hallucinate missing sections** (if absent, report them accurately)
- **Prioritize constitution violations** (these are always CRITICAL)
- **Use examples over exhaustive rules** (cite specific instances, not generic patterns)
- **Report zero issues gracefully** (emit success report with coverage statistics)

## Context

$ARGUMENTS

---

## Response snapshot

## Specification Analysis Report

| ID | Category | Severity | Location(s) | Summary | Recommendation |
|----|----------|----------|-------------|---------|----------------|
| D1 | Duplication | MEDIUM | spec.md:30-40, plan.md:66-74 | Similar API descriptions in both spec and plan | Consolidate to avoid confusion; spec defines WHAT, plan defines HOW |
| A1 | Ambiguity | HIGH | spec.md:43-45 | "Performance: Page load times under 3 seconds" lacks specificity | Define measurable criteria: desktop/mobile, network conditions, metric tools |
| A2 | Ambiguity | HIGH | spec.md:44-45 | "99% uptime" and "concurrent users during hackathon" undefined | Specify time periods, concurrent user counts, and uptime measurement windows |
| U1 | Underspecification | MEDIUM | tasks.md | Tasks lack specific acceptance criteria for RAG quality | Add measurable criteria for response accuracy and source attribution |
| C1 | Constitution Alignment | CRITICAL | plan.md:56 | Plan uses OpenAI embeddings despite cost concerns in risks | Align with constitution principle of staying within free tier limits |
| C2 | Constitution Alignment | HIGH | tasks.md | No explicit TDD/test-first approach mentioned in tasks | Add unit tests before implementation per constitution III |
| G1 | Coverage Gap | MEDIUM | tasks.md | Non-functional requirements not fully covered in tasks | Add tasks for performance testing, reliability monitoring |
| G2 | Coverage Gap | MEDIUM | tasks.md | No tasks for content verification against official docs in Phase 3 | Add verification tasks for RAG content accuracy |
| I1 | Inconsistency | MEDIUM | spec.md:15 vs plan.md:20-27 | Spec mentions Unity but plan doesn't address it consistently | Clarify Unity's role in digital twin approach |
| I2 | Inconsistency | LOW | spec.md:59-61 vs plan.md:59-61 | Different wording of "smallest viable change" principle | Align terminology between documents |

**Coverage Summary Table:**

| Requirement Key | Has Task? | Task IDs | Notes |
|-----------------|-----------|----------|-------|
| book-platform-deployable | Yes | 1.1, 1.3 | Covered in Phase 1 |
| content-modules-complete | Yes | 2.1-2.4 | Covered in Phase 2 |
| rag-chatbot-answers-content | Yes | 3.1-3.4 | Covered in Phase 3 |
| code-examples-runnable | Partial | 2.1-2.4 | Need specific verification tasks |
| performance-page-load | Partial | N/A | Missing specific performance tasks |
| reliability-uptime | No | N/A | No reliability monitoring tasks |
| scalability-concurrent-users | No | N/A | No scalability testing tasks |

**Constitution Alignment Issues:**
- Critical: Plan mentions OpenAI usage but risks section acknowledges cost concerns - conflicts with free-tier constraint
- High: Tasks don't explicitly follow TDD/test-first approach as mandated in constitution
- Medium: No explicit verification of technical claims against official docs in RAG implementation tasks

**Unmapped Tasks:**
- Tasks 1.3 (deployment pipeline), 4.3 (UX testing), 5.1-5.2 (final deployment) have weak mapping to specific functional requirements

**Metrics:**
- Total Requirements: 12 (functional + non-functional)
- Total Tasks: 23 main task categories
- Coverage % (requirements with >=1 task): 75%
- Ambiguity Count: 2
- Duplication Count: 1
- Critical Issues Count: 1

### Next Actions

CRITICAL issues must be resolved before `/sp.implement`: The constitution mandates free-tier usage but the plan relies on paid OpenAI services. Adjust either the plan to use free/local alternatives or update the constitution.

For LOW/MEDIUM issues: Consider refining the performance requirements with specific metrics, aligning the Unity digital twin approach between spec and plan, and adding explicit testing tasks that follow the TDD approach mandated in the constitution.

Command suggestions:
- Run `/sp.plan` to adjust OpenAI service approach for free-tier compliance
- Manually edit tasks.md to add specific TDD test creation before implementation
- Add performance testing tasks to cover the non-functional requirements

### Remediation Offer

Would you like me to suggest concrete remediation edits for the top 5 issues identified above?

## Outcome

- ✅ Impact: Identified critical constitution violation requiring plan adjustment to align with free-tier constraints
- 🧪 Tests: N/A (analysis activity)
- 📁 Files: ./specs/book/spec.md, ./specs/book/plan.md, ./specs/book/tasks.md, .specify/memory/constitution.md
- 🔁 Next prompts: /sp.plan to adjust architecture, /sp.tasks to add TDD tasks
- 🧠 Reflection: Constitution violations must be resolved before implementation; cross-artifact consistency is crucial for project success

## Evaluation notes (flywheel)

- Failure modes observed: Constitution violations that could lead to budget overruns
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Adjust plan to use local embedding models instead of OpenAI