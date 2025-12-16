# Physical AI & Humanoid Robotics — AI-Native Book with Integrated RAG Chatbot Constitution

## Core Principles

### I. Technical Accuracy
Technical claims must be verifiable against official documentation or research from Tier 1 sources (official docs) or Tier 2 sources (peer-reviewed papers). All code examples must run or clearly state assumptions.

### II. AI-Native Methodology
Follow Spec-Driven Development (SDD) using Spec-Kit Plus for all implementations. Every feature must start with clear specifications, plans, and testable tasks before implementation.

### III. Test-First (NON-NEGOTIABLE)
TDD mandatory: Specifications written → User approved → Tests fail → Then implement. Red-Green-Refactor cycle strictly enforced for all code changes.

### IV. Reproducibility
All code, configurations, and architectures must be traceable and reproducible. Every system component must include clear setup instructions and dependency specifications.

### V. Systems Thinking
Design end-to-end solutions covering AI → Simulation → Robotics → Deployment. Consider integration points between all system components from initial design.

### VI. RAG Integrity
RAG chatbot must answer questions strictly from the book content with proper source attribution. No hallucinated content outside the knowledge base is allowed.

## Additional Constraints

Technology Stack:
- Platform: Docusaurus
- Deployment: GitHub Pages
- Backend: FastAPI
- Database: Neon Serverless Postgres
- Vector DB: Qdrant Cloud (Free Tier)
- AI SDKs: OpenAI Agents / ChatKit
- Spec framework: Spec-Kit Plus
- Code language: Python (primary), TypeScript where required

Content Standards:
- Writing style: instructional, precise, and implementation-focused
- Reading level: advanced undergraduate to graduate (CS/AI)
- Every module must include conceptual explanation, architecture overview, practical examples, and code snippets

Module Structure:
- ROS 2 (Robotic Nervous System)
- Gazebo & Unity (Digital Twin)
- NVIDIA Isaac (AI-Robot Brain)
- Vision-Language-Action (VLA)

## Development Workflow

Spec-first approach:
- All features begin with specification (specs/<feature>/spec.md)
- Followed by architectural plan (specs/<feature>/plan.md)
- Then testable tasks (specs/<feature>/tasks.md)
- Finally implementation with PHR tracking

Quality Gates:
- All technical claims verified against official documentation
- Code examples tested and functional
- Architecture diagrams reflect actual implementation
- RAG responses properly grounded in indexed content

## Governance

Constitution supersedes all other practices; amendments require documentation and approval.
All PRs/reviews must verify compliance with technical accuracy and RAG integrity principles.
Complexity must be justified with clear benefits to the learning objectives.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16
