# Physical AI & Humanoid Robotics Book - Architectural Plan

## 1. Scope and Dependencies

### In Scope
- Docusaurus-based book platform with modular content structure
- Content modules: ROS 2, Gazebo, NVIDIA Isaac, Vision-Language-Action (Unity optional)
- FastAPI backend for RAG chatbot functionality
- Neon Postgres for metadata storage
- Qdrant vector database for document embeddings
- Integration layer between book content and RAG system

### Out of Scope
- Hardware control systems
- Real-time robot control interfaces
- Third-party service provisioning
- Advanced mathematical derivations

### External Dependencies
- ROS 2 ecosystem (Humble Hawksbill or later)
- NVIDIA Isaac Sim/Gym environments
- Gazebo simulation platform
- Unity Digital Twin (optional)
- OpenAI API for embeddings/completions
- Docusaurus framework
- Qdrant Cloud service

## 2. Key Decisions and Rationale

### Technology Stack Selection
**Options Considered**:
- Static site generators: Docusaurus vs Hugo vs Jekyll
- Backend frameworks: FastAPI vs Express vs Flask
- Vector databases: Qdrant vs Pinecone vs Weaviate
- Database: Neon Postgres vs Supabase vs traditional Postgres

**Trade-offs**:
- Docusaurus offers excellent documentation features and React-based customization
- FastAPI provides async support and automatic API documentation
- Qdrant offers open-source compatibility and cloud options
- Neon provides serverless Postgres with git-like branching

**Rationale**: Selected stack balances developer productivity, performance, and hackathon requirements.

### RAG Architecture
**Options Considered**:
- Embedding models: OpenAI ada-002 vs local models (SentenceTransformers)
- Retrieval strategies: Semantic search vs hybrid search vs dense retrieval
- Response generation: Direct prompting vs chain-of-thought vs structured output

**Trade-offs**:
- OpenAI embeddings are consistent but cost money; local models are free but require maintenance
- Semantic search is simpler but hybrid might offer better precision
- Direct prompting is faster but structured output provides more control

**Rationale**: Using OpenAI ada-002 for consistency and quality, semantic search for simplicity, direct prompting for speed.

### Principles
- Smallest viable change: Start with basic RAG functionality, expand iteratively
- Measurable: Define clear performance metrics (response time, accuracy)
- Reversible: Use feature flags and modular architecture

## 3. Interfaces and API Contracts

### Public APIs
**Book Content API**:
- Input: Request for content by module/chapter
- Output: Markdown content with metadata
- Errors: 404 for missing content, 500 for server errors

**RAG Chatbot API**:
- Input: Query string, optional context parameters
- Output: Response text with source citations
- Errors: 400 for invalid queries, 500 for processing failures

### Versioning Strategy
- API versioning via URL path (e.g., /api/v1/chat)
- Backward compatibility maintained for minor versions
- Breaking changes require new major version

### Error Handling
- Idempotency: Safe to retry GET requests
- Timeouts: 30-second limit for API requests
- Retries: Exponential backoff for transient failures

### Error Taxonomy
- 400: Client errors (malformed requests, invalid parameters)
- 404: Resource not found
- 500: Server errors (processing failures, service unavailable)

## 4. Non-Functional Requirements (NFRs) and Budgets

### Performance
- p95 latency: Under 2 seconds for chat responses
- Throughput: Support 10 concurrent users during demo
- Resource caps: Stay within free tier limits for Qdrant and Neon

### Reliability
- SLOs: 99% availability during hackathon
- Error budget: 1% acceptable downtime
- Degradation strategy: Fallback to static content if RAG unavailable

### Security
- AuthN/AuthZ: None required for public book content
- Data handling: No personal data collection
- Secrets: Environment variables for API keys
- Auditing: Log access patterns for debugging

### Cost
- Unit economics: Target free tier usage for all services
- Monitoring: Track API usage to avoid surprise costs

## 5. Data Management and Migration

### Source of Truth
- Primary: Markdown files in docs/src directory
- Secondary: Vector embeddings in Qdrant
- Metadata: Neon Postgres tables

### Schema Evolution
- Document embedding schema: versioned by model type
- Metadata schema: versioned with clear migration paths

### Migration and Rollback
- Embedding migrations: Re-process documents with new model
- Schema migrations: Versioned SQL scripts with rollback capability

### Data Retention
- Embeddings: Retained as long as source documents exist
- Query logs: Temporary for debugging, no permanent retention

## 6. Operational Readiness

### Observability
- Logs: Request/response logging for debugging
- Metrics: Response times, error rates, API usage
- Traces: Request flow through system components

### Alerting
- Thresholds: Response time > 5 seconds, error rate > 5%
- On-call: Not required for demo system

### Runbooks
- Common tasks: Document rebuild, embedding refresh, API key rotation
- Troubleshooting: Connection issues, performance problems

### Deployment and Rollback Strategies
- Deployment: GitHub Actions CI/CD pipeline
- Rollback: Git revert with redeployment
- Feature Flags: Gradual rollout of new functionality

## 7. Risk Analysis and Mitigation

### Top 3 Risks
1. **API Costs**: Uncontrolled OpenAI usage could exceed budget
   - Mitigation: Rate limiting, usage monitoring, cost alerts
2. **Service Dependencies**: Third-party services (Qdrant, Neon) could be unavailable
   - Mitigation: Fallback to static content, redundant providers
3. **Content Quality**: Technical inaccuracies could mislead readers
   - Mitigation: Verification against official docs, peer review process

### Blast Radius
- Small: Individual API endpoints
- Medium: Module-specific functionality
- Large: Full RAG system

### Kill Switches/Guardrails
- API rate limiting to control costs
- Circuit breakers for external dependencies
- Content validation before RAG indexing

## 8. Evaluation and Validation

### Definition of Done
- Tests: Unit tests for API endpoints, integration tests for RAG flow
- Scans: Security scanning for dependencies, content verification
- Documentation: Setup guides, API documentation, troubleshooting

### Output Validation
- Format: Responses follow expected JSON schema
- Requirements: Content stays within book scope
- Safety: No hallucinated information outside knowledge base

## 9. Architecture Diagram

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Book User     │    │   RAG User       │    │  Admin Console  │
└─────────┬───────┘    └─────────┬────────┘    └─────────┬───────┘
          │                      │                       │
          ▼                      ▼                       ▼
    ┌─────────────┐      ┌──────────────────┐      ┌─────────────┐
    │ Docusaurus  │      │   FastAPI        │      │  Content    │
    │   Frontend  │      │   Backend        │      │  Management │
    └─────────────┘      └──────────────────┘      └─────────────┘
           │                       │                       │
           │              ┌────────▼────────┐             │
           └──────────────►               ◄───────────────┘
                          │   Neon Postgres │
                          │   (Metadata)    │
                          └────────▲────────┘
                                   │
                    ┌──────────────┴──────────────┐
                    │                             │
              ┌─────▼─────┐                ┌──────▼──────┐
              │  Qdrant   │                │OpenAI APIs  │
              │(Vectors)  │                │(Embeddings) │
              └───────────┘                └─────────────┘
```