# Physical AI & Humanoid Robotics Book

An AI-native interactive book on Physical AI and Humanoid Robotics, built with Docusaurus and powered by an Advanced RAG chatbot. Features user authentication via [better-auth](https://www.better-auth.com/), personalized content based on user profiles, and AI-driven chapter explanations.

**Live:** [ghulammustafa119.github.io/physical-ai-humanoid-robotics-book](https://ghulammustafa119.github.io/physical-ai-humanoid-robotics-book/)

## Tech Stack

| Layer | Technology |
|-------|-----------|
| Frontend | Docusaurus 3, React 18, TypeScript |
| Authentication | better-auth client SDK |
| Backend | FastAPI (Python 3.11) |
| AI/LLM | OpenAI Agents SDK, Cohere, Gemini |
| Vector DB | Qdrant Cloud |
| Database | Neon Serverless Postgres |
| Hosting | GitHub Pages (frontend), Hugging Face Spaces (backend) |

## Features

- **Advanced RAG Chatbot** — Ask questions about any chapter and get AI-generated answers grounded in book content
- **User Authentication** — Sign up / sign in via better-auth with session management
- **Personalized Content** — Per-chapter personalization based on user's programming level, hardware, and experience
- **Interactive Book** — Full Docusaurus-powered documentation site with search, navigation, and dark mode

## Project Structure

```
physical-ai-book/          # Frontend (Docusaurus)
├── src/
│   ├── components/        # React components (auth, chat, personalize)
│   ├── lib/auth-client.ts # better-auth client instance
│   ├── utils/auth.ts      # Auth utilities and API helpers
│   └── theme/             # Swizzled Docusaurus theme components
├── docs/                  # Book content (Markdown)
└── docusaurus.config.ts

backend/                   # Backend (FastAPI)
├── src/
│   ├── api/v1/            # API routes (chat, auth, personalize, book-content)
│   ├── services/          # Business logic (auth, RAG, AI)
│   ├── models/            # SQLModel database models
│   └── config/            # Settings and configuration
```

## Getting Started

### Frontend

```bash
cd physical-ai-book
npm install
npm run start
```

### Backend

```bash
cd backend
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
uvicorn src.api.main:app --reload
```

### Environment Variables (Backend)

Create a `.env` file in the `backend/` directory:

```env
DATABASE_URL=postgresql://...
QDRANT_URL=https://...
QDRANT_API_KEY=...
OPENAI_API_KEY=...
COHERE_API_KEY=...
GEMINI_API_KEY=...
```

## Book Modules

### Module 1: ROS 2 — Robotic Nervous System
Distributed computing framework, message passing, node management, and deployment strategies.

### Module 2: Simulation (Gazebo Primary)
Digital twin platform, physics-based simulation, sensor integration, and visualization.

### Module 3: NVIDIA Isaac & Physics-Based Learning
AI-robot brain architecture, perception systems, physics-informed learning, and control systems.

### Module 4: Vision-Language-Action (VLA)
Multimodal intelligence, end-to-end learning architectures, and embodied AI principles.

## Deployment

- **Frontend:** Auto-deploys to GitHub Pages on push to `main`
- **Backend:** Deployed on Hugging Face Spaces at `ghulammustafabhutto/gmbhutto`
