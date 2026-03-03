# Physical AI & Humanoid Robotics — AI-Native Book with Integrated RAG Chatbot

An AI-native interactive textbook on Physical AI and Humanoid Robotics with an integrated RAG chatbot, Urdu translation, personalized learning, and Claude Code agent skills.

## Live Demo

- **Book**: [ghulammustafa119.github.io/physical-ai-humanoid-robotics-book](https://ghulammustafa119.github.io/physical-ai-humanoid-robotics-book/)
- **Backend API**: [ghulammustafabhutto-gmbhutto.hf.space](https://ghulammustafabhutto-gmbhutto.hf.space/)

## Technology Stack

| Layer | Technology |
|-------|-----------|
| Frontend | Docusaurus 3, React 18, TypeScript 5.0+ |
| Backend | FastAPI, Python 3.11 |
| Authentication | Better Auth (signup, signin, sessions) |
| Database | Neon Serverless Postgres |
| Vector DB | Qdrant Cloud |
| AI Services | Cohere (primary), Gemini, OpenAI (fallback) |
| Deployment | GitHub Pages (frontend), Hugging Face Spaces (backend) |
| Methodology | Spec-Driven Development (SDD) via Spec-Kit Plus |

## Features

- **16 Chapters** across 4 modules covering ROS 2, Gazebo, NVIDIA Isaac, and VLA
- **RAG Chatbot** — Ask questions about book content with source attribution
- **Translate to Urdu** — One-click Urdu translation on every chapter (no login required)
- **Personalized Learning** — Content adapts to your programming, AI/ML, and robotics experience
- **User Authentication** — Sign up, sign in, and profile-based personalization
- **Claude Code Agent Skills** — 5 reusable skills for book development automation

## Claude Code Agent Skills

| Skill | Command | Description |
|-------|---------|-------------|
| Book Chapter | `/book-chapter` | Generate chapter outlines and content from learning objectives |
| Book Translate | `/book-translate` | Translate content to Urdu with technical term preservation |
| Book Personalize | `/book-personalize` | Adapt content for beginner/intermediate/advanced readers |
| RAG Index | `/rag-index` | Index book content into Qdrant vector DB |
| Book Review | `/book-review` | Review content for technical accuracy and code validity |

## Project Structure

```
├── physical-ai-book/           # Docusaurus frontend
│   ├── docs/                   # Book chapters (4 modules × 4 chapters)
│   └── src/
│       ├── components/
│       │   ├── ChatPanel/      # RAG chatbot UI
│       │   ├── PersonalizeButton/ # Content personalization
│       │   ├── TranslateButton/   # Urdu translation
│       │   └── auth/           # SignIn, SignUp, AuthProvider
│       ├── utils/auth.ts       # API utilities
│       └── theme/DocItem/      # Swizzled layout (inject buttons)
├── backend/                    # FastAPI backend
│   └── src/
│       ├── api/v1/             # Endpoints: chat, translate, personalize, auth
│       ├── services/           # Cohere, Gemini, OpenAI, Qdrant, Auth
│       └── config/             # Settings, database
├── .claude/commands/           # Claude Code agent skills (5 skills)
├── specs/                      # Feature specifications (SDD)
└── history/                    # Prompt History Records
```

## Book Modules

### Module 1: The Robotic Nervous System (ROS 2)
- Chapter 1: ROS 2 as the Robotic Nervous System
- Chapter 2: ROS 2 Communication Primitives
- Chapter 3: Bridging Python AI Agents to ROS 2
- Chapter 4: Modeling Humanoid Robots with URDF

### Module 2: The Digital Twin (Gazebo & Unity)
- Chapter 1: Gazebo Overview and Physics Simulation
- Chapter 2: Sensor Simulation in Gazebo
- Chapter 3: Unity for High-Fidelity Interaction
- Chapter 4: Bridging Gazebo and Unity

### Module 3: AI-Robot Brain (NVIDIA Isaac)
- Chapter 1: What Is an AI-Robot Brain?
- Chapter 2: Perception in Physical AI Systems
- Chapter 3: Planning & Decision Making
- Chapter 4: Bridging the AI Brain to ROS 2

### Module 4: Vision-Language-Action (VLA)
- Chapter 1: Introduction to VLA
- Chapter 2: Vision-Language Models (VLMs)
- Chapter 3: Action Planning & Execution
- Chapter 4: Integration & Simulation

## Getting Started

### Prerequisites

- Python 3.11+
- Node.js 18+ and npm
- API keys: Cohere, Neon Postgres, Qdrant Cloud

### Installation

```bash
# Clone
git clone git@github.com:ghulammustafa119/physical-ai-humanoid-robotics-book.git
cd physical-ai-humanoid-robotics-book

# Frontend
cd physical-ai-book && npm install

# Backend
cd ../backend
python -m venv venv_simple
source venv_simple/bin/activate
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your API keys
```

### Running Locally

```bash
# Terminal 1 — Backend
cd backend && source venv_simple/bin/activate && python main.py
# → http://localhost:8000

# Terminal 2 — Frontend
cd physical-ai-book && npm start
# → http://localhost:3000/physical-ai-humanoid-robotics-book/
```

### Testing the APIs

```bash
# Chat
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'

# Translate
curl -X POST http://localhost:8000/api/v1/translate \
  -H "Content-Type: application/json" \
  -d '{"chapter_content": "Robots use sensors.", "chapter_title": "Intro"}'
```

## Development Workflow

This project follows **Spec-Driven Development (SDD)**:

1. `/sp.specify` — Define feature requirements
2. `/sp.plan` — Create implementation plan
3. `/sp.tasks` — Break down into testable tasks
4. `/sp.implement` — Execute the plan
5. `/sp.analyze` — Cross-artifact quality check

## License

This project is licensed under the terms specified in the repository.
