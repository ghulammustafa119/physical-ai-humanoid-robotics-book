# Physical AI & Humanoid Robotics — AI-Native Book with Integrated RAG Chatbot

This project is an AI-native book focusing on Physical AI and Humanoid Robotics with an integrated RAG (Retrieval-Augmented Generation) chatbot. The book covers end-to-end systems from AI development through simulation to real-world robotics deployment.

## Project Structure

```
├── .specify/                 # Spec-Kit Plus configuration
├── specs/                    # Specifications for all features
│   ├── book/                 # Book content specifications
│   ├── rag/                  # RAG chatbot specifications
│   ├── ros2/                 # ROS 2 module specifications
│   ├── gazebo/               # Gazebo simulation specifications
│   ├── nvidia_isaac/         # NVIDIA Isaac specifications
│   └── vla/                  # Vision-Language-Action specifications
├── docs/                     # Docusaurus documentation site
│   ├── intro.md              # Introduction page
│   ├── ros2/chapters/        # Module 1: ROS 2 chapters (1-4)
│   ├── gazebo/chapters/      # Module 2: Gazebo & Unity chapters (1-4)
│   ├── isaac/chapters/       # Module 3: NVIDIA Isaac chapters (1-4)
│   └── vla/chapters/         # Module 4: Vision-Language-Action chapters (1-4)
├── backend/                  # FastAPI backend for RAG system
├── history/                  # Prompt History Records and ADRs
└── package.json              # Frontend dependencies (Docusaurus)
└── requirements.txt          # Backend dependencies (FastAPI)
└── sidebars.js               # Navigation configuration
```

## Features

- **Complete Book Content**: 4 comprehensive modules with 4 chapters each (16 total chapters)
- **Module 1**: The Robotic Nervous System (ROS 2) - Understanding ROS 2 as the middleware backbone
- **Module 2**: The Digital Twin (Gazebo & Unity) - Physics simulation and high-fidelity visualization
- **Module 3**: AI-Robot Brain (NVIDIA Isaac™) - AI decision-making systems for humanoid robots
- **Module 4**: Vision-Language-Action (VLA) - Connecting perception to action execution
- **Interactive RAG Chatbot**: Ask questions about book content with source attribution
- **Proper Docusaurus Integration**: Full navigation, proper frontmatter, and optimized for search
- **RAG-Ready Content**: Structured for semantic search and retrieval-augmented generation
- **Code Examples**: All examples in Python with proper formatting and explanations

## Technology Stack

- **Platform**: Docusaurus v3.1.0
- **Deployment**: GitHub Pages
- **Backend**: FastAPI + Python 3.11+
- **Database**: Neon Serverless Postgres
- **Vector DB**: Qdrant Cloud
- **AI Model**: Cohere API (command-r-08-2024)
- **Spec Framework**: Spec-Kit Plus
- **Primary Language**: Python, with TypeScript for frontend
- **Chat UI**: React + TypeScript integrated into Docusaurus

## Book Modules Overview

### Module 1: The Robotic Nervous System (ROS 2)
- Chapter 1: ROS 2 as the Robotic Nervous System
- Chapter 2: ROS 2 Communication Primitives (nodes, topics, services, actions)
- Chapter 3: Bridging Python AI Agents to ROS 2 Controllers
- Chapter 4: Modeling Humanoid Robots with URDF

### Module 2: The Digital Twin (Gazebo & Unity)
- Chapter 1: Gazebo Overview and Physics Simulation
- Chapter 2: Sensor Simulation in Gazebo
- Chapter 3: Unity for High-Fidelity Interaction
- Chapter 4: Bridging Gazebo and Unity

### Module 3: AI-Robot Brain (NVIDIA Isaac™)
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
- Cohere API key (free tier available at https://cohere.com/)

### Installation

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd hackathon_book
   ```

2. **Install frontend dependencies**:
   ```bash
   cd physical-ai-book
   npm install
   ```

3. **Install backend dependencies**:
   ```bash
   cd ../backend
   python -m venv venv_simple
   source venv_simple/bin/activate  # On Windows: venv_simple\Scripts\activate
   pip install fastapi uvicorn pydantic pydantic-settings cohere python-dotenv
   ```

4. **Configure environment variables**:
   ```bash
   cd backend
   cp .env.example .env
   # Edit .env and add your API keys:
   # - COHERE_API_KEY=your_cohere_api_key
   # - DATABASE_URL=your_neon_postgres_url
   # - QDRANT_URL=your_qdrant_cloud_url
   # - QDRANT_API_KEY=your_qdrant_api_key
   ```

### Running the Application

1. **Start the backend server** (Terminal 1):
   ```bash
   cd backend
   source venv_simple/bin/activate
   python main.py
   # Backend will run on http://localhost:8000
   ```

2. **Start the Docusaurus frontend** (Terminal 2):
   ```bash
   cd physical-ai-book
   npm start
   # Frontend will run on http://localhost:3000
   ```

3. **Access the application**:
   - Open browser: `http://localhost:3000/physical-ai-humanoid-robotics-book/`
   - Look for the purple chat button in the bottom-right corner
   - Click to open the AI chat assistant
   - Ask questions about robotics, ROS 2, or book content

### Testing the Chat API

Test the backend directly:
```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"query":"What is ROS 2?"}'
```

## Development Workflow

This project follows Spec-Driven Development (SDD) methodology:

1. **Specification**: Define requirements in `specs/<feature>/spec.md`
2. **Planning**: Create architectural plans in `specs/<feature>/plan.md`
3. **Tasks**: Break down implementation in `specs/<feature>/tasks.md`
4. **Implementation**: Code and test following the specifications
5. **Documentation**: Update documentation and create PHRs

## Content Structure

All book content is properly integrated with:
- YAML frontmatter (title, description, sidebar_position)
- Proper heading hierarchy for RAG-friendly chunking
- Correctly formatted Python code blocks
- Sequential sidebar positioning for navigation
- Self-contained chapters without forward references

Backend RAG ingests content from a mirrored docs directory for clean separation from frontend book build.

## Troubleshooting

### Backend Issues

**Problem**: `ModuleNotFoundError: No module named 'cohere'`
- **Solution**: Make sure you activated the virtual environment and installed dependencies:
  ```bash
  cd backend
  source venv_simple/bin/activate
  pip install cohere
  ```

**Problem**: `404 error: model 'command-r' was removed`
- **Solution**: Update your `.env` file to use the new model:
  ```env
  COHERE_MODEL=command-r-08-2024
  ```
  Then restart the backend server.

**Problem**: Chat button not appearing
- **Solution**: Clear browser cache and ensure Docusaurus rebuilt:
  ```bash
  cd physical-ai-book
  rm -rf .docusaurus build
  npm start
  ```

**Problem**: CORS errors in browser console
- **Solution**: Ensure backend is running on port 8000 and `ALLOWED_ORIGINS=*` in `.env`

### Getting API Keys

- **Cohere**: Sign up at https://cohere.com/ for free tier (1000 calls/month)
- **Qdrant Cloud**: Sign up at https://cloud.qdrant.io/ for free tier
- **Neon Postgres**: Sign up at https://neon.tech/ for free serverless database

## Contributing

1. Create feature branch from `main`
2. Follow the SDD workflow (spec → plan → tasks → implement)
3. Create Prompt History Records (PHRs) for all significant changes
4. Submit pull request with clear description of changes

## License

This project is licensed under the terms specified in the repository.

<!-- Trigger GitHub Actions rebuild -->
<!-- New trigger for deployment -->