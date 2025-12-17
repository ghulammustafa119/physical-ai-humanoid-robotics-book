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
- **Backend**: FastAPI
- **Database**: Neon Serverless Postgres
- **Vector DB**: Qdrant Cloud
- **AI SDKs**: OpenAI API
- **Spec Framework**: Spec-Kit Plus
- **Primary Language**: Python, with TypeScript for frontend

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

1. **Install dependencies**:
   ```bash
   npm install  # For Docusaurus frontend
   pip install -r requirements.txt  # For FastAPI backend
   ```

2. **Run the development server**:
   ```bash
   npm run dev  # For Docusaurus frontend
   ```

3. **Start the backend**:
   ```bash
   uvicorn backend.main:app --reload  # For FastAPI backend
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

## Contributing

1. Create feature branch from `main`
2. Follow the SDD workflow (spec → plan → tasks → implement)
3. Create Prompt History Records (PHRs) for all significant changes
4. Submit pull request with clear description of changes

## License

This project is licensed under the terms specified in the repository.