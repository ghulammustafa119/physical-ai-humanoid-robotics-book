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
├── backend/                  # FastAPI backend for RAG system
├── history/                  # Prompt History Records and ADRs
└── package.json              # Frontend dependencies (Docusaurus)
└── requirements.txt          # Backend dependencies (FastAPI)
```

## Features

- **Modular Book Content**: Organized by technology areas (ROS 2, Gazebo, NVIDIA Isaac, VLA)
- **Interactive RAG Chatbot**: Ask questions about book content with source attribution
- **End-to-End Examples**: Complete implementation guides from AI to deployment
- **Architecture Diagrams**: Visual representations of system components
- **Code Examples**: All examples are tested and functional

## Technology Stack

- **Platform**: Docusaurus
- **Deployment**: GitHub Pages
- **Backend**: FastAPI
- **Database**: Neon Serverless Postgres
- **Vector DB**: Qdrant Cloud
- **AI SDKs**: OpenAI API
- **Spec Framework**: Spec-Kit Plus
- **Primary Language**: Python, with TypeScript for frontend

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

## Contributing

1. Create feature branch from `master`
2. Follow the SDD workflow (spec → plan → tasks → implement)
3. Create Prompt History Records (PHRs) for all significant changes
4. Submit pull request with clear description of changes

## License

This project is licensed under the terms specified in the repository.