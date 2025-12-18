# Physical AI & Humanoid Robotics Book

This repository contains an AI-native book focusing on Physical AI and Humanoid Robotics with an integrated RAG (Retrieval-Augmented Generation) chatbot. The book covers end-to-end systems from AI development through simulation to real-world robotics deployment, emphasizing architecture-first and systems thinking approaches.

## Project Overview

Physical AI represents a paradigm shift from software-only AI to embodied, multimodal intelligence that interacts with the physical world. This book addresses the critical gap between large language models and real-world robotics by demonstrating how Vision-Language-Action (VLA) systems can create truly intelligent agents.

## Installation

```bash
npm install
```

## Local Development

```bash
npm run start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Book Structure

### Module 1: ROS 2 — Robotic Nervous System
- Distributed computing framework for robotics
- Message passing and service architecture
- Node management and communication patterns
- Real-world deployment strategies

### Module 2: Simulation (Gazebo Primary)
- Gazebo as primary digital twin platform
- Physics-based simulation environments
- Sensor integration and visualization
- Unity as optional future enhancement

### Module 3: NVIDIA Isaac & Physics-Based Learning
- AI-robot brain architecture
- Perception systems and sensor fusion
- Physics-informed learning approaches
- Control systems and action planning

### Module 4: Vision-Language-Action (VLA)
- Multimodal intelligence systems
- End-to-end learning architectures
- Real-world interaction patterns
- Embodied AI principles
