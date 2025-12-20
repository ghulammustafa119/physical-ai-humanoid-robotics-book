#!/bin/bash
# Script to run the backend server

# Check if virtual environment exists, if not create it
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Install dependencies
echo "Installing dependencies..."
pip install -r requirements.txt

# Run the server
echo "Starting the RAG API server..."
uvicorn main:app --host 0.0.0.0 --port 8000 --reload