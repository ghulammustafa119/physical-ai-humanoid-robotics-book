#!/usr/bin/env python3
"""
Script to ingest book documents into the vector database
"""
import asyncio
import sys
import os
from pathlib import Path

# Add the backend directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent))

from services.document_ingestion import document_ingestion_service

async def main():
    print("Starting document ingestion process...")

    # Check if the docs directory exists
    docs_path = "physical-ai-book/docs"
    if not os.path.exists(docs_path):
        print(f"Error: Docs directory not found at {docs_path}")
        print("Make sure you have the book content in the physical-ai-book/docs directory")
        return

    print(f"Ingesting documents from {docs_path}...")
    result = await document_ingestion_service.ingest_documents(docs_path)

    print(f"Result: {result['message']}")
    print(f"Processed documents: {result['processed_documents']}")
    print(f"Stored chunks: {result['stored_chunks']}")

if __name__ == "__main__":
    asyncio.run(main())