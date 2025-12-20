#!/usr/bin/env python3
"""
Script to run the complete ingestion process for the book content
"""
import asyncio
import sys
import os
from pathlib import Path

# Add the backend directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent))

from services.document_ingestion import document_ingestion_service

async def main():
    print("🚀 Starting Physical AI & Humanoid Robotics Book RAG System Setup")
    print("=" * 60)

    # Check if the docs directory exists
    docs_path = "physical-ai-book/docs"
    if not os.path.exists(docs_path):
        print(f"❌ Error: Docs directory not found at {docs_path}")
        print("Make sure you have the book content in the physical-ai-book/docs directory")
        print("\nTo add book content:")
        print("1. Navigate to the physical-ai-book directory")
        print("2. Add your markdown files to the docs/ folder")
        print("3. Run this script again")
        return

    print(f"📖 Found book content in {docs_path}")
    print("📝 Starting document ingestion process...")

    # Get initial status
    initial_status = document_ingestion_service.get_ingestion_status()
    print(f"📊 Initial vector database status: {initial_status['message']}")
    print(f"📦 Current vectors in database: {initial_status['vectors_count']}")

    print("\n🔄 Processing documents...")
    result = await document_ingestion_service.ingest_documents(docs_path)

    print(f"\n✅ {result['message']}")
    print(f"📄 Processed documents: {result['processed_documents']}")
    print(f"🔗 Stored content chunks: {result['stored_chunks']}")

    # Get final status
    final_status = document_ingestion_service.get_ingestion_status()
    print(f"\n📈 Final vector database status: {final_status['message']}")
    print(f"📦 Final vectors in database: {final_status['vectors_count']}")

    print("\n✨ Ingestion process completed successfully!")
    print("\n📋 Next steps:")
    print("1. Start the API server: uvicorn main:app --host 0.0.0.0 --port 8000")
    print("2. Test the chat endpoint to ask questions about your book content")
    print("3. Use the /api/v1/chat endpoint to interact with the RAG system")

if __name__ == "__main__":
    asyncio.run(main())