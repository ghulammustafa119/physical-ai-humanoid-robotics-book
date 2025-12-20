import os
import asyncio
from typing import List, Dict, Any
from pathlib import Path
from services.vector_db import vector_db_service
from config.settings import settings
import logging
import hashlib
from langchain.text_splitter import RecursiveCharacterTextSplitter

class DocumentIngestionService:
    def __init__(self):
        self.vector_db = vector_db_service
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=settings.chunk_size,
            chunk_overlap=settings.chunk_overlap,
            length_function=len,
        )

    def read_book_content(self, source_path: str = "physical-ai-book/docs") -> List[Dict[str, Any]]:
        """
        Read book content from Docusaurus docs directory
        """
        documents = []

        source_dir = Path(source_path)
        if not source_dir.exists():
            logging.error(f"Source directory does not exist: {source_path}")
            return documents

        # Find all markdown files in the docs directory
        for md_file in source_dir.rglob("*.md"):
            try:
                with open(md_file, 'r', encoding='utf-8') as f:
                    content = f.read()

                    # Create metadata
                    metadata = {
                        "source": str(md_file.relative_to(source_dir)),
                        "file_path": str(md_file),
                        "file_name": md_file.name,
                        "directory": str(md_file.parent.relative_to(source_dir))
                    }

                    documents.append({
                        "content": content,
                        "metadata": metadata
                    })
            except Exception as e:
                logging.error(f"Error reading file {md_file}: {e}")

        return documents

    def chunk_document(self, content: str, metadata: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Split document content into chunks
        """
        chunks = self.text_splitter.split_text(content)
        chunked_docs = []

        for i, chunk in enumerate(chunks):
            chunk_metadata = metadata.copy()
            chunk_metadata["chunk_index"] = i
            chunk_metadata["total_chunks"] = len(chunks)

            chunked_docs.append({
                "content": chunk,
                "metadata": chunk_metadata
            })

        return chunked_docs

    async def ingest_documents(self, source_path: str = "physical-ai-book/docs") -> Dict[str, Any]:
        """
        Ingest all documents from the source path into the vector database
        """
        try:
            # Read all book content
            documents = self.read_book_content(source_path)
            logging.info(f"Found {len(documents)} documents to process")

            total_chunks = 0
            processed_docs = 0

            for doc in documents:
                # Chunk each document
                chunked_docs = self.chunk_document(doc["content"], doc["metadata"])

                # Store each chunk in the vector database
                for chunk_doc in chunked_docs:
                    # Generate a deterministic ID based on content and metadata
                    content_hash = hashlib.md5(
                        (chunk_doc["content"] + str(chunk_doc["metadata"])).encode()
                    ).hexdigest()

                    try:
                        doc_id = self.vector_db.store_document(
                            chunk_doc["content"],
                            chunk_doc["metadata"],
                            doc_id=content_hash
                        )
                        total_chunks += 1
                        logging.debug(f"Stored chunk with ID: {doc_id}")
                    except Exception as e:
                        logging.error(f"Error storing chunk: {e}")
                        continue

                processed_docs += 1

            return {
                "status": "success",
                "processed_documents": processed_docs,
                "stored_chunks": total_chunks,
                "message": f"Successfully ingested {processed_docs} documents with {total_chunks} chunks"
            }

        except Exception as e:
            logging.error(f"Error during document ingestion: {e}")
            return {
                "status": "error",
                "processed_documents": 0,
                "stored_chunks": 0,
                "message": f"Error during document ingestion: {str(e)}"
            }

    def get_ingestion_status(self) -> Dict[str, Any]:
        """
        Get current ingestion status
        """
        try:
            # In a real implementation, we'd check the actual vector DB
            # For now, we'll return a placeholder
            collection_info = self.vector_db.client.get_collection(
                collection_name=self.vector_db.collection_name
            )

            return {
                "status": "ready",
                "collection_name": self.vector_db.collection_name,
                "vectors_count": collection_info.points_count,
                "message": "Vector database is ready for queries"
            }
        except Exception as e:
            logging.error(f"Error getting ingestion status: {e}")
            return {
                "status": "error",
                "message": f"Error getting ingestion status: {str(e)}"
            }

# Global instance
document_ingestion_service = DocumentIngestionService()