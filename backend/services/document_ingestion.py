import os
import asyncio
from typing import List, Dict, Any
from pathlib import Path
from services.vector_db import vector_db_service
from config.settings import settings
import logging
import hashlib
from langchain_text_splitters import RecursiveCharacterTextSplitter


class DocumentIngestionService:
    def __init__(self):
        self.vector_db = vector_db_service
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=settings.chunk_size,
            chunk_overlap=settings.chunk_overlap,
            length_function=len,
        )

    def read_book_content(self, source_path: str = None) -> List[Dict[str, Any]]:
        """
        Read book content from Docusaurus docs directory
        """
        documents = []

        # Use default path if none provided, resolving relative to backend directory
        if source_path is None:
            # Get the directory of this file and build path to docs
            current_file_dir = Path(__file__).resolve().parent
            source_dir = current_file_dir.parent / "physical-ai-book" / "docs"
        else:
            source_dir = Path(source_path)
        if not source_dir.exists():
            logging.error(f"Source directory does not exist: {source_path}")
            return documents

        # Find all markdown and PDF files in the docs directory recursively
        for file_path in source_dir.rglob("*"):
            if file_path.suffix.lower() in ['.md', '.pdf']:
                try:
                    if file_path.suffix.lower() == '.md':
                        # Read markdown files
                        with open(file_path, 'r', encoding='utf-8') as f:
                            content = f.read()
                    elif file_path.suffix.lower() == '.pdf':
                        # For PDF files, we'll need to extract text
                        try:
                            from pypdf import PdfReader
                            with open(file_path, 'rb') as f:
                                pdf_reader = PdfReader(f)
                                content = ""
                                for page in pdf_reader.pages:
                                    page_text = page.extract_text()
                                    if page_text:  # Check if page text is not None or empty
                                        content += page_text + "\n"
                                    else:
                                        logging.warning(f"Page in PDF {file_path} returned empty text")
                        except ImportError:
                            logging.warning(f"pypdf not installed, skipping PDF file: {file_path}")
                            continue
                        except Exception as e:
                            logging.error(f"Error reading PDF file {file_path}: {e}")
                            continue
                    else:
                        continue  # Should not happen due to the suffix check above

                    # Add debug logging
                    content_length = len(content.strip()) if content else 0
                    logging.info(f"File: {file_path}, Content length: {content_length}")

                    # Hard guard: if extracted text is empty, skip the file
                    if not content or len(content.strip()) == 0:
                        logging.warning(f"Skipping file with empty content: {file_path}")
                        continue

                    # Create metadata
                    metadata = {
                        "source": str(file_path.relative_to(source_dir)),
                        "file_path": str(file_path),
                        "file_name": file_path.name,
                        "directory": str(file_path.parent.relative_to(source_dir)),
                        "file_type": file_path.suffix.lower()
                    }

                    documents.append({
                        "content": content,
                        "metadata": metadata
                    })
                except Exception as e:
                    logging.error(f"Error reading file {file_path}: {e}")

        return documents

    def chunk_document(self, content: str, metadata: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Split document content into chunks
        """
        # Add debug logging
        logging.info(f"Chunking document: {metadata.get('file_name', 'unknown')}, original content length: {len(content)}")

        chunks = self.text_splitter.split_text(content)
        logging.info(f"Number of chunks produced: {len(chunks)}")

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

    async def ingest_documents(self, source_path: str = None) -> Dict[str, Any]:
        """
        Ingest all documents from the source path into the vector database
        """
        try:
            # Read all book content
            documents = self.read_book_content(source_path)
            logging.info(f"Found {len(documents)} documents with non-empty content to process")

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
                logging.info(f"Processed document {processed_docs}/{len(documents)}, stored {len(chunked_docs)} chunks for {doc['metadata']['file_name']}")

            logging.info(f"Total ingestion completed: {processed_docs} documents, {total_chunks} chunks stored")
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
            # Get collection info from Qdrant - this may raise validation errors with Qdrant Cloud
            collection_info = self.vector_db.client.get_collection(
                collection_name=self.vector_db.collection_name
            )

            # Safely extract vectors_count with fallback to 0
            # Handle potential attribute access issues with getattr
            vectors_count = getattr(collection_info, 'points_count', 0)

            # If points_count is not available or None, default to 0
            if vectors_count is None:
                vectors_count = 0

            return {
                "status": "ready",
                "collection_name": self.vector_db.collection_name,
                "vectors_count": vectors_count,
                "message": "Vector database is ready for queries"
            }
        except Exception as e:
            logging.warning(f"Error getting ingestion status (this is expected with Qdrant Cloud): {e}")
            # Return safe default values when collection info fails
            # This can happen with Qdrant Cloud due to response schema differences
            return {
                "status": "ready",  # Don't mark as error since it might be just a schema issue
                "vectors_count": 0,  # Default to 0 if we can't get the count
                "message": f"Vector database connection OK, count unavailable: {str(e)}"
            }

# Global instance
document_ingestion_service = DocumentIngestionService()