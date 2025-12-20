from typing import Dict, Any, List, Optional
from services.vector_db import vector_db_service
from services.document_ingestion import document_ingestion_service
import logging
import asyncio
from pathlib import Path

class IntegrationService:
    def __init__(self):
        self.vector_db = vector_db_service
        self.doc_ingestion = document_ingestion_service

    async def sync_content(self, source_path: str = "physical-ai-book/docs") -> Dict[str, Any]:
        """
        Synchronize book content with the RAG system
        This checks for changes in the source content and updates the vector database accordingly
        """
        try:
            # For now, we'll do a full reindex
            # In a more advanced implementation, we would track file modification times
            # and only update changed content
            result = await self.doc_ingestion.ingest_documents(source_path)

            return {
                "status": "success",
                "message": f"Content synchronized successfully: {result['message']}",
                "processed_documents": result['processed_documents'],
                "stored_chunks": result['stored_chunks']
            }
        except Exception as e:
            logging.error(f"Error during content synchronization: {e}")
            return {
                "status": "error",
                "message": f"Error during content synchronization: {str(e)}",
                "processed_documents": 0,
                "stored_chunks": 0
            }

    def get_content_mapping(self) -> Dict[str, Any]:
        """
        Get mapping of book content to vector database entries
        """
        try:
            # Get collection info
            collection_info = self.vector_db.client.get_collection(
                collection_name=self.vector_db.collection_name
            )

            return {
                "status": "success",
                "collection_name": self.vector_db.collection_name,
                "total_entries": collection_info.points_count,
                "message": "Content mapping retrieved successfully"
            }
        except Exception as e:
            logging.error(f"Error getting content mapping: {e}")
            return {
                "status": "error",
                "message": f"Error getting content mapping: {str(e)}"
            }

    async def add_feedback(self, query: str, response: str, feedback: str, rating: Optional[int] = None) -> Dict[str, Any]:
        """
        Add user feedback for a query-response pair
        """
        try:
            # In a real implementation, we would store feedback in a database
            # For now, we'll just log it
            feedback_data = {
                "query": query,
                "response": response,
                "feedback": feedback,
                "rating": rating,
                "timestamp": str(asyncio.get_event_loop().time())
            }

            logging.info(f"Feedback received: {feedback_data}")

            return {
                "status": "success",
                "message": "Feedback recorded successfully"
            }
        except Exception as e:
            logging.error(f"Error recording feedback: {e}")
            return {
                "status": "error",
                "message": f"Error recording feedback: {str(e)}"
            }

    def get_fallback_content(self, query: str) -> str:
        """
        Get fallback content when RAG system cannot answer the query
        """
        # In a real implementation, this might fetch from a static FAQ or documentation
        fallback_content = """
        I couldn't find specific information about your query in the book content.

        The Physical AI & Humanoid Robotics Book covers these main topics:
        1. ROS 2 (Robotic Nervous System)
        2. Gazebo & Unity (Digital Twin)
        3. NVIDIA Isaac (AI-Robot Brain)
        4. Vision-Language-Action (VLA)

        Please try rephrasing your question or check the relevant chapters in the book.
        You can also browse the book directly for more information.
        """

        return fallback_content

# Global instance
integration_service = IntegrationService()