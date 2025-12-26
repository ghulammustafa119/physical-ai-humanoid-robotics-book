from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from config.settings import settings
import logging
import uuid

class VectorDBService:
    def __init__(self):
        # Initialize Qdrant client
        if settings.qdrant_api_key:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key
            )
        else:
            self.client = QdrantClient(url=settings.qdrant_url)


        self.collection_name = settings.qdrant_collection_name
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """
        Ensure the collection exists in Qdrant
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with appropriate configuration
                # Using 768 dimensions for Google's text-embedding-004 model
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=768,  # Size for Google text-embedding-004 embeddings
                        distance=models.Distance.COSINE
                    )
                )
                logging.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logging.info(f"Qdrant collection exists: {self.collection_name}")
        except Exception as e:
            logging.error(f"Error ensuring collection exists: {e}")
            raise e

    def store_document(self, content: str, metadata: Dict[str, Any], doc_id: Optional[str] = None) -> str:
        """
        Store a document in the vector database
        """
        try:
            if not doc_id:
                doc_id = str(uuid.uuid4())

            # Generate embedding for the content using Google's service
            # Using Google's text embedding service
            # In production, you'd use Google's embedding API
            embedding = self._get_google_embedding(content)

            # Store in Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=doc_id,
                        vector=embedding,
                        payload={
                            "content": content,
                            "metadata": metadata
                        }
                    )
                ]
            )

            return doc_id
        except Exception as e:
            logging.error(f"Error storing document: {e}")
            raise e

    def _get_google_embedding(self, text: str):
        """
        Get embedding from Google's service
        """
        try:
            # Use Google's text embedding service
            import google.generativeai as genai

            # Configure the API key
            genai.configure(api_key=settings.google_api_key)

            # Log the model name being used for debugging
            model_name = settings.embedding_model
            logging.debug(f"Using embedding model: {model_name}")

            # Get embeddings using the embedding API
            result = genai.embed_content(
                model=model_name,  # Google's text embedding model
                content=[text],
                task_type="RETRIEVAL_DOCUMENT"
            )

            # Return the embedding vector
            return result['embedding'][0]
        except Exception as e:
            logging.warning(f"Error getting Google embedding with model '{settings.embedding_model}': {e}, using fallback method")
            # Fallback to hash-based approach if API fails
            import hashlib
            import numpy as np

            # Create a deterministic vector based on the text
            # that has 768 dimensions (matching Google's text-embedding-004)
            text_hash = hashlib.md5(text.encode()).hexdigest()
            # Convert hex to numbers and create a 768-dimensional vector
            embedding = []
            for i in range(0, len(text_hash), 2):
                if len(embedding) < 768:
                    hex_pair = text_hash[i:i+2]
                    val = int(hex_pair, 16) / 255.0  # Normalize to 0-1
                    embedding.append(val)

            # Pad or truncate to exactly 768 dimensions
            while len(embedding) < 768:
                embedding.append(0.0)
            embedding = embedding[:768]

            return embedding

    def search_documents(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for documents similar to the query
        """
        try:
            # Generate embedding for the query using Google's service
            query_embedding = self._get_google_embedding(query)

            # Search in Qdrant using the correct method for Qdrant client 1.7.0
            from qdrant_client.http import models
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit
            )

            results = []
            for result in search_results:
                results.append({
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "metadata": result.payload.get("metadata", {}),
                    "score": result.score
                })

            return results
        except Exception as e:
            logging.error(f"Error searching documents: {e}")
            raise e

    def delete_document(self, doc_id: str) -> bool:
        """
        Delete a document by ID
        """
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[doc_id]
                )
            )
            return True
        except Exception as e:
            logging.error(f"Error deleting document: {e}")
            return False

# Global instance
vector_db_service = VectorDBService()