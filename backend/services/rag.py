from typing import List, Dict, Any
from services.vector_db import vector_db_service
from config.settings import settings
from api.chat import Source
import logging
import google.generativeai as genai
from services.cache import query_cache

class RAGService:
    def __init__(self):
        self.vector_db = vector_db_service
        # Configure Google Generative AI
        genai.configure(api_key=settings.google_api_key)
        self.model = genai.GenerativeModel(settings.gemini_model)

    def process_query(self, query: str) -> Dict[str, Any]:
        """
        Process a user query using RAG methodology with caching
        """
        try:
            # Check cache first
            cached_result = query_cache.get(query, {})
            if cached_result:
                logging.info(f"Cache hit for query: {query[:50]}...")
                return cached_result

            # Step 1: Search for relevant documents
            search_results = self.vector_db.search_documents(query, limit=5)

            if not search_results:
                result = {
                    "response": "I couldn't find relevant information in the book to answer your question.",
                    "sources": [],
                    "context": {"query": query, "retrieved_docs": 0}
                }

                # Cache this result too (negative results can be cached)
                query_cache.set(query, result, ttl=300)  # Cache for 5 minutes
                return result

            # Step 2: Prepare context from retrieved documents
            context_parts = []
            sources = []

            for result in search_results:
                content = result["content"]
                metadata = result["metadata"]

                context_parts.append(content)

                # Create source attribution
                sources.append(Source(
                    content=content[:200] + "..." if len(content) > 200 else content,
                    source=metadata.get("source", "unknown"),
                    page=metadata.get("page", None)
                ))

            # Combine context
            context = "\n\n".join(context_parts)

            # Ensure context doesn't exceed limits
            if len(context) > settings.max_context_length:
                context = context[:settings.max_context_length]

            # Step 3: Generate response using Google Gemini
            full_prompt = f"""
            You are an AI assistant for the Physical AI & Humanoid Robotics Book.
            Answer the user's question based ONLY on the provided context from the book.
            If the context doesn't contain enough information to answer the question, say so.

            Context from book:
            {context}

            User's question: {query}

            Answer:
            """

            # Create the generation configuration
            generation_config = {
                "temperature": 0.3,  # Lower temperature for more consistent, factual responses
                "max_output_tokens": settings.max_response_length,
            }

            # Generate content using Gemini
            response = self.model.generate_content(
                full_prompt,
                generation_config=generation_config
            )

            ai_response = response.text

            result = {
                "response": ai_response,
                "sources": sources,
                "context": {
                    "query": query,
                    "retrieved_docs": len(search_results),
                    "context_length": len(context)
                }
            }

            # Cache the result
            query_cache.set(query, result, ttl=3600)  # Cache for 1 hour by default

            return result

        except Exception as e:
            logging.error(f"Error processing query: {e}")
            return {
                "response": "Sorry, I encountered an error processing your request. Please try again.",
                "sources": [],
                "context": {"query": query, "error": str(e)}
            }

    def validate_query(self, query: str) -> Dict[str, Any]:
        """
        Validate that the query is appropriate for the RAG system
        """
        if not query or len(query.strip()) < 3:
            return {
                "valid": False,
                "message": "Query must be at least 3 characters long"
            }

        if len(query) > settings.max_query_length:
            return {
                "valid": False,
                "message": f"Query exceeds maximum length of {settings.max_query_length} characters"
            }

        return {
            "valid": True,
            "message": "Query is valid"
        }

# Global instance
rag_service = RAGService()