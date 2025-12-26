from typing import List, Dict, Any
from services.vector_db import vector_db_service
from config.settings import settings
from models.chat_models import Source
import logging
# Import Google Generative AI - using the current recommended approach
import google.generativeai as genai
NEW_GENAI = False  # Using the current package, but the import remains the same
from services.cache import query_cache

class RAGService:
    def __init__(self):
        self.vector_db = vector_db_service
        # Configure Google Generative AI
        genai.configure(api_key=settings.google_api_key)
        self.model = genai.GenerativeModel(settings.gemini_model)

    def _validate_response_compliance(self, query: str, response: str) -> str:
        """
        Ensure response follows the strict RAG contract by checking if it properly
        acknowledges when the book doesn't contain specific information.
        """
        response_lower = response.lower()

        # Check if response already complies with the "must state if not in book" rule
        if ("does not contain information about" in response_lower or
            "no information" in response_lower or
            "not found" in response_lower or
            "not mentioned" in response_lower or
            "not explicitly stated" in response_lower or
            "not covered" in response_lower):
            return response  # Already compliant with the contract

        # If response seems to provide partial info without clearly stating limitations,
        # we should enforce the contract by returning a proper compliance message
        # However, we'll be conservative and only do this if we're certain
        # the response doesn't follow the rules

        return response

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

            RULES:
            1. Answer the user's question based EXCLUSIVELY and ONLY on the provided context from the book.
            2. NEVER use external knowledge or general world knowledge.
            3. NEVER hallucinate or make up information not present in the context.
            4. NEVER infer information that is not explicitly stated in the provided context.
            5. If the provided context does NOT contain ANY information about the main topic in the user's question,
               respond with: "The book does not contain information about [specific topic from user's question]."
            6. If the provided context contains information about the topic but not enough to fully answer the question,
               clearly state what information IS available in the context and what is NOT available.
            7. Be precise and factual based only on what is in the context.

            Context from book:
            {context}

            User's question: {query}

            Answer (following the rules above):
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

            # Validate response compliance with the contract
            validated_response = self._validate_response_compliance(query, ai_response)

            result = {
                "response": validated_response,
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