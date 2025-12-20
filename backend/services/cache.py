import hashlib
import json
from typing import Optional, Dict, Any
from datetime import datetime, timedelta
import asyncio

class SimpleCache:
    """
    Simple in-memory cache for storing query results
    In production, you might want to use Redis or another caching solution
    """
    def __init__(self, default_ttl: int = 3600):  # 1 hour default TTL
        self.cache = {}
        self.default_ttl = default_ttl

    def _generate_key(self, query: str, context: Dict[str, Any] = None) -> str:
        """
        Generate a cache key based on query and context
        """
        key_data = {
            "query": query,
            "context": context or {}
        }
        key_str = json.dumps(key_data, sort_keys=True)
        return hashlib.md5(key_str.encode()).hexdigest()

    def get(self, query: str, context: Dict[str, Any] = None) -> Optional[Dict[str, Any]]:
        """
        Get cached result if it exists and hasn't expired
        """
        key = self._generate_key(query, context)

        if key in self.cache:
            cached_item = self.cache[key]
            if datetime.now() < cached_item['expires_at']:
                return cached_item['data']
            else:
                # Remove expired item
                del self.cache[key]

        return None

    def set(self, query: str, result: Dict[str, Any], context: Dict[str, Any] = None, ttl: int = None) -> None:
        """
        Store result in cache with TTL
        """
        key = self._generate_key(query, context)
        ttl = ttl or self.default_ttl

        self.cache[key] = {
            'data': result,
            'expires_at': datetime.now() + timedelta(seconds=ttl)
        }

    def delete(self, query: str, context: Dict[str, Any] = None) -> bool:
        """
        Remove item from cache
        """
        key = self._generate_key(query, context)
        if key in self.cache:
            del self.cache[key]
            return True
        return False

    def clear(self) -> None:
        """
        Clear all cache
        """
        self.cache.clear()

    def size(self) -> int:
        """
        Get current cache size
        """
        return len(self.cache)

# Global cache instance
query_cache = SimpleCache()