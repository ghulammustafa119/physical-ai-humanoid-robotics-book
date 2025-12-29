import { useState, useEffect, useCallback } from 'react';
import { ChatSession, UserQuery, ChatResponse, ChatRequest } from '../types/chat';
import chatSessionManager from '../services/chat-session';
import apiClient from '../services/api-client';

interface UseChatSessionReturn {
  session: ChatSession | null;
  history: Array<{ query: UserQuery; response: ChatResponse }>;
  isLoading: boolean;
  error: string | null;
  createSession: (selectedTextId?: string) => void;
  sendMessage: (request: ChatRequest) => Promise<ChatResponse>;
  clearSession: () => void;
  updateSession: (selectedTextId?: string) => void;
}

const useChatSession = (): UseChatSessionReturn => {
  const [session, setSession] = useState<ChatSession | null>(null);
  const [history, setHistory] = useState<Array<{ query: UserQuery; response: ChatResponse }>>([]);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  // Initialize session when component mounts
  useEffect(() => {
    const currentSession = chatSessionManager.getSession();
    setSession(currentSession);
    setHistory(chatSessionManager.getHistory());
  }, []);

  const createSession = useCallback((selectedTextId?: string) => {
    const newSession = chatSessionManager.createSession(selectedTextId);
    setSession(newSession);
    setHistory([]);
    setError(null);
  }, []);

  const updateSession = useCallback((selectedTextId?: string) => {
    const updatedSession = chatSessionManager.updateSession(selectedTextId);
    if (updatedSession) {
      setSession(updatedSession);
    }
  }, []);

  const sendMessage = useCallback(async (request: ChatRequest): Promise<ChatResponse> => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await apiClient.sendChatQuery(request);

      // Create mock query object (in a real implementation, the backend would return this)
      const query: UserQuery = {
        id: `query_${Date.now()}`,
        sessionId: request.sessionId || session?.id || 'unknown',
        queryText: request.query,
        selectedTextId: undefined, // Would come from backend in real implementation
        timestamp: new Date().toISOString(),
        queryType: 'initial' // Would be determined by context in real implementation
      };

      // Add to history
      chatSessionManager.addMessageToHistory(query, response);
      setHistory(chatSessionManager.getHistory());

      return response;
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Unknown error occurred';
      setError(errorMessage);
      throw err;
    } finally {
      setIsLoading(false);
    }
  }, [session?.id]);

  const clearSession = useCallback(() => {
    chatSessionManager.clearSession();
    setSession(null);
    setHistory([]);
    setError(null);
  }, []);

  return {
    session,
    history,
    isLoading,
    error,
    createSession,
    sendMessage,
    clearSession,
    updateSession
  };
};

export default useChatSession;