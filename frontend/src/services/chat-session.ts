import { ChatSession, UserQuery, ChatResponse } from '../types/chat';

export interface ChatHistoryItem {
  query: UserQuery;
  response: ChatResponse;
}

export class ChatSessionManager {
  private currentSession: ChatSession | null = null;
  private history: ChatHistoryItem[] = [];

  constructor() {
    this.initializeSession();
  }

  private initializeSession(): void {
    const sessionId = localStorage.getItem('currentChatSessionId');
    if (sessionId) {
      this.currentSession = {
        id: sessionId,
        userId: localStorage.getItem('userId') || undefined,
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString(),
        active: true,
      };
    } else {
      this.createSession();
    }
  }

  createSession(selectedTextId?: string): ChatSession {
    const sessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    this.currentSession = {
      id: sessionId,
      userId: localStorage.getItem('userId') || undefined,
      createdAt: new Date().toISOString(),
      updatedAt: new Date().toISOString(),
      active: true,
      selectedTextId: selectedTextId,
    };

    localStorage.setItem('currentChatSessionId', sessionId);
    this.history = [];

    return this.currentSession;
  }

  getSession(): ChatSession | null {
    return this.currentSession;
  }

  updateSession(selectedTextId?: string): ChatSession | null {
    if (!this.currentSession) {
      return null;
    }

    this.currentSession = {
      ...this.currentSession,
      selectedTextId,
      updatedAt: new Date().toISOString(),
    };

    return this.currentSession;
  }

  addMessageToHistory(query: UserQuery, response: ChatResponse): void {
    this.history.push({ query, response });
  }

  getHistory(): ChatHistoryItem[] {
    return [...this.history]; // Return a copy to prevent direct mutation
  }

  clearHistory(): void {
    this.history = [];
  }

  clearSession(): void {
    this.currentSession = null;
    this.history = [];
    localStorage.removeItem('currentChatSessionId');
  }

  isActive(): boolean {
    return this.currentSession?.active === true;
  }

  // Get recent context for maintaining conversation flow
  getRecentContext(limit: number = 3): ChatHistoryItem[] {
    return this.history.slice(-limit);
  }
}

// Export a singleton instance
export default new ChatSessionManager();