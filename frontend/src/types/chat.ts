// TypeScript types for chat entities

export interface TextSelection {
  id: string;
  content: string;
  startPosition: number;
  endPosition: number;
  bookSection: string;
  createdAt: string;
  userId?: string;
}

export interface ChatSession {
  id: string;
  userId?: string;
  createdAt: string;
  updatedAt: string;
  active: boolean;
  selectedTextId?: string;
}

export interface UserQuery {
  id: string;
  sessionId: string;
  queryText: string;
  selectedTextId?: string;
  timestamp: string;
  queryType: 'initial' | 'followup' | 'context_change';
}

export interface SourceReference {
  title: string;
  url: string;
  page: number;
  section?: string;
  textSnippet?: string;
}

export interface ChatResponse {
  id: string;
  queryId: string;
  answer: string;
  sources: SourceReference[];
  queryId: string;
  sessionId: string;
  confidenceScore: number;
  timestamp: string;
  tokenUsage?: {
    inputTokens: number;
    outputTokens: number;
    totalTokens: number;
  };
}

export interface ChatRequest {
  query: string;
  selectedText?: string;
  sessionId?: string;
  bookSection?: string;
}