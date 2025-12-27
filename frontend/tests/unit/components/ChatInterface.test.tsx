import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import ChatInterface from '../../../src/components/ChatInterface';

// Mock the useChatSession hook
jest.mock('../../../src/hooks/useChatSession', () => ({
  __esModule: true,
  default: () => ({
    session: { id: 'test-session-id', active: true },
    history: [],
    isLoading: false,
    error: null,
    createSession: jest.fn(),
    sendMessage: jest.fn(),
    clearSession: jest.fn(),
    updateSession: jest.fn(),
  })
}));

// Mock the MessageDisplay component
jest.mock('../../../src/components/MessageDisplay', () => ({
  __esModule: true,
  default: ({ query, response }: { query: string; response: any }) => (
    <div data-testid="message-display">
      <div>Query: {query}</div>
      <div>Response: {response.response_text || response.answer}</div>
    </div>
  )
}));

describe('ChatInterface Component', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('renders correctly with default props', () => {
    render(<ChatInterface />);

    expect(screen.getByText('Book Assistant')).toBeInTheDocument();
    expect(screen.getByPlaceholderText('Ask a question about the book...')).toBeInTheDocument();
    expect(screen.getByText('Ask me questions about the book!')).toBeInTheDocument();
  });

  it('renders with selected text context', () => {
    render(<ChatInterface selectedText="This is selected text for context" />);

    expect(screen.getByText('Book Assistant')).toBeInTheDocument();
    expect(screen.getByText(/Context: Using selected text/)).toBeInTheDocument();
  });

  it('allows user to type and submit a message', async () => {
    const mockSendMessage = jest.fn().mockResolvedValue({
      answer: 'Test response',
      sources: [],
      queryId: 'test-query-id',
      confidenceScore: 0.85,
      timestamp: new Date().toISOString()
    });

    // Update mock to include the sendMessage function
    jest.mock('../../../src/hooks/useChatSession', () => ({
      __esModule: true,
      default: () => ({
        session: { id: 'test-session-id', active: true },
        history: [],
        isLoading: false,
        error: null,
        createSession: jest.fn(),
        sendMessage: mockSendMessage,
        clearSession: jest.fn(),
        updateSession: jest.fn(),
      })
    }));

    render(<ChatInterface />);

    const input = screen.getByPlaceholderText('Ask a question about the book...');
    const submitButton = screen.getByText('Send');

    fireEvent.change(input, { target: { value: 'Test question' } });
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(mockSendMessage).toHaveBeenCalled();
    });
  });

  it('disables submit button when input is empty', () => {
    render(<ChatInterface />);

    const submitButton = screen.getByText('Send');
    expect(submitButton).toBeDisabled();

    const input = screen.getByPlaceholderText('Ask a question about the book...');
    fireEvent.change(input, { target: { value: 'Test question' } });

    expect(submitButton).not.toBeDisabled();
  });

  it('shows loading state when submitting', () => {
    // Update mock to simulate loading
    jest.mock('../../../src/hooks/useChatSession', () => ({
      __esModule: true,
      default: () => ({
        session: { id: 'test-session-id', active: true },
        history: [],
        isLoading: true, // Simulate loading
        error: null,
        createSession: jest.fn(),
        sendMessage: jest.fn(),
        clearSession: jest.fn(),
        updateSession: jest.fn(),
      })
    }));

    render(<ChatInterface />);

    expect(screen.getByText('Thinking...')).toBeInTheDocument();
  });

  it('displays error message when error occurs', () => {
    // Update mock to include error
    jest.mock('../../../src/hooks/useChatSession', () => ({
      __esModule: true,
      default: () => ({
        session: { id: 'test-session-id', active: true },
        history: [],
        isLoading: false,
        error: 'Test error message',
        createSession: jest.fn(),
        sendMessage: jest.fn(),
        clearSession: jest.fn(),
        updateSession: jest.fn(),
      })
    }));

    render(<ChatInterface />);

    expect(screen.getByText(/Error: Test error message/)).toBeInTheDocument();
  });

  it('displays chat history when messages exist', () => {
    // Update mock to include history
    const mockHistory = [
      {
        query: { queryText: 'Test query', id: 'q1', sessionId: 's1', timestamp: '2023-01-01', queryType: 'initial' },
        response: {
          answer: 'Test response',
          sources: [],
          id: 'r1',
          queryId: 'q1',
          confidenceScore: 0.85,
          timestamp: '2023-01-01'
        }
      }
    ];

    jest.mock('../../../src/hooks/useChatSession', () => ({
      __esModule: true,
      default: () => ({
        session: { id: 'test-session-id', active: true },
        history: mockHistory,
        isLoading: false,
        error: null,
        createSession: jest.fn(),
        sendMessage: jest.fn(),
        clearSession: jest.fn(),
        updateSession: jest.fn(),
      })
    }));

    render(<ChatInterface />);

    expect(screen.getByText('Test query')).toBeInTheDocument();
    expect(screen.getByText('Test response')).toBeInTheDocument();
  });
});