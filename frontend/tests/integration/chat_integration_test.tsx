import React from 'react';
import { render, screen, fireEvent, waitFor, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import ChatInterface from '../../src/components/ChatInterface';
import TextSelector from '../../src/components/TextSelector';

// Mock the API client
jest.mock('../../src/services/api-client', () => ({
  __esModule: true,
  default: {
    sendChatQuery: jest.fn().mockResolvedValue({
      answer: 'This is a test response based on the selected text.',
      sources: [
        {
          title: 'Chapter 1: Introduction',
          url: '/book/chapter-1',
          page: 5,
          section: '1.1',
          textSnippet: 'This is a sample text snippet...'
        }
      ],
      queryId: 'test-query-id',
      sessionId: 'test-session-id',
      confidenceScore: 0.85,
      timestamp: new Date().toISOString()
    }),
    createTextSelection: jest.fn().mockResolvedValue({
      id: 'test-selection-id',
      content: 'This is the selected text',
      createdAt: new Date().toISOString()
    })
  }
}));

// Mock the hooks
jest.mock('../../src/hooks/useChatSession', () => ({
  __esModule: true,
  default: () => ({
    session: { id: 'test-session-id', active: true },
    history: [],
    isLoading: false,
    error: null,
    createSession: jest.fn(),
    sendMessage: jest.fn().mockResolvedValue({
      answer: 'This is a test response based on the selected text.',
      sources: [
        {
          title: 'Chapter 1: Introduction',
          url: '/book/chapter-1',
          page: 5,
          section: '1.1',
          textSnippet: 'This is a sample text snippet...'
        }
      ],
      queryId: 'test-query-id',
      sessionId: 'test-session-id',
      confidenceScore: 0.85,
      timestamp: new Date().toISOString()
    }),
    clearSession: jest.fn(),
    updateSession: jest.fn(),
  })
}));

jest.mock('../../src/hooks/useTextSelection', () => ({
  __esModule: true,
  default: () => ({
    selectedText: 'This is the selected text for testing',
    selectionInfo: { start: 10, end: 30, element: null },
    isSelecting: false,
    startSelection: jest.fn(),
    endSelection: jest.fn(),
    clearSelection: jest.fn(),
    getSelectedText: jest.fn(),
    getSelectionInfo: jest.fn(),
  })
}));

describe('Chat Integration Tests', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('completes full chat flow with selected text', async () => {
    const selectedText = 'This is the selected text for testing purposes.';

    render(
      <div>
        <TextSelector onTextSelected={jest.fn()}>
          <div>Selectable content area</div>
        </TextSelector>
        <ChatInterface selectedText={selectedText} />
      </div>
    );

    // Verify that the selected text context is shown
    expect(screen.getByText(/Context: Using selected text/)).toBeInTheDocument();

    // Find and interact with the chat interface
    const input = screen.getByPlaceholderText('Ask a question about the book...');
    const submitButton = screen.getByText('Send');

    // Enter a query related to the selected text
    fireEvent.change(input, { target: { value: 'What does this text say?' } });
    fireEvent.click(submitButton);

    // Wait for the response to be displayed
    await waitFor(() => {
      expect(screen.getByText(/This is a test response based on the selected text/)).toBeInTheDocument();
    });

    // Verify that sources are displayed
    expect(screen.getByText('Sources:')).toBeInTheDocument();
    expect(screen.getByText('Chapter 1: Introduction')).toBeInTheDocument();
  });

  it('handles queries without selected text', async () => {
    render(<ChatInterface />);

    // Verify that the component renders without selected text context
    expect(screen.getByText('Book Assistant')).toBeInTheDocument();
    expect(screen.queryByText(/Context: Using selected text/)).not.toBeInTheDocument();

    const input = screen.getByPlaceholderText('Ask a question about the book...');
    const submitButton = screen.getByText('Send');

    fireEvent.change(input, { target: { value: 'What is the main topic?' } });
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(screen.getByText(/This is a test response based on the selected text/)).toBeInTheDocument();
    });
  });

  it('displays error when API call fails', async () => {
    // Update mock to simulate API failure
    const mockSendMessage = jest.fn().mockRejectedValue(new Error('API Error'));

    jest.mock('../../src/hooks/useChatSession', () => ({
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

    fireEvent.change(input, { target: { value: 'Test query' } });
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(screen.getByText(/Error:/)).toBeInTheDocument();
    });
  });

  it('shows loading state during query processing', async () => {
    // Update mock to simulate loading state
    let resolvePromise: (value: any) => void;
    const promise = new Promise((resolve) => {
      resolvePromise = resolve;
    });

    const mockSendMessage = jest.fn().mockReturnValue(promise);

    jest.mock('../../src/hooks/useChatSession', () => ({
      __esModule: true,
      default: () => ({
        session: { id: 'test-session-id', active: true },
        history: [],
        isLoading: true,
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

    fireEvent.change(input, { target: { value: 'Test query' } });
    fireEvent.click(submitButton);

    // Check that loading state is shown
    expect(screen.getByText('Thinking...')).toBeInTheDocument();

    // Resolve the promise to complete the test
    act(() => {
      resolvePromise!({
        answer: 'This is a test response based on the selected text.',
        sources: [
          {
            title: 'Chapter 1: Introduction',
            url: '/book/chapter-1',
            page: 5,
            section: '1.1',
            textSnippet: 'This is a sample text snippet...'
          }
        ],
        queryId: 'test-query-id',
        sessionId: 'test-session-id',
        confidenceScore: 0.85,
        timestamp: new Date().toISOString()
      });
    });
  });

  it('displays chat history after multiple interactions', async () => {
    const initialHistory = [
      {
        query: {
          queryText: 'First question',
          id: 'q1',
          sessionId: 's1',
          timestamp: '2023-01-01',
          queryType: 'initial'
        },
        response: {
          answer: 'First response',
          sources: [],
          id: 'r1',
          queryId: 'q1',
          confidenceScore: 0.85,
          timestamp: '2023-01-01'
        }
      }
    ];

    // Update mock to include history
    jest.mock('../../src/hooks/useChatSession', () => ({
      __esModule: true,
      default: () => ({
        session: { id: 'test-session-id', active: true },
        history: initialHistory,
        isLoading: false,
        error: null,
        createSession: jest.fn(),
        sendMessage: jest.fn().mockResolvedValue({
          answer: 'This is a test response based on the selected text.',
          sources: [],
          queryId: 'test-query-id',
          sessionId: 'test-session-id',
          confidenceScore: 0.85,
          timestamp: new Date().toISOString()
        }),
        clearSession: jest.fn(),
        updateSession: jest.fn(),
      })
    }));

    render(<ChatInterface />);

    // Verify that the initial history is displayed
    expect(screen.getByText('First question')).toBeInTheDocument();
    expect(screen.getByText('First response')).toBeInTheDocument();

    // Add a new message
    const input = screen.getByPlaceholderText('Ask a question about the book...');
    const submitButton = screen.getByText('Send');

    fireEvent.change(input, { target: { value: 'Second question' } });
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(screen.getByText('Second question')).toBeInTheDocument();
    });
  });

  it('maintains session context across interactions', () => {
    render(<ChatInterface />);

    // Verify that session information is displayed
    expect(screen.getByText('Book Assistant')).toBeInTheDocument();

    // Check that the session ID is being used (would be visible in a real implementation)
    // For now, we're just verifying the component renders correctly with session context
  });
});