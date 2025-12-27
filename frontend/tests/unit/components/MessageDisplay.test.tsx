import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import MessageDisplay from '../../../src/components/MessageDisplay';

describe('MessageDisplay Component', () => {
  const mockQuery = 'What is the main concept?';
  const mockResponse = {
    answer: 'The main concept is RAG (Retrieval Augmented Generation).',
    sources: [
      {
        title: 'Chapter 1: Introduction to RAG',
        url: '/book/chapter-1',
        page: 5,
        section: '1.1',
        textSnippet: 'RAG combines retrieval and generation...'
      }
    ],
    queryId: 'test-query-id',
    sessionId: 'test-session-id',
    confidenceScore: 0.85,
    timestamp: new Date().toISOString()
  };

  it('renders query and response correctly', () => {
    render(
      <MessageDisplay
        query={mockQuery}
        response={mockResponse}
      />
    );

    expect(screen.getByText(/You:/)).toBeInTheDocument();
    expect(screen.getByText(mockQuery)).toBeInTheDocument();
    expect(screen.getByText(/Assistant:/)).toBeInTheDocument();
    expect(screen.getByText(mockResponse.answer)).toBeInTheDocument();
  });

  it('displays sources when they exist', () => {
    render(
      <MessageDisplay
        query={mockQuery}
        response={mockResponse}
      />
    );

    expect(screen.getByText('Sources:')).toBeInTheDocument();
    expect(screen.getByText(mockResponse.sources[0].title)).toBeInTheDocument();
    expect(screen.getByText(/Page 5/)).toBeInTheDocument();
    expect(screen.getByText(/Section 1.1/)).toBeInTheDocument();
    expect(screen.getByText(mockResponse.sources[0].textSnippet)).toBeInTheDocument();
  });

  it('does not display sources section when no sources exist', () => {
    const responseWithoutSources = {
      ...mockResponse,
      sources: []
    };

    render(
      <MessageDisplay
        query={mockQuery}
        response={responseWithoutSources}
      />
    );

    expect(screen.queryByText('Sources:')).not.toBeInTheDocument();
  });

  it('displays confidence score when available', () => {
    render(
      <MessageDisplay
        query={mockQuery}
        response={mockResponse}
      />
    );

    expect(screen.getByText(/Confidence: 85.0%/)).toBeInTheDocument();
  });

  it('does not display confidence score when not available', () => {
    const responseWithoutConfidence = {
      ...mockResponse,
      confidenceScore: undefined
    };

    render(
      <MessageDisplay
        query={mockQuery}
        response={responseWithoutConfidence}
      />
    );

    expect(screen.queryByText(/Confidence:/)).not.toBeInTheDocument();
  });

  it('formats source link correctly', () => {
    render(
      <MessageDisplay
        query={mockQuery}
        response={mockResponse}
      />
    );

    const link = screen.getByRole('link');
    expect(link).toHaveAttribute('href', mockResponse.sources[0].url);
  });

  it('handles response with multiple sources', () => {
    const responseWithMultipleSources = {
      ...mockResponse,
      sources: [
        {
          title: 'Chapter 1: Introduction to RAG',
          url: '/book/chapter-1',
          page: 5,
          section: '1.1',
          textSnippet: 'RAG combines retrieval and generation...'
        },
        {
          title: 'Chapter 2: Advanced RAG Techniques',
          url: '/book/chapter-2',
          page: 15,
          section: '2.3',
          textSnippet: 'Advanced techniques for improving RAG...'
        }
      ]
    };

    render(
      <MessageDisplay
        query={mockQuery}
        response={responseWithMultipleSources}
      />
    );

    expect(screen.getByText('Sources:')).toBeInTheDocument();
    expect(screen.getAllByText('source-reference')).toHaveLength(2); // Each source renders as a div with this class
  });

  it('handles response with partial source information', () => {
    const responseWithPartialSource = {
      ...mockResponse,
      sources: [
        {
          title: 'Chapter 1: Introduction to RAG',
          url: '/book/chapter-1',
          page: 5,
          // Missing section and textSnippet
        }
      ]
    };

    render(
      <MessageDisplay
        query={mockQuery}
        response={responseWithPartialSource}
      />
    );

    expect(screen.getByText('Chapter 1: Introduction to RAG')).toBeInTheDocument();
    expect(screen.getByText(/Page 5/)).toBeInTheDocument();
    // Should not show section or snippet if they're undefined
  });
});