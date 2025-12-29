import React from 'react';
import { UserQuery, ChatResponse, SourceReference } from '../types/chat';

interface MessageDisplayProps {
  query: string;
  response: ChatResponse;
}

const MessageDisplay: React.FC<MessageDisplayProps> = ({ query, response }) => {
  const { answer, sources, confidenceScore } = response;

  // Function to verify source authenticity (simplified for this example)
  const verifySource = (source: SourceReference): boolean => {
    // In a real implementation, this would check if the source URL is valid,
    // if the page number is within range, etc.
    // For now, we'll just check if it has a valid URL format
    try {
      new URL(source.url);
      return true;
    } catch {
      return false;
    }
  };

  const renderSource = (source: SourceReference) => {
    const isVerified = verifySource(source);

    return (
      <div key={`${source.url}-${source.page}`} className="source-reference">
        <a href={source.url} target="_blank" rel="noopener noreferrer">
          {source.title}
        </a>
        {isVerified && <span className="verification-badge" title="Verified source">✓</span>}
        {!isVerified && <span className="verification-badge unverified" title="Unverified source">⚠</span>}
        {source.page && <span className="page-ref"> (Page {source.page})</span>}
        {source.section && <span className="section-ref">, Section {source.section}</span>}
        {source.textSnippet && (
          <div className="snippet-preview">
            <em>"{source.textSnippet}"</em>
          </div>
        )}
      </div>
    );
  };

  return (
    <div className="message-display">
      <div className="user-query">
        <strong>You:</strong> {query}
      </div>
      <div className="chat-response">
        <strong>Assistant:</strong>
        <div className="response-content">
          {answer}
        </div>

        {sources && sources.length > 0 && (
          <div className="sources-section">
            <h4>Sources:</h4>
            {sources.map(renderSource)}
          </div>
        )}

        {confidenceScore !== undefined && (
          <div className="confidence-score">
            <small>Confidence: {(confidenceScore * 100).toFixed(1)}%</small>
          </div>
        )}
      </div>
    </div>
  );
};

export default MessageDisplay;