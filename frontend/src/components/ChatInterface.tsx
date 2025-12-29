import React, { useState, useRef, useEffect } from 'react';
import useChatSession from '../hooks/useChatSession';
import MessageDisplay from './MessageDisplay';
import { ChatRequest } from '../types/chat';

interface ChatInterfaceProps {
  selectedText?: string;
  bookSection?: string;
}

const ChatInterface: React.FC<ChatInterfaceProps> = ({ selectedText, bookSection }) => {
  const { session, history, isLoading, error, sendMessage } = useChatSession();
  const [inputValue, setInputValue] = useState<string>('');
  const [isProcessing, setIsProcessing] = useState<boolean>(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [history]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading || isProcessing) {
      return;
    }

    setIsProcessing(true);

    try {
      const chatRequest: ChatRequest = {
        query: inputValue,
        selectedText: selectedText,
        sessionId: session?.id,
        bookSection: bookSection
      };

      await sendMessage(chatRequest);
      setInputValue('');
    } catch (err) {
      console.error('Error sending message:', err);
    } finally {
      setIsProcessing(false);
    }
  };

  return (
    <div className="chat-interface">
      <div className="chat-header">
        <h3>Book Assistant</h3>
        {selectedText && (
          <div className="active-selection">
            <small>Context: Using selected text ({selectedText.substring(0, 50)}...)</small>
          </div>
        )}
      </div>

      <div className="chat-messages">
        {history.length === 0 ? (
          <div className="welcome-message">
            <p>Ask me questions about the book content!</p>
            {selectedText && (
              <p><em>Selected text is being used as context for your questions.</em></p>
            )}
          </div>
        ) : (
          history.map((item, index) => (
            <MessageDisplay
              key={index}
              query={item.query.queryText}
              response={item.response}
            />
          ))
        )}
        {isLoading && (
          <div className="loading-response">
            <p>Thinking...</p>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <form onSubmit={handleSubmit} className="chat-input-form">
        <div className="input-container">
          <input
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            placeholder="Ask a question about the book..."
            disabled={isLoading || isProcessing}
          />
          <button
            type="submit"
            disabled={!inputValue.trim() || isLoading || isProcessing}
          >
            Send
          </button>
        </div>
        {error && (
          <div className="error-message">
            <p>Error: {error}</p>
          </div>
        )}
      </form>
    </div>
  );
};

export default ChatInterface;