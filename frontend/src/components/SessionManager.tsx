import React, { useEffect } from 'react';
import useChatSession from '../hooks/useChatSession';

interface SessionManagerProps {
  onSessionChange?: (sessionActive: boolean) => void;
}

const SessionManager: React.FC<SessionManagerProps> = ({ onSessionChange }) => {
  const { session, createSession, clearSession, isLoading } = useChatSession();

  useEffect(() => {
    if (onSessionChange) {
      onSessionChange(!!session);
    }
  }, [session, onSessionChange]);

  const handleNewSession = () => {
    createSession();
  };

  const handleClearSession = () => {
    clearSession();
  };

  return (
    <div className="session-manager">
      <div className="session-status">
        {session ? (
          <div className="active-session">
            <span className="status-indicator active"></span>
            <span>Active Session: {session.id.substring(0, 8)}...</span>
          </div>
        ) : (
          <div className="no-session">
            <span className="status-indicator inactive"></span>
            <span>No Active Session</span>
          </div>
        )}
      </div>

      <div className="session-controls">
        {!session ? (
          <button
            onClick={handleNewSession}
            disabled={isLoading}
            className="new-session-btn"
          >
            Start New Session
          </button>
        ) : (
          <button
            onClick={handleClearSession}
            disabled={isLoading}
            className="clear-session-btn"
          >
            Clear Session
          </button>
        )}
      </div>
    </div>
  );
};

export default SessionManager;