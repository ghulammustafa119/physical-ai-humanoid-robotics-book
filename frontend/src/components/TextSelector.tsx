import React, { useState, useEffect, useRef } from 'react';
import useTextSelection from '../hooks/useTextSelection';

interface TextSelectorProps {
  onTextSelected: (text: string, selectionInfo: any) => void;
  children: React.ReactNode;
}

const TextSelector: React.FC<TextSelectorProps> = ({ onTextSelected, children }) => {
  const { selectedText, selectionInfo, startSelection, endSelection, clearSelection } = useTextSelection();
  const [isSelectionMode, setIsSelectionMode] = useState(false);
  const containerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (selectedText && selectionInfo) {
      onTextSelected(selectedText, selectionInfo);
    }
  }, [selectedText, selectionInfo, onTextSelected]);

  const toggleSelectionMode = () => {
    if (isSelectionMode) {
      endSelection();
      setIsSelectionMode(false);
    } else {
      startSelection();
      setIsSelectionMode(true);
    }
  };

  const handleClearSelection = () => {
    clearSelection();
    setIsSelectionMode(false);
  };

  return (
    <div ref={containerRef} className="text-selector-container">
      <div className="text-selector-toolbar">
        <button
          className={`selection-mode-btn ${isSelectionMode ? 'active' : ''}`}
          onClick={toggleSelectionMode}
        >
          {isSelectionMode ? 'Exit Selection Mode' : 'Select Text'}
        </button>
        {selectedText && (
          <button
            className="clear-selection-btn"
            onClick={handleClearSelection}
          >
            Clear Selection
          </button>
        )}
        {isSelectionMode && (
          <div className="selection-instructions">
            Select text by clicking and dragging over the content
          </div>
        )}
      </div>

      <div className="selectable-content">
        {children}
      </div>

      {selectedText && (
        <div className="selected-text-preview">
          <h4>Selected Text:</h4>
          <p>"{selectedText}"</p>
          <div className="selection-stats">
            Length: {selectedText.length} characters
          </div>
        </div>
      )}
    </div>
  );
};

export default TextSelector;