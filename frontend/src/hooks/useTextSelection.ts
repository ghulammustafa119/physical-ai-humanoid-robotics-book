import { useState, useEffect, useCallback } from 'react';
import { TextSelection } from '../types/chat';

interface UseTextSelectionReturn {
  selectedText: string | null;
  selectionInfo: {
    start: number;
    end: number;
    element: HTMLElement | null;
  } | null;
  isSelecting: boolean;
  startSelection: () => void;
  endSelection: () => void;
  clearSelection: () => void;
  getSelectedText: () => string | null;
  getSelectionInfo: () => { start: number; end: number; element: HTMLElement | null } | null;
}

const useTextSelection = (): UseTextSelectionReturn => {
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [selectionInfo, setSelectionInfo] = useState<{
    start: number;
    end: number;
    element: HTMLElement | null;
  } | null>(null);
  const [isSelecting, setIsSelecting] = useState<boolean>(false);

  const getSelectedText = useCallback((): string | null => {
    const selection = window.getSelection();
    if (selection && selection.toString().trim() !== '') {
      return selection.toString();
    }
    return null;
  }, []);

  const getSelectionInfo = useCallback((): { start: number; end: number; element: HTMLElement | null } | null => {
    const selection = window.getSelection();
    if (selection && selection.toString().trim() !== '') {
      const range = selection.getRangeAt(0);
      return {
        start: range.startOffset,
        end: range.endOffset,
        element: range.startContainer.parentElement
      };
    }
    return null;
  }, []);

  const handleSelectionChange = useCallback(() => {
    const text = getSelectedText();
    if (text) {
      setSelectedText(text);
      setSelectionInfo(getSelectionInfo());
      setIsSelecting(true);
    } else {
      setSelectedText(null);
      setSelectionInfo(null);
      setIsSelecting(false);
    }
  }, [getSelectedText, getSelectionInfo]);

  const startSelection = useCallback(() => {
    document.addEventListener('mouseup', handleSelectionChange);
    document.addEventListener('keyup', handleSelectionChange);
  }, [handleSelectionChange]);

  const endSelection = useCallback(() => {
    document.removeEventListener('mouseup', handleSelectionChange);
    document.removeEventListener('keyup', handleSelectionChange);
    setIsSelecting(false);
  }, [handleSelectionChange]);

  const clearSelection = useCallback(() => {
    window.getSelection()?.removeAllRanges();
    setSelectedText(null);
    setSelectionInfo(null);
    setIsSelecting(false);
  }, []);

  // Set up global event listeners when component mounts
  useEffect(() => {
    document.addEventListener('mouseup', handleSelectionChange);
    document.addEventListener('keyup', handleSelectionChange);

    return () => {
      document.removeEventListener('mouseup', handleSelectionChange);
      document.removeEventListener('keyup', handleSelectionChange);
    };
  }, [handleSelectionChange]);

  return {
    selectedText,
    selectionInfo,
    isSelecting,
    startSelection,
    endSelection,
    clearSelection,
    getSelectedText,
    getSelectionInfo
  };
};

export default useTextSelection;