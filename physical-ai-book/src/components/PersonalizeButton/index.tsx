import React, { useState, useEffect, useCallback } from 'react';
import { useLocation } from '@docusaurus/router';
import { useAuth } from '../auth/AuthProvider';
import { personalizeChapter } from '../../utils/auth';
import styles from './styles.module.css';

export default function PersonalizeButton(): React.JSX.Element | null {
  const { isAuthenticated } = useAuth();
  const location = useLocation();
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [userLevel, setUserLevel] = useState<string>('');
  const [showPersonalized, setShowPersonalized] = useState(false);

  // Cache key based on current page path
  const cacheKey = `personalize_${location.pathname}`;

  // Load cached personalized content on mount / page change
  useEffect(() => {
    const cached = sessionStorage.getItem(cacheKey);
    if (cached) {
      try {
        const parsed = JSON.parse(cached);
        setPersonalizedContent(parsed.content);
        setUserLevel(parsed.level);
        setShowPersonalized(true);
      } catch {
        sessionStorage.removeItem(cacheKey);
      }
    } else {
      setPersonalizedContent(null);
      setShowPersonalized(false);
      setUserLevel('');
    }
    setError(null);
  }, [location.pathname]);

  const handlePersonalize = useCallback(async () => {
    setIsPersonalizing(true);
    setError(null);

    try {
      // Extract chapter content from the rendered page
      const contentEl = document.querySelector('.theme-doc-markdown');
      if (!contentEl) {
        setError('Could not find chapter content on this page.');
        return;
      }

      const chapterContent = contentEl.textContent || '';
      const titleEl = contentEl.querySelector('h1');
      const chapterTitle = titleEl?.textContent || document.title;

      if (chapterContent.length < 50) {
        setError('This page does not have enough content to personalize.');
        return;
      }

      const result = await personalizeChapter(chapterContent, chapterTitle);

      if (!result) {
        setError('Failed to personalize content. Please try again.');
        return;
      }

      setPersonalizedContent(result.personalized_content);
      setUserLevel(result.user_level);
      setShowPersonalized(true);

      // Cache in session storage
      sessionStorage.setItem(
        cacheKey,
        JSON.stringify({ content: result.personalized_content, level: result.user_level })
      );
    } catch (err) {
      setError('An error occurred while personalizing. Please try again.');
      console.error('Personalization error:', err);
    } finally {
      setIsPersonalizing(false);
    }
  }, [cacheKey]);

  const handleReset = useCallback(() => {
    setShowPersonalized(false);
    setPersonalizedContent(null);
    setUserLevel('');
    sessionStorage.removeItem(cacheKey);
  }, [cacheKey]);

  const handleClick = () => {
    if (!isAuthenticated) {
      window.location.href = '/physical-ai-humanoid-robotics-book/signin';
      return;
    }
    handlePersonalize();
  };

  return (
    <div className={styles.container}>
      {error && <div className={styles.error}>{error}</div>}

      <div className={styles.personalizeBar}>
        <p className={styles.barLabel}>
          Click to personalize this chapter based on your experience level.
        </p>

        <div className={styles.btnGroup}>
          <button
            className={styles.personalizeBtn}
            onClick={handleClick}
            disabled={isPersonalizing}
          >
            {isPersonalizing ? (
              <>
                <span className={styles.spinner} />
                Personalizing...
              </>
            ) : (
              'Personalize for Me'
            )}
          </button>

          {showPersonalized && (
            <button className={styles.resetBtn} onClick={handleReset}>
              Show Original
            </button>
          )}
        </div>
      </div>

      {showPersonalized && personalizedContent && (
        <div className={styles.personalizedPanel}>
          <div className={styles.panelHeader}>
            <span>Personalized Content ({userLevel} level)</span>
            <button
              className={styles.resetBtn}
              onClick={handleReset}
              style={{ color: 'white', borderColor: 'rgba(255,255,255,0.5)' }}
            >
              Close
            </button>
          </div>
          <div
            className={styles.panelContent}
            dangerouslySetInnerHTML={{ __html: formatMarkdown(personalizedContent) }}
          />
        </div>
      )}
    </div>
  );
}

/**
 * Simple markdown to HTML converter for personalized content.
 * Handles headings, bold, italic, code blocks, lists, and paragraphs.
 */
function formatMarkdown(md: string): string {
  let html = md
    // Code blocks
    .replace(/```(\w*)\n([\s\S]*?)```/g, '<pre><code class="language-$1">$2</code></pre>')
    // Inline code
    .replace(/`([^`]+)`/g, '<code>$1</code>')
    // Headings
    .replace(/^#### (.+)$/gm, '<h4>$1</h4>')
    .replace(/^### (.+)$/gm, '<h3>$1</h3>')
    .replace(/^## (.+)$/gm, '<h2>$1</h2>')
    .replace(/^# (.+)$/gm, '<h1>$1</h1>')
    // Bold and italic
    .replace(/\*\*(.+?)\*\*/g, '<strong>$1</strong>')
    .replace(/\*(.+?)\*/g, '<em>$1</em>')
    // Unordered lists
    .replace(/^- (.+)$/gm, '<li>$1</li>')
    // Ordered lists
    .replace(/^\d+\. (.+)$/gm, '<li>$1</li>')
    // Line breaks to paragraphs
    .replace(/\n\n/g, '</p><p>')
    .replace(/\n/g, '<br/>');

  // Wrap consecutive <li> in <ul>
  html = html.replace(/(<li>.*?<\/li>(\s*<br\/>)?)+/g, (match) => {
    return '<ul>' + match.replace(/<br\/>/g, '') + '</ul>';
  });

  return '<p>' + html + '</p>';
}
