import React, { useState, useEffect, useCallback } from 'react';
import { useLocation } from '@docusaurus/router';
import { translateChapter } from '../../utils/auth';
import styles from './styles.module.css';

export default function TranslateButton(): React.JSX.Element | null {
  const location = useLocation();
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [isTranslating, setIsTranslating] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [showTranslated, setShowTranslated] = useState(false);

  // Cache key based on current page path
  const cacheKey = `translate_${location.pathname}`;

  // Load cached translated content on mount / page change
  useEffect(() => {
    const cached = sessionStorage.getItem(cacheKey);
    if (cached) {
      try {
        const parsed = JSON.parse(cached);
        setTranslatedContent(parsed.content);
        setShowTranslated(true);
      } catch {
        sessionStorage.removeItem(cacheKey);
      }
    } else {
      setTranslatedContent(null);
      setShowTranslated(false);
    }
    setError(null);
  }, [location.pathname]);

  const handleTranslate = useCallback(async () => {
    setIsTranslating(true);
    setError(null);

    try {
      const contentEl = document.querySelector('.theme-doc-markdown');
      if (!contentEl) {
        setError('Could not find chapter content on this page.');
        return;
      }

      const chapterContent = contentEl.textContent || '';
      const titleEl = contentEl.querySelector('h1');
      const chapterTitle = titleEl?.textContent || document.title;

      if (chapterContent.length < 50) {
        setError('This page does not have enough content to translate.');
        return;
      }

      const result = await translateChapter(chapterContent, chapterTitle);

      if (!result) {
        setError('Failed to translate content. Please try again.');
        return;
      }

      setTranslatedContent(result.translated_content);
      setShowTranslated(true);

      sessionStorage.setItem(
        cacheKey,
        JSON.stringify({ content: result.translated_content })
      );
    } catch (err) {
      setError('An error occurred while translating. Please try again.');
      console.error('Translation error:', err);
    } finally {
      setIsTranslating(false);
    }
  }, [cacheKey]);

  const handleReset = useCallback(() => {
    setShowTranslated(false);
    setTranslatedContent(null);
    sessionStorage.removeItem(cacheKey);
  }, [cacheKey]);

  return (
    <div className={styles.container}>
      {error && <div className={styles.error}>{error}</div>}

      <div className={styles.translateBar}>
        <p className={styles.barLabel}>
          اردو میں ترجمہ کریں — Click to translate this chapter to Urdu.
        </p>

        <div className={styles.btnGroup}>
          <button
            className={styles.translateBtn}
            onClick={handleTranslate}
            disabled={isTranslating}
          >
            {isTranslating ? (
              <>
                <span className={styles.spinner} />
                Translating...
              </>
            ) : (
              'Translate to Urdu'
            )}
          </button>

          {showTranslated && (
            <button className={styles.resetBtn} onClick={handleReset}>
              Show Original
            </button>
          )}
        </div>
      </div>

      {showTranslated && translatedContent && (
        <div className={styles.translatedPanel}>
          <div className={styles.panelHeader}>
            <span>اردو ترجمہ — Urdu Translation</span>
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
            dir="rtl"
            dangerouslySetInnerHTML={{ __html: formatMarkdown(translatedContent) }}
          />
        </div>
      )}
    </div>
  );
}

/**
 * Simple markdown to HTML converter for translated content.
 */
function formatMarkdown(md: string): string {
  let html = md
    .replace(/```(\w*)\n([\s\S]*?)```/g, '<pre><code class="language-$1">$2</code></pre>')
    .replace(/`([^`]+)`/g, '<code>$1</code>')
    .replace(/^#### (.+)$/gm, '<h4>$1</h4>')
    .replace(/^### (.+)$/gm, '<h3>$1</h3>')
    .replace(/^## (.+)$/gm, '<h2>$1</h2>')
    .replace(/^# (.+)$/gm, '<h1>$1</h1>')
    .replace(/\*\*(.+?)\*\*/g, '<strong>$1</strong>')
    .replace(/\*(.+?)\*/g, '<em>$1</em>')
    .replace(/^- (.+)$/gm, '<li>$1</li>')
    .replace(/^\d+\. (.+)$/gm, '<li>$1</li>')
    .replace(/\n\n/g, '</p><p>')
    .replace(/\n/g, '<br/>');

  html = html.replace(/(<li>.*?<\/li>(\s*<br\/>)?)+/g, (match) => {
    return '<ul>' + match.replace(/<br\/>/g, '') + '</ul>';
  });

  return '<p>' + html + '</p>';
}
