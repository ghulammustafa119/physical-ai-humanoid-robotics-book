import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import { useAuth } from './AuthProvider';
import styles from './styles.module.css';

const ProfileCompletionPopup: React.FC = () => {
  const { user, loading } = useAuth();
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    if (loading || !user) {
      setIsVisible(false);
      return;
    }

    const completeness = user.profile?.profile_completeness || 0;
    const completenessPercent = Math.round(completeness * 100);

    // 1. Detect profile completion
    if (completenessPercent >= 100) {
      // 2. Auto-dismiss popup when 100%
      setIsVisible(false);
      // 3. Persist dismissal state
      localStorage.setItem('profile_popup_dismissed', 'true');
      return;
    }

    // 4. Correct popup visibility logic
    const isDismissed = localStorage.getItem('profile_popup_dismissed') === 'true';
    if (!isDismissed && completenessPercent < 100) {
      setIsVisible(true);
    } else {
      setIsVisible(false);
    }
  }, [user, loading]);

  const handleDismiss = () => {
    setIsVisible(false);
    localStorage.setItem('profile_popup_dismissed', 'true');
  };

  if (!isVisible || !user) return null;

  const completeness = user.profile?.profile_completeness || 0;
  const completenessPercent = Math.round(completeness * 100);

  return (
    <div className={styles.modalOverlay}>
      <div className={styles.modalContent}>
        <button className={styles.modalClose} onClick={handleDismiss} aria-label="Close">
          &times;
        </button>
        <h3 className={styles.modalTitle}>Complete Your Profile</h3>
        <p>Your profile is <strong>{completenessPercent}%</strong> complete. Completing your profile helps us personalize your book experience.</p>

        <div className={styles.progressBar}>
          <div
            className={styles.progressFill}
            style={{ width: `${completenessPercent}%` }}
          />
        </div>

        <div className={styles.modalFooter}>
          <Link
            className={styles.button}
            to="/profile"
            onClick={() => setIsVisible(false)}
          >
            Go to Profile
          </Link>
          <button
            className={styles.cancelButton}
            onClick={handleDismiss}
          >
            Later
          </button>
        </div>
      </div>
    </div>
  );
};

export default ProfileCompletionPopup;
