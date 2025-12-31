import React from 'react';
import Link from '@docusaurus/Link';
import { useAuth } from './AuthProvider';
import styles from './styles.module.css';

const AuthNavbarItem: React.FC = () => {
  const { user, isAuthenticated, signOut, loading } = useAuth();

  if (loading) {
    return null;
  }

  const handleSignOut = async (e: React.MouseEvent) => {
    e.preventDefault();
    await signOut();
    window.location.href = '/physical-ai-humanoid-robotics-book/';
  };

  return (
    <div className={styles.authNav}>
      {isAuthenticated && user ? (
        <div className={styles.authenticatedNav}>
          <span className={styles.userEmail}>{user.email}</span>
          <Link
            className={styles.navLink}
            to="/profile">
            Profile
          </Link>
          <button
            onClick={handleSignOut}
            className={styles.navLink}
            style={{ background: 'none', border: 'none', cursor: 'pointer', font: 'inherit' }}>
            Sign Out
          </button>
        </div>
      ) : (
        <div className={styles.unauthenticatedNav}>
          <Link
            className={styles.navLink}
            to="/signin">
            Sign In
          </Link>
          <Link
            className={styles.navLink}
            to="/signup">
            Sign Up
          </Link>
        </div>
      )}
    </div>
  );
};

export default AuthNavbarItem;
