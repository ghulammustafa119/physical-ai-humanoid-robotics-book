/**
 * Auth Navigation Component
 * Provides navigation links for authentication (Sign In/Up/Out)
 */
import React from 'react';
import { useAuth } from './AuthProvider';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

const AuthNavigation: React.FC = () => {
  const { user, loading, signOut } = useAuth();

  const handleSignOut = async () => {
    await signOut();
  };

  if (loading) {
    return <div>Loading...</div>;
  }

  return (
    <div className={styles.authNav}>
      {user ? (
        <div className={styles.authenticatedNav}>
          <span className={styles.userEmail}>Welcome, {user.email}</span>
          <Link to="/profile" className={styles.navLink}>Profile</Link>
          <button onClick={handleSignOut} className={styles.signOutButton}>
            Sign Out
          </button>
        </div>
      ) : (
        <div className={styles.unauthenticatedNav}>
          <Link to="/signin" className={styles.navLink}>Sign In</Link>
          <Link to="/signup" className={styles.navLink}>Sign Up</Link>
        </div>
      )}
    </div>
  );
};

export default AuthNavigation;