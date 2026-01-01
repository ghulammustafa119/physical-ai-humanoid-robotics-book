/**
 * Auth Provider Component
 * Manages authentication state across the application
 */
import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react';
import {
  getAuthToken,
  getUserEmail,
  setAuthTokens,
  clearAuthTokens,
  isAuthenticated as checkIsAuthenticated,
  getUserProfile,
  signOut as signOutAPI,
  API_BASE_URL,
} from '../../utils/auth';

interface AuthContextType {
  user: {
    email: string | null;
    profile: any | null;
  } | null;
  loading: boolean;
  signIn: (email: string, password: string) => Promise<boolean>;
  signUp: (email: string, password: string, profile?: any) => Promise<boolean>;
  signOut: () => Promise<void>;
  refreshProfile: () => Promise<void>;
  isAuthenticated: boolean;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<AuthContextType['user']>(null);
  const [loading, setLoading] = useState(true);

  // Check authentication status on mount
  useEffect(() => {
    const checkAuthStatus = async () => {
      if (checkIsAuthenticated()) {
        try {
          const email = getUserEmail();
          const profile = await getUserProfile();

          setUser({
            email,
            profile,
          });
        } catch (error) {
          console.error('Error checking auth status:', error);
          // If token is invalid, clear it
          clearAuthTokens();
        }
      }
      setLoading(false);
    };

    checkAuthStatus();
  }, []);

  const signIn = async (email: string, password: string): Promise<boolean> => {
    try {
      setLoading(true);

      const response = await fetch(`${API_BASE_URL}/api/v1/auth/signin`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include', // Support session cookies
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Signin failed');
      }

      // Store session token and user info
      if (data.session && data.session.token) {
        setAuthTokens(data.session.token, data.user.email);

        setUser({
          email: data.user.email,
          profile: data.user.profile || data.user, // Fallback to user object if profile missing
        });
      }

      return true;
    } catch (error) {
      console.error('Sign in error:', error);
      return false;
    } finally {
      setLoading(false);
    }
  };

  const signUp = async (email: string, password: string, profile?: any): Promise<boolean> => {
    try {
      setLoading(true);

      const response = await fetch(`${API_BASE_URL}/api/v1/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include', // Support session cookies
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Signup failed');
      }

      // Automatically sign in after signup
      return await signIn(email, password);
    } catch (error) {
      console.error('Sign up error:', error);
      return false;
    } finally {
      setLoading(false);
    }
  };

  const signOut = async () => {
    try {
      await signOutAPI();
      setUser(null);
    } catch (error) {
      console.error('Sign out error:', error);
      // Still clear local state even if API call fails
      setUser(null);
    } finally {
      setLoading(false);
    }
  };

  const refreshProfile = async () => {
    if (!checkIsAuthenticated()) {
      setUser(null);
      return;
    }

    try {
      const email = getUserEmail();
      const profile = await getUserProfile();

      setUser({
        email,
        profile,
      });
    } catch (error) {
      console.error('Error refreshing profile:', error);
      // Don't clear tokens here to avoid infinite loops on 404/500
    }
  };

  const value = {
    user,
    loading,
    signIn,
    signUp,
    signOut,
    refreshProfile,
    isAuthenticated: !!user,
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};