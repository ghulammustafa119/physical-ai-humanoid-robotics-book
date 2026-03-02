/**
 * Auth Provider Component
 * Manages authentication state using better-auth client SDK
 */
import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react';
import { authClient } from '../../lib/auth-client';
import {
  setAuthTokens,
  clearAuthTokens,
  isAuthenticated as checkIsAuthenticated,
  getUserEmail,
  getUserProfile,
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
      // First check localStorage for existing token
      if (checkIsAuthenticated()) {
        try {
          const email = getUserEmail();
          const profile = await getUserProfile();
          setUser({ email, profile });
        } catch (error) {
          console.error('Error checking auth status:', error);
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

      const { data, error } = await authClient.signIn.email({
        email,
        password,
        fetchOptions: { credentials: 'include' },
      });

      if (error) {
        const errAny = error as any;
        const msg = errAny.body?.message
          || errAny.body?.error?.message
          || error.message
          || 'Invalid email or password';
        throw new Error(msg);
      }

      // Extract token — SDK may return it as data.token or nested in session
      const dataAny = data as any;
      const token = dataAny?.token || dataAny?.session?.token;
      const userEmail = dataAny?.user?.email;

      if (token && userEmail) {
        setAuthTokens(token, userEmail);
        setUser({
          email: userEmail,
          profile: dataAny.user,
        });
      }

      return true;
    } catch (error) {
      console.error('Sign in error:', error);
      throw error;
    } finally {
      setLoading(false);
    }
  };

  const signUp = async (email: string, password: string): Promise<boolean> => {
    try {
      setLoading(true);

      const { error } = await authClient.signUp.email({
        email,
        password,
        name: email.split('@')[0],
        fetchOptions: { credentials: 'include' },
      });

      if (error) {
        const errAny = error as any;
        const msg = errAny.body?.message
          || errAny.body?.error?.message
          || error.message
          || 'Signup failed';
        throw new Error(msg);
      }

      return true;
    } catch (error) {
      console.error('Sign up error:', error);
      throw error;
    } finally {
      setLoading(false);
    }
  };

  const signOut = async () => {
    try {
      await authClient.signOut({
        fetchOptions: { credentials: 'include' },
      });
    } catch (error) {
      console.error('Sign out error:', error);
    } finally {
      clearAuthTokens();
      setUser(null);
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
      setUser({ email, profile });
    } catch (error) {
      console.error('Error refreshing profile:', error);
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