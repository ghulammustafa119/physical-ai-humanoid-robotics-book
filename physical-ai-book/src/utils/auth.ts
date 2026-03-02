/**
 * Auth Utilities
 * Handles localStorage token management and API communication
 */
import { authClient, API_BASE_URL } from '../lib/auth-client';

export { API_BASE_URL };

// Get auth token from localStorage
export const getAuthToken = (): string | null => {
  return localStorage.getItem('auth_token');
};

// Get user email from localStorage
export const getUserEmail = (): string | null => {
  return localStorage.getItem('user_email');
};

// Set auth token and user email in localStorage
export const setAuthTokens = (token: string, email: string): void => {
  localStorage.setItem('auth_token', token);
  localStorage.setItem('user_email', email);
};

// Clear auth tokens from localStorage
export const clearAuthTokens = (): void => {
  localStorage.removeItem('auth_token');
  localStorage.removeItem('user_email');
};

// Check if user is authenticated
export const isAuthenticated = (): boolean => {
  const token = getAuthToken();
  return !!token;
};

// Get user profile from API
export const getUserProfile = async (): Promise<any | null> => {
  const token = getAuthToken();

  if (!token) {
    throw new Error('Not authenticated');
  }

  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/auth/profile`, {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
      credentials: 'include', // Support session cookies
    });

    if (!response.ok) {
      throw new Error('Failed to fetch profile');
    }

    return await response.json();
  } catch (error) {
    console.error('Error fetching user profile:', error);
    throw error;
  }
};

// Update user profile via API
export const updateUserProfile = async (profileData: Partial<any>): Promise<any> => {
  const token = getAuthToken();

  if (!token) {
    throw new Error('Not authenticated');
  }

  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/auth/profile`, {
      method: 'PUT',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(profileData),
      credentials: 'include', // Support session cookies
    });

    if (!response.ok) {
      throw new Error('Failed to update profile');
    }

    return await response.json();
  } catch (error) {
    console.error('Error updating user profile:', error);
    throw error;
  }
};

// Sign out user via better-auth client
export const signOut = async (): Promise<void> => {
  try {
    await authClient.signOut({
      fetchOptions: { credentials: 'include' },
    });
  } catch (error) {
    console.error('Error during signout:', error);
  } finally {
    clearAuthTokens();
  }
};

// Personalize chapter content based on user profile
export const personalizeChapter = async (
  chapterContent: string,
  chapterTitle: string
): Promise<{ personalized_content: string; user_level: string } | null> => {
  const token = getAuthToken();
  if (!token) return null;

  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/personalize`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
      credentials: 'include',
      body: JSON.stringify({
        chapter_content: chapterContent,
        chapter_title: chapterTitle,
      }),
    });

    if (!response.ok) {
      throw new Error('Failed to personalize chapter');
    }

    return await response.json();
  } catch (error) {
    console.error('Error personalizing chapter:', error);
    return null;
  }
};

// Translate chapter content to Urdu (no auth required)
export const translateChapter = async (
  chapterContent: string,
  chapterTitle: string
): Promise<{ translated_content: string; source_language: string; target_language: string } | null> => {
  try {
    const response = await fetch(`${API_BASE_URL}/api/v1/translate`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        chapter_content: chapterContent,
        chapter_title: chapterTitle,
      }),
    });

    if (!response.ok) {
      throw new Error('Failed to translate chapter');
    }

    return await response.json();
  } catch (error) {
    console.error('Error translating chapter:', error);
    return null;
  }
};

// Get personalization context for RAG
export const getPersonalizationContext = async (): Promise<any | null> => {
  if (!isAuthenticated()) {
    return null; // Return null for guest users
  }

  try {
    const profile = await getUserProfile();

    // Construct personalization context from profile data
    const personalizationContext = {
      programming_level: profile.programming_level,
      python_level: profile.python_level,
      ai_ml_level: profile.ai_ml_level,
      robotics_level: profile.robotics_level,
      system_type: profile.system_type,
      gpu_availability: profile.gpu_availability,
      hardware_access: profile.hardware_access,
      simulator_experience: profile.simulator_experience,
      profile_completeness: profile.profile_completeness,
    };

    return personalizationContext;
  } catch (error) {
    console.error('Error getting personalization context:', error);
    return null; // Return null if unable to get context
  }
};