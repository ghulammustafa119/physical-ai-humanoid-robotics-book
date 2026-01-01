/**
 * Better Auth Client Integration Utilities
 * Handles authentication state management and API communication with Better Auth
 */

// API Base URL configuration
export const API_BASE_URL =
  process.env.NODE_ENV === 'production'
    ? 'https://ghulammustafabhutto-gmbhutto.hf.space' // Hugging Face Backend URL
    : 'http://localhost:8000';

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

// Sign out user
export const signOut = async (): Promise<void> => {
  const token = getAuthToken();

  if (!token) {
    clearAuthTokens();
    return;
  }

  try {
    await fetch(`${API_BASE_URL}/api/v1/auth/signout`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
      credentials: 'include', // Support session cookies
    });
  } catch (error) {
    console.error('Error during signout:', error);
    // Even if API call fails, clear local tokens
  } finally {
    clearAuthTokens();
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