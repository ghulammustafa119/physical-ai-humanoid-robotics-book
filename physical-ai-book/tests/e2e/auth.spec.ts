/**
 * E2E Authentication Tests
 * Tests for the complete authentication flow including signup, signin, profile management, and signout
 */

import { test, expect, Page } from '@playwright/test';

// Test environment variables
const BASE_URL = 'http://localhost:3000'; // Update to your Docusaurus URL
const API_BASE_URL = 'http://localhost:8000/api/v1';

test.describe('Authentication Flow', () => {
  let page: Page;

  test.beforeEach(async ({ browser }) => {
    page = await browser.newPage();
  });

  test.afterEach(async () => {
    await page.close();
  });

  test('should allow user to sign up with profile information', async () => {
    await page.goto(`${BASE_URL}/signup`);

    // Fill in signup form
    await page.locator('input[name="email"]').fill('testuser@example.com');
    await page.locator('input[name="password"]').fill('TestPassword123!');
    await page.locator('input[name="confirmPassword"]').fill('TestPassword123!');

    // Expand profile questions
    await page.locator('button').filter({ hasText: 'Add Profile Information (Optional)' }).click();

    // Fill in profile information
    await page.locator('#programmingLevel').selectOption('intermediate');
    await page.locator('#pythonLevel').selectOption('strong');
    await page.locator('#aiMlLevel').selectOption('applied');
    await page.locator('#roboticsLevel').selectOption('practical');
    await page.locator('#systemType').selectOption('desktop');
    await page.locator('#gpuAvailability').selectOption('nvidia_cuda');
    await page.locator('#hardwareAccess').selectOption('real');

    // Select simulator experience
    await page.locator('input[value="gazebo"]').click();
    await page.locator('input[value="isaac_sim"]').click();

    // Submit the form
    await page.locator('button[type="submit"]').click();

    // Verify successful signup and redirect to profile or dashboard
    await expect(page).toHaveURL(`${BASE_URL}/profile`);
    await expect(page.locator('h2')).toContainText('User Profile');
  });

  test('should allow user to sign in with valid credentials', async () => {
    await page.goto(`${BASE_URL}/signin`);

    // Fill in sign in form
    await page.locator('input[name="email"]').fill('testuser@example.com');
    await page.locator('input[name="password"]').fill('TestPassword123!');

    // Submit the form
    await page.locator('button[type="submit"]').click();

    // Verify successful sign in and redirect to profile
    await expect(page).toHaveURL(`${BASE_URL}/profile`);
    await expect(page.locator('h2')).toContainText('User Profile');
  });

  test('should display error for invalid credentials', async () => {
    await page.goto(`${BASE_URL}/signin`);

    // Fill in incorrect credentials
    await page.locator('input[name="email"]').fill('testuser@example.com');
    await page.locator('input[name="password"]').fill('WrongPassword123!');

    // Submit the form
    await page.locator('button[type="submit"]').click();

    // Verify error message is displayed
    await expect(page.locator('.error')).toContainText('Signin failed');
  });

  test('should allow user to update profile information', async () => {
    // First sign in
    await page.goto(`${BASE_URL}/signin`);
    await page.locator('input[name="email"]').fill('testuser@example.com');
    await page.locator('input[name="password"]').fill('TestPassword123!');
    await page.locator('button[type="submit"]').click();

    // Navigate to profile page
    await page.goto(`${BASE_URL}/profile`);

    // Click edit button
    await page.locator('button').filter({ hasText: 'Edit Profile' }).click();

    // Update profile information
    await page.locator('#programming_level').selectOption('advanced');
    await page.locator('#python_level').selectOption('strong');
    await page.locator('#ai_ml_level').selectOption('applied');

    // Submit changes
    await page.locator('button').filter({ hasText: 'Save' }).click();

    // Verify changes are saved
    await expect(page.locator('div').filter({ hasText: 'Programming Level: advanced' })).toBeVisible();
  });

  test('should allow user to sign out', async () => {
    // First sign in
    await page.goto(`${BASE_URL}/signin`);
    await page.locator('input[name="email"]').fill('testuser@example.com');
    await page.locator('input[name="password"]').fill('TestPassword123!');
    await page.locator('button[type="submit"]').click();

    // Verify user is signed in
    await expect(page.locator('h2')).toContainText('User Profile');

    // Click sign out button
    await page.locator('button').filter({ hasText: 'Sign Out' }).click();

    // Verify user is redirected to home or sign in page
    await expect(page).toHaveURL(BASE_URL);
  });

  test('should validate required fields in signup form', async () => {
    await page.goto(`${BASE_URL}/signup`);

    // Submit form without filling required fields
    await page.locator('button[type="submit"]').click();

    // Verify error message is displayed
    await expect(page.locator('.error')).toContainText('Email and password are required');
  });

  test('should validate password match in signup form', async () => {
    await page.goto(`${BASE_URL}/signup`);

    // Fill in form with mismatched passwords
    await page.locator('input[name="email"]').fill('testuser@example.com');
    await page.locator('input[name="password"]').fill('TestPassword123!');
    await page.locator('input[name="confirmPassword"]').fill('DifferentPassword123!');

    // Submit the form
    await page.locator('button[type="submit"]').click();

    // Verify error message is displayed
    await expect(page.locator('.error')).toContainText('Passwords do not match');
  });

  test('should validate password length in signup form', async () => {
    await page.goto(`${BASE_URL}/signup`);

    // Fill in form with short password
    await page.locator('input[name="email"]').fill('testuser@example.com');
    await page.locator('input[name="password"]').fill('short');
    await page.locator('input[name="confirmPassword"]').fill('short');

    // Submit the form
    await page.locator('button[type="submit"]').click();

    // Verify error message is displayed
    await expect(page.locator('.error')).toContainText('Password must be at least 8 characters');
  });
});

test.describe('API Authentication Tests', () => {
  test('should create user via API', async ({ request }) => {
    const response = await request.post(`${API_BASE_URL}/auth/signup`, {
      data: {
        email: 'apiuser@example.com',
        password: 'ApiPassword123!',
        profile: {
          programming_level: 'beginner',
          python_level: 'basic',
          ai_ml_level: 'none',
          robotics_level: 'none',
          system_type: 'laptop',
          gpu_availability: 'none',
          hardware_access: 'none',
          simulator_experience: ['gazebo']
        }
      }
    });

    expect(response.status()).toBe(200);
    const data = await response.json();
    expect(data.user.email).toBe('apiuser@example.com');
    expect(data.session.token).toBeDefined();
  });

  test('should authenticate user via API', async ({ request }) => {
    const response = await request.post(`${API_BASE_URL}/auth/signin`, {
      data: {
        email: 'apiuser@example.com',
        password: 'ApiPassword123!'
      }
    });

    expect(response.status()).toBe(200);
    const data = await response.json();
    expect(data.user.email).toBe('apiuser@example.com');
    expect(data.session.token).toBeDefined();
  });

  test('should get user profile via API', async ({ request }) => {
    // First authenticate to get token
    const authResponse = await request.post(`${API_BASE_URL}/auth/signin`, {
      data: {
        email: 'apiuser@example.com',
        password: 'ApiPassword123!'
      }
    });

    const authData = await authResponse.json();
    const token = authData.session.token;

    // Get profile with token
    const response = await request.get(`${API_BASE_URL}/auth/profile`, {
      headers: {
        'Authorization': `Bearer ${token}`
      }
    });

    expect(response.status()).toBe(200);
    const data = await response.json();
    expect(data.email).toBe('apiuser@example.com');
    expect(data.programming_level).toBe('beginner');
  });

  test('should update user profile via API', async ({ request }) => {
    // First authenticate to get token
    const authResponse = await request.post(`${API_BASE_URL}/auth/signin`, {
      data: {
        email: 'apiuser@example.com',
        password: 'ApiPassword123!'
      }
    });

    const authData = await authResponse.json();
    const token = authData.session.token;

    // Update profile with token
    const response = await request.put(`${API_BASE_URL}/auth/profile`, {
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json'
      },
      data: {
        programming_level: 'intermediate'
      }
    });

    expect(response.status()).toBe(200);
    const data = await response.json();
    expect(data.programming_level).toBe('intermediate');
  });

  test('should sign out user via API', async ({ request }) => {
    // First authenticate to get token
    const authResponse = await request.post(`${API_BASE_URL}/auth/signin`, {
      data: {
        email: 'apiuser@example.com',
        password: 'ApiPassword123!'
      }
    });

    const authData = await authResponse.json();
    const token = authData.session.token;

    // Sign out with token
    const response = await request.post(`${API_BASE_URL}/auth/signout`, {
      headers: {
        'Authorization': `Bearer ${token}`
      }
    });

    expect(response.status()).toBe(200);
  });
});