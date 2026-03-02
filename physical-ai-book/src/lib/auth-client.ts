import { createAuthClient } from "better-auth/client";

const API_BASE_URL =
  process.env.NODE_ENV === 'production'
    ? 'https://ghulammustafabhutto-gmbhutto.hf.space'
    : 'http://localhost:8000';

export const authClient = createAuthClient({
  baseURL: API_BASE_URL,
});

export { API_BASE_URL };
