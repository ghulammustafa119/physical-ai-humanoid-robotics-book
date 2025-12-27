import axios, { AxiosInstance } from 'axios';
import { ChatRequest, ChatResponse, TextSelection } from '../types/chat';

const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';

class ApiClient {
  private client: AxiosInstance;

  constructor() {
    this.client = axios.create({
      baseURL: API_BASE_URL,
      timeout: 30000, // 30 second timeout
      headers: {
        'Content-Type': 'application/json',
      },
    });

    // Add request interceptor for authentication if needed
    this.client.interceptors.request.use(
      (config) => {
        // Add auth token if available
        const token = localStorage.getItem('authToken');
        if (token) {
          config.headers.Authorization = `Bearer ${token}`;
        }
        return config;
      },
      (error) => {
        return Promise.reject(error);
      }
    );

    // Add response interceptor for error handling
    this.client.interceptors.response.use(
      (response) => response,
      (error) => {
        console.error('API Error:', error);
        return Promise.reject(error);
      }
    );
  }

  // Chat API methods
  async sendChatQuery(chatRequest: ChatRequest): Promise<ChatResponse> {
    try {
      const response = await this.client.post('/api/v1/chat', chatRequest);
      return response.data;
    } catch (error) {
      console.error('Error sending chat query:', error);
      throw error;
    }
  }

  // Text Selection API methods
  async createTextSelection(selection: Omit<TextSelection, 'id' | 'createdAt'>): Promise<TextSelection> {
    try {
      const response = await this.client.post('/api/v1/text-selection', selection);
      return response.data;
    } catch (error) {
      console.error('Error creating text selection:', error);
      throw error;
    }
  }

  async getTextSelection(selectionId: string): Promise<TextSelection> {
    try {
      const response = await this.client.get(`/api/v1/text-selection/${selectionId}`);
      return response.data;
    } catch (error) {
      console.error('Error getting text selection:', error);
      throw error;
    }
  }

  // Additional API methods can be added here as needed
}

export default new ApiClient();