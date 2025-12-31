import React from 'react';
import { AuthProvider } from '../components/auth/AuthProvider';
import ChatPanel from '../components/ChatPanel';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <AuthProvider>
      <>
        {children}
        <ChatPanel />
      </>
    </AuthProvider>
  );
}
