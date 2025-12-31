import React from 'react';
import { AuthProvider } from '../components/auth/AuthProvider';
import ChatPanel from '../components/ChatPanel';
import ProfileCompletionPopup from '../components/auth/ProfileCompletionPopup';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <AuthProvider>
      <>
        {children}
        <ProfileCompletionPopup />
        <ChatPanel />
      </>
    </AuthProvider>
  );
}
