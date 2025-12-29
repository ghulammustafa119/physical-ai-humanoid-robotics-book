import React from 'react';
import ChatPanel from '../components/ChatPanel';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <>
      {children}
      <ChatPanel />
    </>
  );
}
