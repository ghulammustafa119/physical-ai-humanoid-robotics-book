import React from 'react';
import Layout from '@theme/Layout';
import Profile from '../components/auth/Profile';

export default function ProfilePage(): JSX.Element {
  return (
    <Layout title="User Profile" description="Manage your user profile">
      <main>
        <div style={{ padding: '2rem', maxWidth: '600px', margin: '0 auto' }}>
          <Profile />
        </div>
      </main>
    </Layout>
  );
}