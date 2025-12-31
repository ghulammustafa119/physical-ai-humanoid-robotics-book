import React from 'react';
import Layout from '@theme/Layout';
import SignIn from '../components/auth/SignIn';

export default function SignInPage(): JSX.Element {
  return (
    <Layout title="Sign In" description="Sign in to your account">
      <main>
        <div style={{ padding: '2rem', maxWidth: '600px', margin: '0 auto' }}>
          <SignIn />
        </div>
      </main>
    </Layout>
  );
}