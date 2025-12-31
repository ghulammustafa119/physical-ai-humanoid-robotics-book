import React from 'react';
import Layout from '@theme/Layout';
import SignUp from '../components/auth/SignUp';

export default function SignUpPage(): JSX.Element {
  return (
    <Layout title="Sign Up" description="Create a new account">
      <main>
        <div style={{ padding: '2rem', maxWidth: '600px', margin: '0 auto' }}>
          <SignUp />
        </div>
      </main>
    </Layout>
  );
}