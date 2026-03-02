import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import TranslateButton from '@site/src/components/TranslateButton';

export default function LayoutWrapper(props: Record<string, unknown>): React.JSX.Element {
  return (
    <>
      <PersonalizeButton />
      <TranslateButton />
      <Layout {...props} />
    </>
  );
}
