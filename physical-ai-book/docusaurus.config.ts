import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';

const config: Config = {
  title: 'Test Site',
  tagline: 'Test Site',
  favicon: 'img/favicon.ico',

  url: 'https://ghulammustafa119.github.io',
  baseUrl: '/physical-ai-humanoid-robotics-book/',

  organizationName: 'ghulammustafa119',
  projectName: 'physical-ai-humanoid-robotics-book',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Docs',
        },
        {
          to: '/signup',
          label: 'Sign Up',
          position: 'right',
        },
        {
          to: '/signin',
          label: 'Sign In',
          position: 'right',
        },
        {
          to: '/profile',
          label: 'Profile',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      copyright: `Copyright © ${new Date().getFullYear()} Test Site`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },
};

export default config;