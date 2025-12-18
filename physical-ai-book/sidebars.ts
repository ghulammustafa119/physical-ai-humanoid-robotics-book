import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'ros2/chapters/chapter1',
        'ros2/chapters/chapter2',
        'ros2/chapters/chapter3',
        'ros2/chapters/chapter4',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'gazebo/chapters/chapter1',
        'gazebo/chapters/chapter2',
        'gazebo/chapters/chapter3',
        'gazebo/chapters/chapter4',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'isaac/chapters/chapter1',
        'isaac/chapters/chapter2',
        'isaac/chapters/chapter3',
        'isaac/chapters/chapter4',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'vla/chapters/chapter1',
        'vla/chapters/chapter2',
        'vla/chapters/chapter3',
        'vla/chapters/chapter4',
      ],
    },
  ],
};

export default sidebars;
