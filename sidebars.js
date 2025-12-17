// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
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

module.exports = sidebars;