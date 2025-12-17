import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-humanoid-robotics-book/docs',
    component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs', '2d8'),
    routes: [
      {
        path: '/physical-ai-humanoid-robotics-book/docs',
        component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs', '4c4'),
        routes: [
          {
            path: '/physical-ai-humanoid-robotics-book/docs',
            component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs', 'be8'),
            routes: [
              {
                path: '/physical-ai-humanoid-robotics-book/docs/gazebo/chapters/chapter1',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/gazebo/chapters/chapter1', '346'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/gazebo/chapters/chapter2',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/gazebo/chapters/chapter2', '9cb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/gazebo/chapters/chapter3',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/gazebo/chapters/chapter3', '714'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/gazebo/chapters/chapter4',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/gazebo/chapters/chapter4', '81b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/intro',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/intro', '0da'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/isaac/chapters/chapter1',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/isaac/chapters/chapter1', '06f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/isaac/chapters/chapter2',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/isaac/chapters/chapter2', 'a52'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/isaac/chapters/chapter3',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/isaac/chapters/chapter3', 'dde'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/isaac/chapters/chapter4',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/isaac/chapters/chapter4', 'e44'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/ros2/chapters/chapter1',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/ros2/chapters/chapter1', '31c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/ros2/chapters/chapter2',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/ros2/chapters/chapter2', '0fa'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/ros2/chapters/chapter3',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/ros2/chapters/chapter3', 'b22'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/ros2/chapters/chapter4',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/ros2/chapters/chapter4', '980'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/vla/chapters/chapter1',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/vla/chapters/chapter1', '72a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/vla/chapters/chapter2',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/vla/chapters/chapter2', '5a7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/vla/chapters/chapter3',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/vla/chapters/chapter3', '59b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics-book/docs/vla/chapters/chapter4',
                component: ComponentCreator('/physical-ai-humanoid-robotics-book/docs/vla/chapters/chapter4', '111'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
