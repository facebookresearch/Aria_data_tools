/**
 * (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.
 */

const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

// With JSDoc @type annotations, IDEs can provide config autocompletion
/** @type {import('@docusaurus/types').DocusaurusConfig} */
(module.exports = {
  title: 'Aria Research Kit',
  tagline: 'Open tooling to support researchers expand the horizons of Augmented Reality, Machine Learning and Artificial Intelligence',
  url: 'https://facebookresearch.github.io',
  baseUrl: '/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'throw',
  trailingSlash: true,
  favicon: 'img/aria_icon.png',
  organizationName: 'facebook',
  projectName: 'aria-research-kit-open-data-tools',

  presets: [
    [
      'docusaurus-plugin-internaldocs-fb/docusaurus-preset',
      /** @type {import('docusaurus-plugin-internaldocs-fb').PresetOptions} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/facebookresearch/aria_data_tools', // TODO Please change this to your repo. - changed to github repo -EA
        },
        blog: {
          showReadingTime: true,
          editUrl: 'https://www.internalfb.com/code/fbsource/arvr/projects/ariane/aria_research_kit/aria_data_tools/website', // TODO change to path to your project in fbsource/www
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title: 'Aria Data Tools',
        logo: {
          alt: 'aria-research-kit-sdk Logo',
          src: 'img/glasses-solid.svg',
        },
        items: [
          {
            type: 'doc',
            docId: 'overview',
            position: 'left',
            label: 'Documentation',
          },
          {
            href: 'https://about.facebook.com/realitylabs/projectaria/',
            label: 'Aria Pilot Dataset',
            position: 'left',
          },
          {
            href: 'https://github.com/facebookresearch/aria_data_tools',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learn',
            items: [
              {
                label: 'Documentation',
                to: '/docs/overview',
              },
            ],
          },
          {
            title: 'Project Aria',
            items: [
              {
                label: 'Introducing Project Aria',
                href: 'https://about.facebook.com/realitylabs/projectaria/',
              },
              {
                label: 'Aria Pilot Dataset',
                href: 'https://twitter.com/docusaurus',
              },
            ],
          },
          {
            title: 'Responsible Innovation',
            items: [
              {
                label: 'Responsible Innovation Principles',
                to: 'https://about.facebook.com/realitylabs/responsible-innovation-principles/',
              },
              {
                label: 'Project Aria Research Community Guidelines',
                href: 'https://about.facebook.com/realitylabs/projectaria/community-guidelines/',
              },
            ],
          },
        ],
        logo: {
          alt: 'Facebook Open Source Logo',
          src: 'img/oss_logo.png',
          href: 'https://opensource.facebook.com',
        },

        copyright: `Copyright © ${new Date().getFullYear()} Meta Platforms, Inc. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),
});
