/**
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');
const repoUrl = 'https://github.com/facebookresearch/Aria_data_tools';

// With JSDoc @type annotations, IDEs can provide config autocompletion
/** @type {import('@docusaurus/types').DocusaurusConfig} */
(module.exports = {
  title: 'Aria Data Tools',
  tagline: 'An archive of the original Project Aria open source tooling, this website will be deleted in September 2024. Go to Project Aria Tools for current documentation and tooling.',
  url: 'https://facebookresearch.github.io',
  baseUrl: '/Aria_data_tools/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'throw',
  trailingSlash: true,
  favicon: 'img/glasses-solid.svg',
  organizationName: 'facebook',
  projectName: 'aria-research-kit-open-data-tools',

  presets: [
    [
      'docusaurus-plugin-internaldocs-fb/docusaurus-preset',
      /** @type {import('docusaurus-plugin-internaldocs-fb').PresetOptions} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/facebookresearch/aria_data_tools', //
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
        title: 'Archive: Aria Data Tools',
        logo: {
          alt: 'aria-research-kit-sdk Logo',
          src: 'img/glasses-solid.svg',
        },
        items: [
          {
            type: 'doc',
            docId: 'pilotdata/pilotdata-index',
            position: 'left',
            label: 'Aria Pilot Dataset Documentation',
          },
                    {
            href: 'https://www.projectaria.com/datasets/apd/',
            label: 'Aria Pilot Dataset',
            position: 'left',
          },
                    {
            href: 'https://facebookresearch.github.io/projectaria_tools/docs/intro',
            position: 'left',
            label: 'Project Aria Tools',
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
            title: 'Project Aria',
            items: [
              {
                label: 'About Project Aria',
                href: 'https://www.projectaria.com/',
              },
              {
                label: 'Project Aria Tools',
                href: 'https://facebookresearch.github.io/projectaria_tools/docs/intro',
              },
            ],
          },
          {
            title: 'Open Source Initiatives (OSI)',
            items: [
              {
                label: 'Open Datasets',
                href: 'https://www.projectaria.com/datasets/',
              },
              {
                label: 'Grand Challenges',
                href: 'https://www.projectaria.com/challenges/',
              },
            ],
          },
          {
            title: 'Responsible Innovation',
            items: [
              {
                label: 'Responsible Innovation Principles',
                href: 'https://about.facebook.com/realitylabs/responsible-innovation-principles/',
              },
              {
                label: 'Project Aria Research Community Guidelines',
                href: 'https://about.facebook.com/realitylabs/projectaria/community-guidelines/',
              },
            ],
          },
          {
            title: 'Legal',
            // Please do not remove the privacy and terms, it's a legal requirement.
            items: [
              {
                  label: 'Privacy Policy',
                  href: 'https://opensource.facebook.com/legal/privacy/',
              },
              {
                  label: 'Terms of Use',
                  href: 'https://opensource.facebook.com/legal/terms/',
              },
              {
                  label: 'Data Policy',
                  href: 'https://opensource.facebook.com/legal/data-policy/',
              },
              {
                  label: 'Cookie Policy',
                  href: 'https://opensource.facebook.com/legal/cookie-policy/',
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
