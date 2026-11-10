import { defineConfig } from 'vitepress'
import { withMermaid } from 'vitepress-plugin-mermaid'

const sections = [
  { text: '快速开始', link: '/getting-started/' },
  { text: '系统架构', link: '/architecture/' },
  { text: '模块手册', link: '/modules/' },
  { text: '算法原理', link: '/algorithms/' },
  { text: '开发维护', link: '/development/' },
]

export default withMermaid(defineConfig({
  lang: 'zh-CN',
  title: 'Bochu Capstone',
  description: 'ParamCAD Studio 项目技术文档',
  base: '/Bochu_Capstone/',
  srcDir: 'site',
  markdown: { math: true },
  themeConfig: {
    nav: [sections[0], sections[2], sections[4]],
    sidebar: [{ text: '项目文档', items: sections }],
    socialLinks: [{ icon: 'github', link: 'https://github.com/yang-pei-hua/Bochu_Capstone' }],
    outline: { label: '本页目录' },
    docFooter: { prev: '上一页', next: '下一页' },
    search: {
      provider: 'local',
      options: {
        miniSearch: {
          options: {
            tokenize: (text) => {
              const segmenter = new Intl.Segmenter('zh-CN', { granularity: 'word' })
              return Array.from(segmenter.segment(text))
                .filter(({ isWordLike }) => isWordLike)
                .map(({ segment }) => segment)
            },
          },
        },
        translations: { button: { buttonText: '搜索文档', buttonAriaLabel: '搜索文档' } },
      },
    },
  },
}))
