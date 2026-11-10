# 开发维护

## 本地预览

安装 Node.js 24 后，在仓库根目录运行以下命令，打开终端给出的地址。

```powershell
npm --prefix docs ci
npm --prefix docs run dev
```

检查发布构建时运行 `npm --prefix docs run build`。构建产物位于 `docs/.vitepress/dist/`。

## 添加页面

将 Markdown 放入 `docs/site/` 对应栏目。需要侧边栏入口时，在 `docs/.vitepress/config.mts` 中添加链接。

## 发布

首次部署需要有权限的仓库成员在 **Settings → Pages → Source** 选择 **GitHub Actions**。之后，合入 `main` 的文档变更会触发部署。也可以在 **Actions → Documentation → Run workflow** 从 `main` 手动触发。
