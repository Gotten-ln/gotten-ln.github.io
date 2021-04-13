---
layout: post 
title:  "在GitHub和Gitee上部署Jekyll+NexT主题" 
categories:
 - Tutorial
tags:
 - Blog
 - Jekyll
 - NexT
 - GitHub
 - Gitee
---

NexT 是由 [Hexo NexT](https://github.com/iissnan/hexo-theme-next) 移植而来的 Jekyll 主题

<!--more-->

## 准备工作

安装并配置 Git

下载 NexT 主题，<a href="https://github.com/Simpleyyt/jekyll-theme-next" target="_blank">GitHub仓库地址</a>

```powershell
git clone https://github.com/Simpleyyt/jekyll-theme-next.git
```

如果 Git 太慢，可以设置 Git 代理

```powershell
git config --global http.https://github.com.proxy socks5://127.0.0.1:1080
```

或使用我的<a href="https://gitee.com/gotten-ln/jekyll-theme-next" target="_blank">Gitee仓库地址</a>

```powershell
git clone https://gitee.com/gotten-ln/jekyll-theme-next.git
```

---

## GitHub

创建新的 **Repositories**

**Repositories name** 处填写`username.GitHub.io`，其中 username 为用户名

创建成功后，点击 **Settings**

找到 **GitHub Pages** 选项，在 **Source** 中选择主分支`main`或`master`，文件夹选择`/(root)`，不需要选择主题，其他**默认**即可

将准备好的主题文件复制到本地仓库`/username.GitHub.io`文件夹内

将本地仓库文件全部`push`到新建的`username.GitHub.io`仓库，此时使用命令行操作比较方便

先进入本地仓库文件夹

```powershell
#进入F盘
f:
#进入文件夹
cd .../username.GitHub.io
#push
git add --all
git commit -m "first_commit"
git push
```

等待几分钟后，就可访问网页了

---

## Gitee

创建过程与 GitHub 类似，不过要注意以下几处

**仓库名称**填写`username`，如果从是 GitHub 导过来的，还要检查**路径**是不是

`https://gitee.com/username/username`

创建成功后点击**服务**，选择**Gitee Pages**

勾选**强制使用HTTPS**选项

Gitee Pages 需要**手动更新**，且两次更新间隔1分钟

---

## 参考资料

[给git访问github设置代理 - 起风了](https://www.dyxmq.cn/it/git/set-http-proxy-for-github.html)

[测试git能否连接github_littlehaes的博客-CSDN博客_git测试连接](https://blog.csdn.net/littlehaes/article/details/102082142)

[Jekyll 配置 & Next 主题教程 - 大专栏](https://www.dazhuanlan.com/2019/12/24/5e01f12f3f751/)

[Gitee Pages - Gitee.com](https://gitee.com/help/articles/4136#article-header0)

