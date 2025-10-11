# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a personal blog/portfolio site built with Jekyll using the TeXt theme. The site focuses on Reinforcement Learning and Mathematics content, with posts written primarily in Markdown with LaTeX support for mathematical expressions.

## Development Commands

### Local Development
- **Start development server**: `npm run serve` or `bundle exec jekyll serve -H 0.0.0.0`
- **Build for production**: `npm run build` (sets `JEKYLL_ENV=production`)
- **Development with theme docs config**: `npm run dev` (uses `./docs/_config.dev.yml`)

### Code Quality
- **ESLint**: `npm run eslint` (check JavaScript in `_includes/**/*.js`)
- **ESLint auto-fix**: `npm run eslint-fix`
- **Stylelint**: `npm run stylelint` (check SCSS in `_sass/**/*.scss`)
- **Stylelint auto-fix**: `npm run stylelint-fix`

### Docker Development
- **Default environment**: `npm run docker-dev:default`
- **Development environment**: `npm run docker-dev:dev`
- **Production build**: `npm run docker-prod:build`
- **Production serve**: `npm run docker-prod:serve`

### Gem Management
- **Build gem**: `npm run gem-build`
- **Publish gem**: `npm run gem-push`

## Architecture

### Core Structure

**Jekyll Configuration** (`_config.yml`):
- Site settings (title, description, URL)
- Theme: TeXt with "dark" skin and "tomorrow-night-eighties" highlight theme
- Markdown engine: kramdown with rouge syntax highlighter
- Key features enabled: MathJax (for LaTeX), Mermaid (for diagrams)
- Pagination: 8 posts per page
- Timezone: Asia/Seoul

**Content Organization**:
- `_posts/`: Blog posts in Markdown format
  - Naming convention: `YYYY-MM-DD-title.md`
  - Front matter includes: title, tags, article_header (with type: cover)
  - Posts support LaTeX math expressions via MathJax
- `_layouts/`: HTML layout templates (base, article, archive, home, page, landing)
- `_includes/`: Reusable HTML components (header, footer, scripts, article components)
- `_sass/`: SCSS stylesheets organized by purpose (common, layout, components, skins)
- `_data/`: YAML data files
  - `navigation.yml`: Site navigation structure (header menu, sidebar profile menu)
  - `locale.yml`: Internationalization strings
  - `licenses.yml`: Content license definitions

### Styling System

The theme uses a modular SCSS architecture:
- `_sass/common/`: Base styles, variables, mixins
- `_sass/layout/`: Layout-specific styles
- `_sass/components/`: Reusable component styles
- `_sass/skins/`: Theme color schemes (currently using "dark")
- `_sass/animate/`: Animation effects
- `_sass/additional/`: Additional utility styles
- `custom.scss`: Site-specific customizations

### JavaScript Architecture

JavaScript files are organized in `_includes/scripts/`:
- `lib/`: Third-party libraries (swiper, modal, lazyload, gallery, etc.)
- `components/`: UI components (sidebar, search, lightbox)
- `utils/`: Utility functions (imagesLoad, utils)
- `aside/`: Sidebar-specific scripts (toc, affix)
- Page-specific scripts: `article.js`, `home.js`, `page.js`, `archieve.js`

### Navigation

Defined in `_data/navigation.yml`:
- **Header navigation**: Archive, About pages
- **Sidebar profile navigation**: Links to author info, posts (filtered by tags like "RL"), and contact information (Email, GitHub, LinkedIn)

### Content Defaults

Posts automatically get:
- Layout: article
- Table of contents enabled
- Profile sidebar navigation
- License display disabled (site uses CC-BY-NC-4.0)
- "Edit on GitHub" disabled
- Author profile disabled
- Pageview tracking enabled

## Post Writing Guidelines

### Front Matter Structure
```yaml
---
title: Post Title
tags: Tag1 Tag2
article_header:
  type: cover
---
```

### LaTeX Support
- MathJax is enabled with auto-numbering
- Use `$...$` for inline math or standard LaTeX delimiters
- Complex expressions should be wrapped in `<p>` tags for proper rendering

### Tags
- Common tags in use: `ProbabilityTheory`, `RL` (Reinforcement Learning)
- Archive page supports tag filtering via URL: `/archive.html?tag=RL`

## Commit Conventions

This project uses conventional commits (enforced by commitlint):
- **Types**: build, chore, ci, docs, feat, fix, improvement, perf, refactor, release, revert, style, test
- **Format**: `type(scope): subject` (max 72 chars)
- Scope and subject must be lowercase
- Subject should not end with a period
- Use husky for git hooks (commit-msg validation)

## Dependencies

### Ruby/Jekyll
- Jekyll >= 3.6, < 5.0
- jekyll-paginate ~> 1.1
- jekyll-sitemap ~> 1.0
- jekyll-feed ~> 0.1
- jemoji ~> 0.8

### Node.js (Dev)
- cross-env: Environment variable management
- eslint: JavaScript linting
- stylelint: SCSS linting with SCSS-specific plugins
- commitlint: Commit message validation
- husky: Git hooks
