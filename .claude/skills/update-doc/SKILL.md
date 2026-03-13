---
name: update-docs
description: Documentation update workflow for package READMEs and docs/ markdown pages
triggers:
  - "update documentation"
  - "add docs page"
  - "write documentation"
  - "update docs"
---

# Documentation Update Workflow

The repository uses two layers of documentation:
- **Package READMEs** (`src/<package>/README.md`): Quick-reference for each package
- **Docs folder** (`docs/`): Plain markdown files for detailed or cross-cutting documentation

## Package README Template

Every package should have a `README.md`. Follow this structure:

```markdown
# Package Name (with expanded acronyms + acronym between parentheses)

One-line description (must match the <description> tag in package.xml).

## Overview

What this package does and why it exists (2-3 sentences).

## Quick Start

\`\`\`bash
ros2 launch package_name main.launch.py
\`\`\`

## Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `param_name` | `string` | `""` | What it does |

## ROS 2 Interface Summary

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `~/topic` | `std_msgs/String` | pub/sub | Description |

### Services (if any)

| Service | Type | Description |
|---------|------|-------------|
| `~/my_service` | `std_srvs/srv/SetBool` | Description |

## See Also

For detailed documentation, see the [root README](../../README.md) or the [docs folder](../../docs/).
```

**Rules:**
- The README one-line description must match the `<description>` tag in `package.xml`. Do not duplicate other `package.xml` fields (version, license, dependencies).
- This is a `ros2_control` workspace — packages expose controllers and hardware interfaces, not standalone nodes. Do not add a "Nodes" section.
- Include a "Parameters" table when the package exposes configurable parameters (controllers, hardware interface).

## Adding a Documentation Page

For detailed guides, architecture overviews, tutorials, or cross-package documentation, use the `docs/` folder.

### Step 1: Create the Page

Create a markdown file in `docs/`:

```markdown
# Page Title

## Overview

Description of the topic.

## Content

Your detailed documentation here.
```

### Step 2: Add to the Index

Edit `docs/README.md` (the index page) and add a link to your new page:

```markdown
- [Page Title](page_title.md) — Brief description
```

No build step is needed — these are plain markdown files.

## Updating Existing Documentation

When modifying a package, check and update:
1. The package `README.md` (especially Quick Start, Parameters, and interface tables)
2. Any related page in `docs/` that covers the modified functionality
3. The `<description>` tag in `package.xml` (keep it in sync with the README headline)
