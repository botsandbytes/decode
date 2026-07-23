---
name: ai-rulez
description: "Use AI-Rulez correctly in user projects, including CLI, MCP, configuration, and generation workflows"
# Content-Hash: blake3:63877945f328aa0fad0e1afc85afd6231fe68c7c28c78f053ec3df2485183045
# Source-Hash: blake3:103384c5979634bfc4353e1b9db1d77afb7f544de5eb32c83c64cc353d7bd729
---

# AI-Rulez

Use this skill when working in a project that is managed by AI-Rulez.

## Responsibilities

- Detect whether the project uses AI-Rulez (.ai-rulez/) or a legacy V2 config.
- Edit source files in .ai-rulez/ instead of patching generated assistant files directly
- Prefer the AI-Rulez MCP server for safe reads and CRUD operations when it is available
- Use the CLI to validate, generate, and inspect configuration changes
- Keep generated outputs in sync with configuration changes

## Workflow

1. Check for .ai-rulez/config.yaml (or config.toml), .ai-rulez/skills/, domain folders.
2. If MCP is configured in the config, prefer the MCP server for reading and modifying AI-Rulez content.
3. Update the relevant source files under .ai-rulez/: rules, context, skills, agents, domains, or config.
4. Run `bunx ai-rulez validate` when changing configuration structure.
5. Run `bunx ai-rulez generate` after source changes so assistant-specific outputs stay current.
6. If MCP is available, start it with `bunx ai-rulez mcp` (or the repo’s helper) and use it for CRUD instead of manual edits when possible.

## Core Commands

- `bunx ai-rulez init` — scaffold .ai-rulez/ for a project.
- `bunx ai-rulez add|remove|list rule|context|skill|agent` — manage content files.
- `bunx ai-rulez validate` — ensure config and tree structure are sound.
- `bunx ai-rulez generate [--profile <name>]` — render tool presets after edits.
- `bunx ai-rulez migrate` — convert legacy ai-rulez.yaml to .ai-rulez/.

## Guidelines

- Treat .ai-rulez/ as the source of truth.
- Generated files such as AGENTS.md, CLAUDE.md, or .cursor/ outputs should only change via generation.
- Use `bunx ai-rulez init` to bootstrap, generate to render outputs, validate to check structure, and migrate for format migration.
- Remember that root content is always included, while domains are controlled by profiles.
- MCP can expose read, CRUD, generate, and validate operations for assistants.
- When changing presets, profiles, or domains in config.yaml, rerun validate then generate so downstream files stay in sync.
