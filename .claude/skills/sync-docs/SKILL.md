---
description: Audit docs/ARCHITECTURE.md, DECISIONS.md and all CLAUDE.md files against the current code, report drift, and fix the docs after user approval. Run every few features or after big refactors.
argument-hint: [optional: area to audit, e.g. "backend"]
disable-model-invocation: true
---

# Sync docs with code

Scope (optional): $ARGUMENTS

Docs only — never change source code in this skill.

1. Read docs/ARCHITECTURE.md module map and list every CLAUDE.md file (`git ls-files '*CLAUDE.md'`).
2. **Check drift in parallel** — one `scout` per module area (within scope). Each compares the docs for its area to the code and reports only mismatches:
   `[stale|missing|wrong] doc-file:line — what the doc says — what the code actually does (path)`
   Also report new top-level folders/modules that no doc mentions.
3. **Commands check** — `verifier` runs every command listed in root CLAUDE.md and reports any that fail or no longer exist.
4. Show the user a ≤15-line drift summary, grouped by file. Ask which to fix (default: all).
5. **Fix** — delegate to `architect`: update ARCHITECTURE.md, the affected CLAUDE.md files and DECISIONS.md (append, don't rewrite history; mark superseded decisions). Keep root CLAUDE.md under ~80 lines.
6. Commit on a branch `docs/sync-<date>` (create it from BASE as in `/feature` step 0): `docs: sync docs with code`. Do not push.
