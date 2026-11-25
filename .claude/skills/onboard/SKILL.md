---
description: One-time onboarding for an EXISTING codebase. Maps the system with parallel Haiku scouts, has the architect synthesize docs/ARCHITECTURE.md, fills CLAUDE.md, and adds per-module CLAUDE.md notes.
argument-hint: [optional focus, e.g. "backend only"]
disable-model-invocation: true
---

# Onboard an existing codebase

Focus (optional): $ARGUMENTS

You (main session) orchestrate. Do NOT read source files yourself.

1. **Top-level survey** — run `git ls-files | head -300`, read the README, package/build manifests and CI config only. List the top-level modules/areas (max ~8).

2. **Parallel scouting** — launch one `scout` per area IN PARALLEL. Ask each for:
   - purpose of the area (1–2 lines), entry points, key files (max 15)
   - main data models / types, external services, env vars used
   - how it's tested (test folder, framework)
   - conventions noticed (naming, error handling, patterns), and smells/risks
   - dependencies on other areas
   Each scout writes its report to `docs/onboarding/<area>.md` (≤60 lines) and returns only the handoff block.

3. **Commands check** — delegate to `verifier`: discover the real install/dev/test/lint/type-check/build commands from manifests & CI, run test + lint once, and report which work. Do not fix anything.

4. **Synthesis** — delegate to `architect`: read the `docs/onboarding/*.md` reports + verifier result, then write:
   - `docs/ARCHITECTURE.md` (≤150 lines): system overview, module map with paths, data flow for 2–3 core use cases, external integrations, known risks / tech debt.
   - `docs/DECISIONS.md`: decisions that are visible in the code (stack, auth approach, DB, patterns), marked "(inferred)".
   - A short `CLAUDE.md` inside each major module folder (≤20 lines): what lives here, local conventions, gotchas.

5. **Root CLAUDE.md** — fill Stack, Commands (only commands the verifier confirmed), Structure (1 line per area + pointer to docs/ARCHITECTURE.md), and Conventions observed. Keep it under ~80 lines; details belong in ARCHITECTURE.md.

6. **User review** — show the user a ≤15-line summary: module map, confirmed commands, top 5 risks, any inferred decisions that need confirmation. Ask them to correct anything wrong.

7. Apply corrections, delete `docs/onboarding/` (or keep if the user prefers), commit: `docs: onboard codebase for Claude`.
