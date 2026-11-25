# Project: <TÊN DỰ ÁN>

<!-- Keep this file SHORT (< 80 lines). It is loaded into every agent on every turn. -->

## Language
- Reply to the user in Vietnamese. Code, comments, commits, specs: English.

## Stack
<!-- /kickoff fills this in -->
- Language/framework: TBD
- Package manager: TBD
- Database: TBD

## Commands
<!-- /kickoff fills this in. Agents run ONLY these, never guess. -->
- Install: TBD
- Dev: TBD
- Test: TBD
- Lint/format: TBD
- Type-check: TBD
- Build: TBD

## Structure
<!-- /kickoff fills this in: 1 line per top-level folder -->

## Conventions
- Small focused files (< 300 lines). One responsibility per module.
- No new dependency without it being listed in the spec.
- Every feature ships with tests.

## Team (subagents) — how work flows
- `scout` (Haiku): find code, read docs/logs → returns a short map. Use BEFORE reading many files yourself.
- `architect` (Opus): writes specs in `docs/specs/`. Only for new features, design changes, or bugs Sonnet failed twice.
- `implementer` (Sonnet): implements ONE spec task at a time.
- `verifier` (Haiku): runs lint/type-check/test/build, reports failures only.
- `reviewer` (Opus): reviews the diff against the spec before merge. Skip for trivial changes.
- `test-designer` (Sonnet): test plan + integration/e2e/edge-case tests for non-trivial features. Writes tests only.
- `security-auditor` (Opus): whole-system or feature-scoped security audit → `docs/security/`. Never edits code.

## Workflows (user-invoked skills)
/kickoff · /onboard · /feature · /fix · /continue · /pr · /test-plan · /audit · /sync-docs · /release
Never push, tag, merge or delete branches unless running /pr or /release with user approval.

## Shared memory (single source of truth)
- `docs/ARCHITECTURE.md` — system map. Read it (not the whole repo) to orient before scouting or designing.
- `<module>/CLAUDE.md` — local conventions for that folder.
- `docs/specs/<feature>.md` — the contract for a feature. Agents follow it, never improvise beyond it.
- `docs/DECISIONS.md` — one line per architectural decision. Read before designing.
- `docs/PROGRESS.md` — current feature, task status, blockers. Update after each task.

## Handoff format (every subagent reply ends with this, nothing longer)
```
STATUS: done | blocked | failed
FILES: <changed or relevant files>
RESULT: <≤3 lines>
NEXT: <what should happen next, ≤1 line>
```

## Token rules
- Pass file paths, not file contents, between agents.
- Delegate broad searches to `scout`; don't read whole directories in the main thread.
- After a feature is merged: suggest `/clear` to the user.
