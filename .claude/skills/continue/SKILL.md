---
description: Resume work after /clear or a new session. Rebuilds context cheaply from PROGRESS.md, the spec and git history, then proposes the next step.
argument-hint: [optional: spec slug to resume]
disable-model-invocation: true
---

# Continue where we left off

Target (optional): $ARGUMENTS

Rebuild context from files — do NOT re-scan the codebase.

1. Run:
   - `git branch --show-current`
   - `git status --short`
   - base branch: `git symbolic-ref --short refs/remotes/origin/HEAD` (strip `origin/`; fallback `main`, then `master`) = BASE
   - `git log BASE..HEAD --oneline`
2. Read `docs/PROGRESS.md`. Determine the current spec: $ARGUMENTS if given, else the "Current feature" entry, else a spec whose slug matches the branch name.
3. Read ONLY that spec's Goal, Tasks table and "Notes from implementation / review". Do not open source files.
4. Reconcile: a task is done if PROGRESS says so OR a commit `feat(<slug>): T<n>` / `fix(<slug>)` exists. If they disagree, trust git and fix PROGRESS.md.
5. Report to the user in ≤6 lines:
   - branch · feature · spec path
   - done tasks / remaining tasks
   - uncommitted changes (if any) and blockers
   - proposed next step (e.g. "implementer on T3", "review", "/pr")
6. Ask whether to proceed. On yes, continue with the matching step of the `/feature` (or `/fix`) pipeline, using the same agents and rules.

If nothing is in progress: say so in one line and suggest `/feature` or `/fix`.
