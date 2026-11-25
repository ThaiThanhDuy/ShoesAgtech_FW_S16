---
description: Full pipeline for adding one feature - new branch, scout, spec by architect, user approval, implement task by task, verify, review, commit.
argument-hint: <feature description>
disable-model-invocation: true
---

# Feature pipeline

Feature: $ARGUMENTS

You (main session) are the orchestrator. Keep your own context small: do not read source files yourself — delegate. Pass paths, not contents.

0. **Branch** — never build a feature on the base branch. Do this before any file is written:
   - Find the base branch: `git symbolic-ref --short refs/remotes/origin/HEAD` (strip `origin/`). If that fails, use `main` if it exists, else `master`; if neither exists, ask the user. Call it BASE below.
   - Run `git status --short` and `git branch --show-current`.
   - Uncommitted changes? STOP and ask the user what to do with them. Do not stash, discard or commit them on your own.
   - Empty output from `git branch --show-current` (detached HEAD)? STOP and ask the user.
   - On BASE:
     - If a remote exists (`git remote` is not empty): `git pull --ff-only`. If that fails, STOP and report.
     - Pick a short kebab-case slug from the feature (the spec uses the same slug). Prefix `feat/`, or `fix/` for a bug fix.
     - If that branch already exists (`git branch --list <name>`), ask the user: continue on it, or use a new name.
     - `git switch -c <prefix>/<slug>`.
   - Already on another branch: ask the user whether to continue on it or to start a new branch from BASE.
1. **Scout** — delegate to `scout`: "Find everything relevant to: $ARGUMENTS. List what exists and what is missing."
2. **Spec** — delegate to `architect` with the scout report. It writes `docs/specs/<slug>.md`.
   - Trivial change (≤2 files, no data/API/security impact)? Skip the architect; write a 5-line spec yourself.
3. **Approval** — show the user: goal, key decisions, task list (≤12 lines). STOP and wait. Apply any changes the user requests to the spec. Then commit the spec: `docs: spec for <slug>`.
4. **Build loop** — for each task in order (tasks with no dependency between them and no shared files may run in parallel):
   a. `implementer` with spec path + task ID.
   b. `verifier`.
   c. Verifier fails → send ONLY its error lines to `implementer` (max 2 retries).
   d. Still failing, or implementer reports `blocked` → `architect` diagnoses, updates the spec, then retry once.
   e. Update docs/PROGRESS.md, then commit the task on the feature branch: `feat(<slug>): T<n> <short task title>` (stage only the files from the handoff FILES list plus the spec/PROGRESS).
4.5 **Test hardening** — non-trivial feature: `test-designer` with the spec path. Commit its tests: `test(<slug>): edge cases`. Bugs it reports → `implementer` fixes them (the failing tests are the reproduction; remove any skip/xfail markers), then `verifier`, then commit.
5. **Review** — non-trivial feature: `reviewer` with the spec path and BASE; it reviews `git diff BASE...HEAD`. Send blockers/majors to `implementer`, then `verifier` again, then commit `fix(<slug>): review fixes`.
   Security-sensitive feature (auth, payment, user data, file upload, new external integration): also run `security-auditor` scoped to the spec and the changed files; treat critical/high findings like review blockers.
6. **Finish** — check `git status --short` is clean (commit leftovers that belong to the feature; ask about anything else). Do not push, merge or delete any branch; the user decides that. Summarize to the user in ≤5 lines (include the branch name and number of commits) and suggest `/clear` before the next feature.
