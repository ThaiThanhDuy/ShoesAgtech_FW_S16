---
description: Push the current feature/fix branch and open a pull request whose description is built from the spec and commits. Never merges.
argument-hint: [optional: --draft]
disable-model-invocation: true
---

# Open a pull request

Options: $ARGUMENTS

1. **Pre-flight** (stop and report on any failure):
   - Current branch is not BASE (find BASE as in `/feature` step 0).
   - `git status --short` is empty. If not, ask the user whether to commit the leftovers.
   - `git log BASE..HEAD --oneline` has at least one commit.
   - `gh auth status` works. If `gh` is missing or not logged in, tell the user how to fix it (`gh auth login`) and stop.
2. **Verify** — delegate to `verifier`. Do not open a PR with failing checks; report the failures and suggest `/fix` or continuing `/feature`.
3. **Description** — find the spec (`docs/specs/<slug>.md`, slug from branch name). Write the PR body from the spec and commit list only:
   ```
   ## Goal
   <from spec, 1–3 lines>
   ## Changes
   <one bullet per task / commit>
   ## How to test
   <commands + manual steps from acceptance criteria>
   ## Out of scope / follow-ups
   <from spec>
   Spec: docs/specs/<slug>.md
   ```
   Title: conventional style, e.g. `feat: password reset via email`.
4. **Push** — `git push -u origin <branch>` (the user will be asked to approve). Never use `--force`.
5. **Open PR** — `gh pr create --base BASE --title "<title>" --body-file <tmpfile>` (add `--draft` if requested). Delete the temp file.
6. Mark the feature as "PR opened: <url>" in docs/PROGRESS.md and commit+push that one-line change only if the user agrees.
7. Report the PR URL in one line. Do not merge, approve or delete branches.
