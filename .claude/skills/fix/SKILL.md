---
description: Lightweight bug-fix pipeline - fix branch, locate with scout, reproduce with a failing test first, fix, verify, commit. Escalates to architect only if needed.
argument-hint: <bug description, error message or issue link>
disable-model-invocation: true
---

# Bug-fix pipeline

Bug: $ARGUMENTS

You (main session) orchestrate. Do not read source files yourself — delegate. Pass paths, not contents.

0. **Branch** — same rules as step 0 of `/feature` (find BASE, stop on uncommitted changes or detached HEAD, pull only if a remote exists), but name the branch `fix/<slug>`.
1. **Locate** — delegate to `scout`: "Locate the code most likely responsible for: $ARGUMENTS. Include the related tests and where the error message (if any) is produced." If the user gave a stack trace, pass only its top frames.
2. **Reproduce first** — delegate to `implementer`:
   "Write ONE automated test that reproduces this bug: $ARGUMENTS. Files from scout: <paths>. Run it and confirm it FAILS for the reason described. Do not fix the bug yet."
   - Can't reproduce? STOP and ask the user for steps, input data or logs. Do not guess a fix.
   - Commit the test: `test(<slug>): reproduce bug`.
3. **Fix** — delegate to `implementer`: "Make the reproduction test pass with the smallest correct change. Do not refactor unrelated code."
4. **Verify** — `verifier` runs the full suite (not only the new test).
   - Fails → send ONLY its error lines to `implementer` (max 2 retries).
   - Still failing, or the root cause looks architectural (data model, concurrency, security) → `architect` diagnoses and writes a short note in `docs/specs/<slug>.md`; then implementer retries once.
5. **Commit** — `fix(<slug>): <short description>`. If the bug touches auth/payment/user data, suggest the user run `/security-review`.
6. **Report** (≤5 lines): root cause in one sentence, files changed, test name that guards it, branch name. Do not push. Suggest `/pr` to open a pull request.
