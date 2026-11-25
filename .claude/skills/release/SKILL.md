---
description: Prepare a versioned release on the base branch - pick the semver bump from commits, update CHANGELOG.md and the version file, create an annotated tag. Push only after user approval.
argument-hint: [major|minor|patch] (optional; inferred from commits if omitted)
disable-model-invocation: true
---

# Release

Bump (optional): $ARGUMENTS

1. **Pre-flight** (stop on failure): on BASE (find as in `/feature` step 0); `git status --short` empty; if a remote exists, `git pull --ff-only`.
2. **Verify** — `verifier` must pass fully. Do not release with failing checks.
   **Security gate** — if no `docs/security/audit-*.md` exists since the last tag, ask the user whether to run `/audit deps` (dependency-only, cheaper) or a full `/audit` before releasing. Any open critical finding blocks the release unless the user explicitly accepts the risk.
3. **Collect** — last tag: `git describe --tags --abbrev=0` (none → this is the first release, start at `0.1.0` unless the version file says otherwise). Commits: `git log <lastTag>..HEAD --oneline`.
4. **Version** — use $ARGUMENTS if given. Otherwise: any `BREAKING CHANGE` or `!:` → major; any `feat` → minor; else patch. Show the user: current → new version and why. Wait for confirmation.
5. **Changelog** — write it yourself from the commit list only (no source reading). Prepend to CHANGELOG.md (create if missing):
   ```
   ## [x.y.z] - YYYY-MM-DD
   ### Added   (feat)
   ### Fixed   (fix)
   ### Changed (refactor/perf/other user-visible)
   ```
   Skip chore/docs/test commits unless user-visible.
6. **Bump version** in the project's version file (package.json, pyproject.toml, Cargo.toml, etc. — see CLAUDE.md Stack). If unsure which file, ask.
7. Commit `chore(release): vX.Y.Z`, then `git tag -a vX.Y.Z -m "vX.Y.Z"`.
8. Ask the user before pushing. On yes: `git push` and `git push origin vX.Y.Z` (each will ask for approval). Optionally `gh release create vX.Y.Z --notes-file <section of changelog>` if the user wants a GitHub release.
