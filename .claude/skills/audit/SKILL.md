---
description: Full security audit of the repo or an area using the security-auditor agent, then triage findings with the user and turn the ones to fix into /fix tasks.
argument-hint: [optional scope: area path, feature slug, or "deps" / "secrets" only]
disable-model-invocation: true
---

# Security audit

Scope: $ARGUMENTS (empty = whole repo)

1. If the repo is large and scope is empty: read docs/ARCHITECTURE.md and propose splitting into ≤4 areas; run one `security-auditor` per area IN PARALLEL, each writing `docs/security/audit-<date>-<area>.md`. Otherwise run a single `security-auditor` with the scope.
2. Collect only the handoff blocks. Do not read the full reports into your context; read just the findings tables if you need to merge.
3. Show the user (≤15 lines): counts by severity, every critical/high finding as one line (`ID path:line — issue`), and what was not checked.
4. Ask which findings to fix now (default: all critical + high).
5. For each chosen finding: suggest running `/fix <ID>: <issue> at <path:line>` — or, if the user says "fix them all", run the `/fix` pipeline for each, one branch per finding (or one `fix/security-<date>` branch if the user prefers).
6. Commit the report(s) on a branch `docs/security-audit-<date>` (create from BASE as in `/feature` step 0): `docs: security audit <date>`. Do not push.

Never paste secret values into chat, reports, commits or PR descriptions.
