---
description: Strengthen tests for a feature spec or a code area with the test-designer agent - writes a test plan, adds missing integration/e2e/edge-case tests, reports bugs it uncovers.
argument-hint: <spec slug | path to area>
disable-model-invocation: true
---

# Test plan

Target: $ARGUMENTS

1. Resolve the target: if it matches `docs/specs/<slug>.md`, use that spec; otherwise treat it as a code path. If it's a path without a spec, first delegate to `scout` to summarize the area's public behavior and existing tests (≤20 lines).
2. If on BASE, create a branch `test/<slug>` (rules as in `/feature` step 0). If on a feature branch, stay on it.
3. Delegate to `test-designer` with the spec path (or the scout summary + path).
4. Delegate to `verifier` to run the full suite.
5. Report to the user (≤6 lines): tests added by level, coverage of acceptance criteria, bugs found (`test name — expected vs actual`).
6. Commit the tests: `test(<slug>): strengthen coverage`. For each bug found, suggest `/fix <test name>: <expected vs actual>` — the failing test is already the reproduction.
