---
description: One-time setup for a brand-new empty project. Picks the stack with the user, scaffolds the project, and fills in CLAUDE.md commands and structure.
argument-hint: <short description of the product>
disable-model-invocation: true
---

# Kickoff a new project

Product idea: $ARGUMENTS

1. Ask the user (one AskUserQuestion call, max 4 questions): target platform, preferred language/framework (offer a recommended default), database need, deployment target. Skip questions already answered in the idea.
2. Delegate to `architect`: write `docs/specs/000-foundation.md` covering stack choice + reasons, folder structure, test/lint tooling, and the first 3–5 setup tasks. Record stack decisions in docs/DECISIONS.md.
3. Show the user a ≤10-line summary of the foundation spec and WAIT for approval.
4. For each task in the foundation spec: delegate to `implementer`, then `verifier`. On failure, send only the verifier's error lines back to `implementer`.
5. Fill in the Stack, Commands and Structure sections of CLAUDE.md with the real values (keep it short).
6. `git init` (if needed) and make the first commit: `chore: project foundation`.
7. Tell the user the project is ready, and that new features start with `/feature <name>`.
