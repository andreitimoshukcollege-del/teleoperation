---
description: Audit Teleop.Core against the architectural invariants before committing
argument-hint: "[optional focus, e.g. allocations | clock | il2cpp]"
allowed-tools: Read, Grep, Glob, Bash(dotnet:*), Bash(git diff:*), Bash(git status:*), Bash(find:*), Bash(ls:*)
model: sonnet
---

## Uncommitted changes

!`git diff --stat`

## Task

Delegate to the `invariant-auditor` subagent, focusing on $ARGUMENTS if provided.

Report PASS/FAIL per invariant with file:line for each FAIL, ordered by consequence rather
than by checklist order. Then paste the verbatim output of `dotnet test`,
`Teleop.Eval -- verify`, and `Teleop.Eval -- audit`.

Do not fix anything. This command exists to tell the truth about the current state.
