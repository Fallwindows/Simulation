# Current execution authority

The owner's latest direct instruction and the current `/goal` define the task.
Supplementary project documents apply only when that goal explicitly references
them. Older goals, video/storyboard requirements, role workflows, review gates,
and model-routing rules are historical unless invoked by the current goal.

- No repository instruction requires a particular model or reasoning effort.
  Continue with the active runtime; a model mismatch is not a reason to stop.
- The main agent owns execution and may inspect, implement, test, delegate,
  review, integrate, commit, and push when the current owner instruction
  authorizes those actions. Subagents and independent review are optional unless
  the current goal requires them.
- An explicit request to push or checkpoint everything means preserve the
  appropriate current project work, mark unfinished work WIP, commit it, and
  push it promptly. Do not wait for tests, reviewers, subagents, or goal
  completion. Do not force-push unless explicitly requested.
- Preserve unrelated work. Do not commit secrets, credentials, or clearly
  disposable caches and generated output. State any intentional exclusions.
- Concurrent writers use separate worktrees/branches. Serialize heavy Isaac/GPU
  workloads. Report test and runtime evidence accurately; never claim a check
  passed when it did not run or failed.

Use the current goal to decide what is complete. This file does not start a task
or impose the legacy simulation-video workflow by itself.