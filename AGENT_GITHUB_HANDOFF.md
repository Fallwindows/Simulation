# GitHub Remote Handoff for Agents

## Repository

- Local path: `C:\Users\suyog\OneDrive\Documents\ChatGPT\Simulation`
- GitHub remote: `git@github.com:Fallwindows/Simulation.git`
- Current branch: `master`
- Upstream: `origin/master`
- GitHub repository: `Fallwindows/Simulation`

## Authentication on this machine

This repository uses a dedicated, repository-scoped GitHub deploy key with read/write access.

- Private key path: `C:\Users\suyog\.ssh\codex_simulation_deploy_ed25519`
- Public key path: `C:\Users\suyog\.ssh\codex_simulation_deploy_ed25519.pub`
- Fingerprint: `SHA256:96JFqEIyCcO94kr0hpg6HvdtHD5a9mVsmfoQt9VTgnY`
- SSH executable: `C:\Windows\System32\OpenSSH\ssh.exe`
- Repository-local `core.sshCommand` is configured to use the dedicated key with `IdentitiesOnly=yes`.

The private key must never be committed, pasted into chat, uploaded, or copied to another agent or machine. The public key is registered in GitHub under the repository's Deploy keys as `Codex Simulation deploy key` with `Read/write` access.

## Verify access

From PowerShell:

```powershell
$repo = "C:\Users\suyog\OneDrive\Documents\ChatGPT\Simulation"
git -C $repo remote -v
git -C $repo ls-remote origin
git -C $repo status --short --branch
```

A successful `git ls-remote origin` confirms repository access. A successful push should report `master -> master` or the selected branch.

## Normal workflow

```powershell
$repo = "C:\Users\suyog\OneDrive\Documents\ChatGPT\Simulation"
git -C $repo fetch origin
git -C $repo status --short --branch
# make scoped changes
git -C $repo add <intended-files>
git -C $repo diff --cached --check
git -C $repo commit -m "<scoped message>"
git -C $repo push origin master
git -C $repo ls-remote origin refs/heads/master
```

Do not use `git add .` blindly, force-push, rewrite published history, or commit secrets/generated simulation data.

## New machine or new agent

This deploy key is local to the current machine. An agent running elsewhere cannot use it. The user must either:

1. provide that agent an independently authorized repository credential, preferably a new repository-scoped deploy key; or
2. configure an approved SSH key/credential on that machine and register its public key in GitHub.

Do not copy this machine's private key as a shortcut.

## Current project state

Gate 0 discovery is committed and pushed. The current machine does not have Isaac Sim, ROS 2, RTAB-Map, or WSL installed; it is Windows 10 with an RTX 4070 Ti. The implementation prompt requires stopping at Gate 0 until a supported simulation environment is available.

The repository's implementation journal is in `IMPLEMENTATION_JOURNAL.md`.
