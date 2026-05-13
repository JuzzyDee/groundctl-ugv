# Contributing

This is currently a one-person project, but it's developed with the discipline of a project that expects collaborators. Clean git history, clean PRs, and clean commits matter — they signal professionalism to anyone reading the repo for the first time, and they make future-you's life much easier when you're tracing a regression six months from now.

## Commit conventions

[Conventional Commits](https://www.conventionalcommits.org/), Angular-flavoured.

Format:

```
<type>(<scope>): <short summary>

[optional body]

[optional footer]
```

**Types**:

- `feat` — new feature
- `fix` — bug fix
- `docs` — documentation only
- `chore` — build, dependencies, repo maintenance
- `refactor` — code change that neither fixes a bug nor adds a feature
- `perf` — performance improvement
- `test` — adding or correcting tests
- `ci` — CI configuration

**Scopes** (this project's vocabulary):

- `bridge` — `ros2_bridge.py` and HTTP↔ROS2 surface
- `camera` — `camera_owner.py`, v4l2, USB recovery
- `intent` — intent stack, individual intent classes
- `perception` — YOLO, OAK-D, LiDAR, fusion
- `motor` — motor control, ESP32 firmware, twist_mux
- `ui` — operator console (ClaudeBot)
- `firmware` — ESP32-side changes
- `docs` — when scope isn't clear and the commit is documentation
- `infra` — systemd units, bring-up scripts, deployment

**Examples**:

```
feat(camera): add USBDEVFS_RESET watchdog for endpoint stalls
fix(intent): gate drive_distance heading-hold outliers above 30°
docs(architecture): three-layer perception → intent → motor diagram
refactor(bridge): hoist track_width to module constant
chore(deps): bump waitress to 3.0.1
```

**Discovery notes**: when a commit captures a finding rather than just a change, include a `Discovery:` line in the commit body:

```
fix(camera): USB endpoint wedge requires bus-reset, not v4l2-ctl restart

Discovery: cheap UVC cameras can enter a state where v4l2 open() and
format negotiation both succeed but bulk endpoint transfers never start.
Killing and respawning v4l2-ctl doesn't recover it; only USBDEVFS_RESET
on the underlying USB device file releases the endpoint. Confirmed
2026-05-12 with 300-frame clean stream after manual ioctl.
```

## Branch naming

`<type>/<short-description>`, lowercase, hyphen-separated.

```
feat/usb-reset-watchdog
fix/heading-outlier-gate
docs/architecture-overview
refactor/track-width-constant
```

`main` is always deployable. No direct commits to `main` — always via PR.

## Pull request flow

Trunk-based with squash-merge.

1. Branch from `main`
2. Atomic commits — **one concern per commit**. If you find yourself writing "and also" in a commit message, split it.
3. Open a PR using the template (`.github/PULL_REQUEST_TEMPLATE.md`)
4. Squash-merge to `main`. The squash commit message follows the same Conventional Commits format as individual commits.

Self-PRs are still PRs. The discipline of writing the *what*, *why*, and *test plan* surfaces things you'd otherwise miss, and produces free paper-methods-section material later.

## Code style

- Python: Black formatting, 100-char line limit, type hints where they add clarity (not as a religion)
- Markdown: standard CommonMark; tables OK
- Comments: explain *why*, not *what*. The code already says what.
- File length: prefer functions and modules under what you can hold in working memory. If a file is over 600 lines, consider splitting.

## What not to commit

- API keys, tokens, anything in `.env` files
- Generated TensorRT engines (compile on target hardware, never commit)
- Rosbag captures (use Git LFS if needed, or external storage)
- `__pycache__`, `.DS_Store`, IDE configs
- Resin printer / CAD source files larger than ~10 MB (use Git LFS or link externally)

See `.gitignore` for the full list.

## Issue tracking

Issues and feature work tracked in Linear (`Claude's Rover` team). The repo's `main` is the source of truth for *what exists*; Linear is the source of truth for *what's planned*.

When a Linear issue is closed by a commit, reference it in the commit body:

```
feat(camera): add USBDEVFS_RESET watchdog

Closes CLA-80.
```
