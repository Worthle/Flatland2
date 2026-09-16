# Publish the Flatland 2 branch

The local `flatland2` branch is based on Avidbots' `ros2-humble` history. Its
`upstream` remote points to <https://github.com/avidbots/flatland.git>. No personal
remote is assumed, and preparing this checkout does not publish anything.

## 1. Create a GitHub fork

Open <https://github.com/avidbots/flatland/fork>, choose your own account, and use
`flatland` or `Flatland2` as the fork's repository name. GitHub's fork operation
creates the visible relationship to Avidbots. Creating an unrelated empty
repository and adding an `upstream` remote locally does not create that GitHub
fork relationship. See [GitHub's explanation of forks](https://docs.github.com/en/pull-requests/reference/forks).

Copying only the default branch when creating the fork is sufficient: the local
checkout already contains the Humble history and will push the commits needed
by `flatland2`.

## 2. Review and push the prepared branch

Run these commands from this checkout. Set `FLATLAND_FORK_URL` to the clone URL
of the fork you just created; the value below is a placeholder.

```bash
git branch --show-current
git log --oneline upstream/ros2-humble..flatland2
git diff --stat upstream/ros2-humble...flatland2
git status --short

FLATLAND_FORK_URL='https://github.com/YOUR_ACCOUNT/YOUR_FORK.git'
git remote add origin "$FLATLAND_FORK_URL"
git push -u origin flatland2
```

Run `git remote add` once. If `origin` already exists, check `git remote -v` and
use `git remote set-url origin "$FLATLAND_FORK_URL"` only if its URL needs changing.
Use normal GitHub HTTPS or SSH authentication. No force push is needed.

On GitHub, select `flatland2` and share its branch URL. Optionally make it your
fork's default branch so visitors see Flatland 2 immediately. A suitable repository
description is: "Community ROS 2 Humble extension of Avidbots Flatland with
vehicle plugins, mock sensors and RViz2 visualization."

## 3. Keep working on the fork

Make future Flatland edits in this checkout, then commit and push to `origin`.
The original combined workspace is a separate copy, not a linked worktree;
changes there do not synchronize automatically. This checkout uses the Docker
image tag `flatland2:humble-core`, so rebuilding it does not replace the original
workspace's `flatland2:humble` image. The companion examples use `flatland2:humble-examples`. Their Compose project
uses host ROS networking; stop another simulator before running it, or assign
different `ROS_DOMAIN_ID` values.

```bash
git fetch upstream
git log --oneline flatland2..upstream/ros2-humble
```

Review upstream changes before merging them into your branch. Use `origin` for
your fork and `upstream` for Avidbots. Keep the separate physics engine in its
own development repository.

## Optional upstream contribution

Publishing a fork does not require an upstream pull request. If you want Avidbots
to adopt changes, see [UPSTREAM.md](UPSTREAM.md) and propose a focused contribution
against a target branch agreed with their maintainers. Only those maintainers
can decide whether the upstream project should host a `flatland2` branch.

## Publish examples separately

The sibling `flatland_examples/` checkout is a separate project, with one ROS
package of the same name. Publish it independently when ready; do not place it
inside this core repository. Its Dockerfile extends a core image and its native
build uses the core packages from the same colcon workspace.

The READMEs currently link between the local sibling folders `Flatland2-share`
and `flatland_examples`. After choosing the public repository locations, replace
those sibling links with the actual repository URLs in both READMEs and the
examples attribution page. For remote builds, publish a matching core image and
set `FLATLAND_CORE_IMAGE` to its immutable tag or digest. No example repository
or container registry URL is assumed here.
