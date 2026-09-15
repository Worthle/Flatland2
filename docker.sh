#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runtime="$(docker info --format '{{if index .Runtimes "nvidia"}}nvidia{{end}}')"

export FLATLAND_DOCKER_RUNTIME=runc
export FLATLAND_RENDERER=cpu
export FLATLAND_NVIDIA_DEVICES=void

if [[ "$runtime" == nvidia ]]; then
    probe_image=flatland2:humble-fork
    if ! docker image inspect "$probe_image" >/dev/null 2>&1; then
        probe_image=ros:humble
    fi
    # Probe Docker itself so a missing device or broken runtime falls back too.
    if devices="$(docker run --rm --network none --runtime nvidia \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility \
        --entrypoint nvidia-smi "$probe_image" -L 2>/dev/null)" && [[ "$devices" == *"GPU "* ]]; then
        export FLATLAND_DOCKER_RUNTIME=nvidia
        export FLATLAND_RENDERER=nvidia
        export FLATLAND_NVIDIA_DEVICES=all
    fi
fi

exec docker compose --project-directory "$repo_dir" -f "$repo_dir/docker-compose.yaml" "$@"
