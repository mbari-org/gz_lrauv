#!/usr/bin/env bash

set -euo pipefail

docker_args=(
	build
	--target lrauv
	-t lrauv:harmonic-frames
	-f tools/setup/Dockerfile
)

if [[ "${1:-}" == "--no-cache" ]]; then
	docker_args+=(--no-cache)
fi

docker "${docker_args[@]}" .
