#!/usr/bin/env bash
# Deterministic SO-101 pick-and-place (no LLM). DRIVER=skill wrapper.
exec env DRIVER=skill bash "$(dirname "$0")/run-so101-agent.sh" "$@"
