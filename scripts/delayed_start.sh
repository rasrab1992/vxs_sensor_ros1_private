#!/bin/bash
# Usage: delayed_start.sh <delay_seconds> <command> [args...]
DELAY=$1
shift
echo "[delayed_start] Waiting ${DELAY}s before starting: $*"
sleep "$DELAY"
exec "$@"
