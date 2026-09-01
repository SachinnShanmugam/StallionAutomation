#!/usr/bin/env bash
# Always run from Stallion repo root so 'swarm' package is on path
REPO=/mnt/c/Users/SACHIN/Stallion
cd "$REPO"
PYTHONPATH="$REPO" python3 swarm/verify_imports.py
