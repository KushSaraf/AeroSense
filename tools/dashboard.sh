#!/usr/bin/env bash
# The web dashboard in one command: bridge + frontend + simulation in tmux, browser opened.
# Everything from a previous run is destroyed first. Options: see tools/demo.sh --help.
exec "$(dirname "${BASH_SOURCE[0]}")/demo.sh" --dashboard "$@"
