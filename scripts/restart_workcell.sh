#!/usr/bin/env bash

# Keep shell state isolated when an operator invokes `. restart_workcell.sh`.
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  if bash "${BASH_SOURCE[0]}" "$@"; then
    return 0
  else
    return $?
  fi
fi

set -Eeuo pipefail

WS="${WS:-/home/b1/panthera_workcell_ws}"
bash "$WS/scripts/stop_workcell.sh" --for-restart
exec bash "$WS/scripts/start_workcell.sh"
