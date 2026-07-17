#!/usr/bin/env bash
set -eo pipefail

WS="${WS:-$HOME/panthera_workcell_ws}"
BACKUP_ROOT="${BACKUP_ROOT:-$HOME/panthera_workcell_ws_backups}"
STAMP="$(date +%Y%m%d_%H%M%S)"
ARCHIVE="$BACKUP_ROOT/panthera_workcell_ws_source_docs_${STAMP}.tar.gz"
MANIFEST="$BACKUP_ROOT/panthera_workcell_ws_source_docs_${STAMP}.manifest.txt"

mkdir -p "$BACKUP_ROOT"
cd "$WS"

tar \
  --exclude='./build' \
  --exclude='./install' \
  --exclude='./log' \
  --exclude='./Log' \
  --exclude='./validation_logs' \
  --exclude='./.runtime' \
  --exclude='./.git' \
  --exclude='./.last_*' \
  --exclude='./.runtime_last_build_log' \
  -czf "$ARCHIVE" \
  ./README.md ./docs ./scripts ./src

tar -tzf "$ARCHIVE" > "$MANIFEST"

echo "archive=$ARCHIVE"
echo "manifest=$MANIFEST"
echo "size_bytes=$(stat -c%s "$ARCHIVE")"
