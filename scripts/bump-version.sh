#!/usr/bin/env bash
# Bumps FIRMWARE_VERSION in include/core/config.h. Run this as part of
# cutting a release, alongside writing RELEASE_NOTES.md -- before tagging,
# so the tag lands on a commit that already carries the matching version.
#
# Usage: scripts/bump-version.sh <new-version>
#   scripts/bump-version.sh 1.1.0
set -euo pipefail

if [ $# -ne 1 ]; then
  echo "usage: $0 <new-version>   e.g. $0 1.1.0" >&2
  exit 1
fi

NEW_VERSION="$1"
if ! [[ "$NEW_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "error: version must look like X.Y.Z (got '$NEW_VERSION')" >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_H="$REPO_ROOT/include/core/config.h"

CURRENT=$(grep 'FIRMWARE_VERSION\[\]' "$CONFIG_H" | grep -oE '[0-9]+\.[0-9]+\.[0-9]+')
if [ -z "$CURRENT" ]; then
  echo "error: couldn't find FIRMWARE_VERSION in $CONFIG_H" >&2
  exit 1
fi

if [ "$CURRENT" = "$NEW_VERSION" ]; then
  echo "FIRMWARE_VERSION is already $NEW_VERSION -- nothing to do." >&2
  exit 1
fi

# -i.bak rather than bare -i: BSD sed (macOS) requires an argument for -i
# (even an empty one, with its own quoting quirks); GNU sed (Linux/CI)
# accepts a backup suffix the same way -- this spelling works on both
# without an OS check.
sed -i.bak "s/FIRMWARE_VERSION\[\] = \"$CURRENT\"/FIRMWARE_VERSION[] = \"$NEW_VERSION\"/" "$CONFIG_H"
rm -f "$CONFIG_H.bak"

echo "FIRMWARE_VERSION: $CURRENT -> $NEW_VERSION ($CONFIG_H)"
echo
echo "Next steps:"
echo "  1. Update RELEASE_NOTES.md for v$NEW_VERSION"
echo "  2. Commit both (smart-commit)"
echo "  3. Tag:  git tag -s v$NEW_VERSION -F <release notes file>"
echo "  4. Push: git push origin main && git push origin v$NEW_VERSION"
