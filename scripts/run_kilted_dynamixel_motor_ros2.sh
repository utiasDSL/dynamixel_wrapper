#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_DIR="$REPO_ROOT/.pixi/envs/kilted"
TARGET_LIB="$ENV_DIR/lib/librmw.so"

needs_fix=false
if [[ ! -f "$TARGET_LIB" ]]; then
  needs_fix=true
elif ! file "$TARGET_LIB" | grep -q "ELF 64-bit"; then
  needs_fix=true
fi

if [[ "$needs_fix" == "true" ]]; then
  shopt -s nullglob
  rmw_json=("$ENV_DIR"/conda-meta/ros-kilted-rmw-*.json)
  shopt -u nullglob

  if [[ ${#rmw_json[@]} -eq 0 ]]; then
    echo "ERROR: Cannot auto-repair kilted RMW: ros-kilted-rmw metadata not found in $ENV_DIR/conda-meta" >&2
    exit 1
  fi

  rmw_base="$(basename "${rmw_json[0]}" .json)"
  temp_dir="$(mktemp -d)"
  cleanup() { rm -rf "$temp_dir"; }
  trap cleanup EXIT

  rmw_pkg="$ENV_DIR/pkgs/${rmw_base}.conda"
  if [[ ! -f "$rmw_pkg" ]]; then
    rmw_url="$(python -c 'import json,sys; print(json.load(open(sys.argv[1]))["url"])' "${rmw_json[0]}")"
    rmw_pkg="$temp_dir/${rmw_base}.conda"
    if [[ -z "$rmw_url" ]]; then
      echo "ERROR: Cannot auto-repair kilted RMW: package URL missing in ${rmw_json[0]}" >&2
      exit 1
    fi
    python -c 'import sys, urllib.request; urllib.request.urlretrieve(sys.argv[1], sys.argv[2])' "$rmw_url" "$rmw_pkg"
  fi

  echo "WARNING: Detected invalid kilted librmw.so. Applying automatic repair from cached ros-kilted-rmw package."

  unzip -q -o "$rmw_pkg" -d "$temp_dir"
  payload=("$temp_dir"/pkg-*.tar.zst)
  if [[ ${#payload[@]} -eq 0 ]]; then
    echo "ERROR: Cannot auto-repair kilted RMW: no pkg-*.tar.zst payload found in $rmw_pkg" >&2
    exit 1
  fi

  tar --zstd -xOf "${payload[0]}" lib/librmw.so > "$TARGET_LIB"
  chmod 0644 "$TARGET_LIB"

  if ! file "$TARGET_LIB" | grep -q "ELF 64-bit"; then
    echo "ERROR: Auto-repair ran, but $TARGET_LIB is still not a valid ELF shared library" >&2
    exit 1
  fi

  echo "WARNING: Auto-repair applied successfully for $TARGET_LIB"
fi

exec python -m scripts.dynamixel_motor_ros2 "$@"
