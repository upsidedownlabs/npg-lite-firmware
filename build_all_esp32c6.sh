#!/bin/bash

# 🧰 CONFIG
BOARD_FQBN="esp32:esp32:esp32c6"
BIN_DIR="firmware_bins"
LIB_DIR="$PWD/libraries"

# 🧹 Clean previous build
rm -rf "$BIN_DIR"
mkdir -p "$BIN_DIR"

# 📦 Ensure libraries dir exists
mkdir -p "$LIB_DIR"

# 📌 Set library directory for Arduino CLI
arduino-cli config set library.enable_unsafe_install true
arduino-cli config set directories.user "$PWD"

# 🔄 Compile each .ino file
for sketch in $(find . -type f -name "*.ino"); do
  sketch_dir=$(dirname "$sketch")
  sketch_name=$(basename "$sketch" .ino)
  build_path="$BIN_DIR/$sketch_name"

  echo "🚧 Compiling $sketch_name"

  # 📚 Auto-install libraries by scanning includes
  includes=$(grep -hoP '#include\s+[<"]\K[^">]+' "$sketch" | sort -u)

  for lib in $includes; do
    # Only try to install libraries, skip standard headers
    if [[ "$lib" == *".h" || "$lib" == *".hpp" ]]; then
      base_lib=$(basename "$lib" .h | sed 's/\.hpp$//')
      echo "📦 Checking library: $base_lib"
      arduino-cli lib install "$base_lib" || true
    fi
  done

  # 🧱 Compile
  arduino-cli compile \
    --fqbn "$BOARD_FQBN" \
    --libraries "$LIB_DIR" \
    --output-dir "$build_path" \
    --build-path "$build_path" \
    --warnings all \
    --verbose \
    "$sketch_dir"

  echo "✅ Compiled: $build_path"
done

# ✅ Show built .bin files
echo -e "\n🎉 Built firmware:"
find "$BIN_DIR" -name "*.bin"
