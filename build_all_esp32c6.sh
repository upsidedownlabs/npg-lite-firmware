#!/bin/bash

# Configuration
BOARD_FQBN="esp32:esp32:esp32c6"
OUTPUT_DIR="compiled_binaries"
LIBRARY_DIR="$HOME/Arduino/libraries"

# Install dependencies
echo "➡️ Installing dependencies..."
arduino-cli core update-index
arduino-cli core install esp32:esp32@3.2.0
arduino-cli lib install "IRremote"
arduino-cli lib install "BLE"

# Create fresh output directory
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

# Compilation function with error handling
compile_project() {
  local sketch_dir=$1
  local sketch_name=$(basename "$sketch_dir")
  local output_path="$OUTPUT_DIR/$sketch_name"

  mkdir -p "$output_path"

  echo -e "\n🛠️  Compiling: $sketch_name"
  if arduino-cli compile \
    --fqbn "$BOARD_FQBN" \
    --output-dir "$output_path" \
    --libraries "$LIBRARY_DIR" \
    --export-binaries \
    --warnings default \
    "$sketch_dir" > "$output_path/compile.log" 2>&1; then

    echo "✅ Success"
    echo "   Binary: $output_path/$sketch_name.ino.bin"
  else
    echo "❌ Failed"
    echo "   See $output_path/compile.log"

    # Attempt basic fixes for common errors
    if grep -q "'sum' was not declared" "$output_path/compile.log"; then
      echo "   ⚠️ Applying fix for undeclared 'sum' variable"
      sed -i '/double EnvelopeFilter::getEnvelope(double)/a double sum = 0;' "$sketch_dir/$sketch_name.ino"

      # Retry compilation after fix
      if arduino-cli compile \
        --fqbn "$BOARD_FQBN" \
        --output-dir "$output_path" \
        --libraries "$LIBRARY_DIR" \
        --export-binaries \
        "$sketch_dir" >> "$output_path/compile.log" 2>&1; then

        echo "   🔄 Retry successful after fixes!"
      else
        echo "   ❌ Still failing after fixes"
      fi
    fi
  fi
}

# Process all projects
find . -maxdepth 1 -type d -name "[!.]*" | while read sketch_dir; do
  if [ -f "$sketch_dir/$(basename "$sketch_dir").ino" ]; then
    compile_project "$sketch_dir"
  else
    echo "⚠️  Skipping non-Arduino folder: $(basename "$sketch_dir")"
  fi
done

echo -e "\n📊 Compilation summary:"
find "$OUTPUT_DIR" -name "*.bin" -exec ls -lh {} \;
#!/bin/bash

# Configuration
BOARD_FQBN="esp32:esp32:esp32c6"
OUTPUT_DIR="compiled_binaries"
LIBRARY_DIR="$HOME/Arduino/libraries"

# Create fresh output directory
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

# Compilation function with error handling
compile_project() {
  local sketch_dir=$1
  local sketch_name=$(basename "$sketch_dir")
  local output_path="$OUTPUT_DIR/$sketch_name"

  mkdir -p "$output_path"

  echo -e "\n🛠️  Compiling: $sketch_name"

  # Apply fixes for known issues
  sed -i 's/BandpowerResults r = { 0 };/BandpowerResults r = {0,0,0,0,0,0};/g' "$sketch_dir/$sketch_name.ino"
  sed -i 's/BandpowerResults smoothedPowers = { 0 };/BandpowerResults smoothedPowers = {0,0,0,0,0,0};/g' "$sketch_dir/$sketch_name.ino"
  sed -i 's/^.*NOTCH$//g' "$sketch_dir/$sketch_name.ino"

  if arduino-cli compile \
    --fqbn "$BOARD_FQBN" \
    --output-dir "$output_path" \
    --libraries "$LIBRARY_DIR" \
    --export-binaries \
    --warnings default \
    "$sketch_dir" > "$output_path/compile.log" 2>&1; then

    echo "✅ Success"
    echo "   Binary: $output_path/$sketch_name.ino.bin"
  else
    echo "❌ Failed"
    echo "   See $output_path/compile.log"

    # Attempt basic fixes for common errors
    if grep -q "'sum' was not declared" "$output_path/compile.log"; then
      echo "   ⚠️ Applying fix for undeclared 'sum' variable"
      sed -i '/double EnvelopeFilter::getEnvelope(double)/a double sum = 0;' "$sketch_dir/$sketch_name.ino"

      # Retry compilation after fix
      if arduino-cli compile \
        --fqbn "$BOARD_FQBN" \
        --output-dir "$output_path" \
        --libraries "$LIBRARY_DIR" \
        --export-binaries \
        "$sketch_dir" >> "$output_path/compile.log" 2>&1; then

        echo "   🔄 Retry successful after fixes!"
      else
        echo "   ❌ Still failing after fixes"
      fi
    fi
  fi
}

# Process all projects
find . -maxdepth 1 -type d -name "[!.]*" | while read sketch_dir; do
  if [ -f "$sketch_dir/$(basename "$sketch_dir").ino" ]; then
    compile_project "$sketch_dir"
  else
    echo "⚠️  Skipping non-Arduino folder: $(basename "$sketch_dir")"
  fi
done

echo -e "\n📊 Compilation summary:"
find "$OUTPUT_DIR" -name "*.bin" -exec ls -lh {} \;
