#!/bin/bash

# Configuration
BOARD_FQBN="esp32:esp32:esp32c6"
OUTPUT_DIR="compiled_binaries"
LIBRARY_DIR="$HOME/Arduino/libraries"
SKETCH_NAME="your_sketch_name"  # Change this to your specific sketch name

# Install dependencies (only if needed)
echo "➡️ Checking dependencies..."
arduino-cli core update-index
arduino-cli core install esp32:esp32@3.2.0
arduino-cli lib install "IRremote"
arduino-cli lib install "BLE"

# Create fresh output directory
echo "➡️ Preparing output directory..."
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

# Verify sketch exists
if [ ! -f "$SKETCH_NAME/$SKETCH_NAME.ino" ]; then
  echo "❌ Error: Sketch $SKETCH_NAME not found!"
  exit 1
fi

echo -e "\n🛠️  Compiling: $SKETCH_NAME"
if arduino-cli compile \
  --fqbn "$BOARD_FQBN" \
  --output-dir "$OUTPUT_DIR" \
  --libraries "$LIBRARY_DIR" \
  --export-binaries \
  --warnings all \
  "$SKETCH_NAME" > "$OUTPUT_DIR/compile.log" 2>&1; then

  echo "✅ Success"
  echo "   Binary: $OUTPUT_DIR/$SKETCH_NAME.ino.bin"
else
  echo "❌ Compilation failed"
  echo "   See $OUTPUT_DIR/compile.log for details"
  exit 1
fi

echo -e "\n📊 Build complete"
ls -lh "$OUTPUT_DIR/$SKETCH_NAME.ino.bin"